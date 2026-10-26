"""Bounded scene queries using one isolated, persistent Gazebo connection."""

import multiprocessing
import threading
import time


def _scene_worker(connection):
    """Keep blocking Transport calls and their GIL waits outside the ROS process."""
    try:
        import gz.transport13 as transport
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene

        node = transport.Node()
        while True:
            request = connection.recv()
            if request is None:
                return
            sequence, world, deadline = request
            remaining_ms = int((deadline - time.monotonic()) * 1000)
            names = None
            if remaining_ms > 0:
                try:
                    # Initialize service discovery before entering the blocking
                    # Python request binding; otherwise a cold client can keep
                    # timing out even though the Gazebo service is available.
                    node.service_info(f'/world/{world}/scene/info')
                    remaining_ms = int((deadline - time.monotonic()) * 1000)
                    if remaining_ms > 0:
                        success, scene = node.request(
                            f'/world/{world}/scene/info', Empty(), Empty, Scene,
                            remaining_ms)
                        if success and scene.ListFields() and time.monotonic() <= deadline:
                            names = {model.name for model in scene.model}
                except Exception:
                    # A failed read is unknown, never evidence of an empty scene.
                    pass
            connection.send((sequence, names))
    except (EOFError, BrokenPipeError, OSError, ImportError, KeyboardInterrupt):
        pass
    finally:
        connection.close()


class SceneInventoryClient:
    """Serialize fresh queries without blocking pose feedback or reconnecting each tick.

    An overdue reply is drained before another request and is never used as a
    fresh inventory. A worker that exceeds its request deadline by a further
    second is discarded; ordinary service timeouts retain the connection.
    """

    def __init__(self):
        # Never fork a process that already has ROS / Gazebo Transport threads.
        self._context = multiprocessing.get_context('spawn')
        self._lock = threading.Lock()
        self._process = None
        self._connection = None
        self._pending = None
        self._sequence = 0
        self._closed = False
        self._retry_after = 0.0

    def _stop_worker(self):
        connection, self._connection = self._connection, None
        process, self._process = self._process, None
        self._pending = None
        if connection is not None:
            connection.close()
        if process is not None:
            if process.is_alive():
                process.terminate()
            process.join(timeout=0.2)
            if process.is_alive():
                process.kill()
                process.join(timeout=0.2)
            if not process.is_alive():
                process.close()

    def _start_worker(self):
        parent, child = self._context.Pipe()
        process = self._context.Process(
            target=_scene_worker, args=(child,), name='gazebo_scene_inventory', daemon=True)
        try:
            process.start()
        except Exception:
            parent.close()
            child.close()
            process.close()
            raise
        child.close()
        self._connection, self._process = parent, process

    def _receive(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not self._connection.poll(remaining):
            return None
        sequence, names = self._connection.recv()
        expected = self._pending[0]
        self._pending = None
        if sequence != expected or time.monotonic() > deadline:
            return None
        return names

    def model_names(self, world_name, timeout_ms=1000):
        if timeout_ms <= 0:
            return None
        deadline = time.monotonic() + timeout_ms / 1000.0
        if not self._lock.acquire(timeout=max(0.0, deadline - time.monotonic())):
            return None
        try:
            if self._closed or time.monotonic() < self._retry_after:
                return None
            if self._process is not None and not self._process.is_alive():
                self._stop_worker()
            if self._pending is not None:
                # Wait for and discard an earlier timed-out query. Never queue
                # unbounded requests behind an unresponsive Transport call.
                if time.monotonic() > self._pending[1] + 1.0:
                    self._stop_worker()
                else:
                    self._receive(deadline)
                    if self._pending is not None:
                        return None
            if time.monotonic() >= deadline:
                return None
            if self._process is None:
                self._start_worker()
            if time.monotonic() >= deadline:
                return None
            self._sequence += 1
            self._pending = (self._sequence, deadline)
            self._connection.send((self._sequence, world_name, deadline))
            return self._receive(deadline)
        except (EOFError, BrokenPipeError, OSError, ValueError):
            self._stop_worker()
            self._retry_after = time.monotonic() + 1.0
            return None
        finally:
            self._lock.release()

    def close(self):
        with self._lock:
            self._closed = True
            if self._connection is not None and self._pending is None:
                try:
                    self._connection.send(None)
                    self._process.join(timeout=0.2)
                except (BrokenPipeError, OSError):
                    pass
            self._stop_worker()
