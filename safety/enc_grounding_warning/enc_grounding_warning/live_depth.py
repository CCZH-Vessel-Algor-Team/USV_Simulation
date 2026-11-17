"""Share persistent depth settings between the editor and all safety nodes."""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from enc_grounding_warning_msgs.msg import DepthSettings, DepthShoal
from enc_grounding_warning_msgs.srv import ConfigureDepth
from visualization_msgs.msg import MarkerArray

from .depth_settings import (
    DepthSettingsStore,
    build_provider,
    default_settings_path,
    initial_settings,
    validate_settings,
)
from .markers import depth_region_markers


def settings_from_msg(message):
    return validate_settings({
        "enabled": message.enabled,
        "base_depth_m": message.base_depth_m,
        "shoals": [
            {"x": s.x, "y": s.y, "radius": s.radius, "depth_m": s.depth_m}
            for s in message.shoals
        ],
    })


def settings_to_msg(settings):
    return DepthSettings(
        enabled=settings["enabled"], base_depth_m=settings["base_depth_m"],
        shoals=[DepthShoal(**region) for region in settings["shoals"]],
    )


class LiveDepthProvider:
    """Delegate queries to a replaceable grid; one authority persists edits."""

    def __init__(self, node, config_path, *, authority=False, on_change=None):
        self.node = node
        self.config_path = config_path
        self.on_change = on_change
        node.declare_parameter("depth_settings_file", "")
        path = node.get_parameter("depth_settings_file").value
        self.store = DepthSettingsStore(path or default_settings_path(node.get_namespace()))
        self.base_settings = initial_settings(config_path)
        self.settings = self.store.load(self.base_settings, config_path)
        self._provider = build_provider(config_path, self.settings)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if authority:
            self.publisher = node.create_publisher(DepthSettings, "safety/depth_settings", qos)
            self.marker_publisher = node.create_publisher(
                MarkerArray, "safety/depth_region_markers", qos
            )
            self.service = node.create_service(
                ConfigureDepth, "safety/configure_depth", self.configure
            )
            self._publish()
            node.get_logger().info(f"Persistent depth settings: {self.store.path}")
        else:
            self.subscription = node.create_subscription(
                DepthSettings, "safety/depth_settings", self.receive, qos
            )

    def __getattr__(self, name):
        return getattr(self._provider, name)

    def _publish(self):
        self.publisher.publish(settings_to_msg(self.settings))
        self.marker_publisher.publish(depth_region_markers(
            self.settings, self._provider.frame_id, self.node.get_clock().now().to_msg()
        ))

    def _apply(self, settings, provider):
        self.settings = settings
        self._provider = provider
        if self.on_change is not None:
            self.on_change()

    def receive(self, message):
        try:
            settings = settings_from_msg(message)
            if settings != self.settings:
                self._apply(settings, build_provider(self.config_path, settings))
        except Exception as error:
            self.node.get_logger().error(f"Cannot apply depth settings: {error}")

    def configure(self, request, response):
        response.success = True
        response.message = (
            "自定义水深已启用" if self.settings["enabled"] else "当前使用启动配置中的水深"
        )
        if request.apply:
            try:
                settings = (
                    settings_from_msg(request.settings) if request.settings.enabled
                    else initial_settings(self.config_path)
                )
                settings["base_depth_m"] = self.base_settings["base_depth_m"]
                provider = build_provider(self.config_path, settings)
                self.store.save(settings)
                self._apply(settings, provider)
                self._publish()
                response.message = (
                    "已保存并应用；后续启动自动恢复" if settings["enabled"]
                    else "已恢复启动配置；后续启动继续使用启动配置"
                )
            except Exception as error:
                response.success = False
                response.message = f"水深设置未保存：{error}"
        response.settings = settings_to_msg(self.settings)
        return response
