"""Fresh persistence paths for a whole CCS simulation launch, never a node restart."""
from datetime import datetime, timezone
from pathlib import Path
import re
import tempfile


def new_autonomy_session(root):
    """Create a unique retained directory without touching any previous session."""
    if not str(root).strip():
        raise ValueError('CCS session root must not be empty')
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return Path(tempfile.mkdtemp(prefix='ccs_' + stamp + '_', dir=str(root)))


def autonomy_session_paths(session, namespace):
    """Derive both stores together; repeated calls reuse the same vessel data."""
    parts = namespace.strip('/').split('/')
    if not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part) for part in parts):
        raise ValueError('invalid autonomy session vessel namespace')
    vessel = Path(session).resolve().joinpath(*parts)
    vessel.mkdir(parents=True, exist_ok=True)
    return {'autonomy_database': str(vessel / 'tasks.sqlite3'),
            'autonomy_rules_checkpoint': str(vessel / 'maritime.json')}
