import hashlib
import importlib
import json
from pathlib import Path
import sys

root=Path('/candidate')
expected=json.loads((root/'results/handshake-sources.json').read_text())
sys.path.insert(0,'/opt/undock_test_ws/install/patrolbot_navigation/lib/patrolbot_navigation')
result={}
for relative,digest in expected.items():
    path=Path(relative)
    if path.name=='patrolbot_joy_teleop.py': name='patrolbot_joy_teleop'
    elif 'patrolbot_dock_manager' in path.parts: name='patrolbot_dock_manager.'+path.stem
    else: name='patrolbot_bridge.'+path.stem
    module=importlib.import_module(name)
    installed=Path(module.__file__).resolve()
    actual=hashlib.sha256(installed.read_bytes()).hexdigest()
    assert actual==digest,(name,actual,digest)
    assert str(installed).startswith('/opt/undock_test_ws/install/'),installed
    result[name]={'installed_path':str(installed),'sha256':actual}
print(json.dumps({'installed_exact_imports':result},indent=2))
