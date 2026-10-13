"""Installed web adapter against the generated candidate action, offline only."""
import json
import os
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from patrolbot_interfaces.action import LocalizeOnMap
from patrolbot_web_bridge.commands import CommandExecutor

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert LocalizeOnMap.Goal().measured_pose is False
executor = CommandExecutor.__new__(CommandExecutor)
executor._PoseWithCovarianceStamped = PoseWithCovarianceStamped
executor._LocalizeOnMap = LocalizeOnMap
executor._node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
    now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
executor._active = None
executor._map_client = Mock()
executor._map_client.server_is_ready.return_value = True
executor._ack = Mock()
goal = {'map_id':'fixture','map_revision':'v1','x':2.,'y':3.,'yaw':.2}
for profile, expected in [(None,[.25,.25,.0685]),('measured',[.0009,.0009,.00030461742])]:
    executor._set_initial_pose('offline-request',goal,True,profile)
    sent = executor._map_client.send_goal_async.call_args.args[0]
    assert isinstance(sent,LocalizeOnMap.Goal)
    assert sent.measured_pose is (profile=='measured')
    assert [sent.pose.covariance[i] for i in (0,7,35)] == expected
    assert sent.map_id=='fixture' and sent.map_revision=='v1'
    assert sent.pose.pose.position.x==2. and sent.pose.pose.position.y==3.
executor._ack.assert_not_called()
print(json.dumps({'passed':['installed_adapter','generated_action','coarse_covariance_unchanged','measured_covariance'],'physical_acceptance':False}))
