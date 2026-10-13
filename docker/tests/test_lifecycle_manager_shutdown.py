"""A launch signal may have already shut down the ROS context before cleanup."""
import importlib.util
from pathlib import Path
import sys
import types
from unittest.mock import Mock, patch


def test_cleanup_after_context_shutdown():
    ros=types.ModuleType('rclpy')
    ros.init=Mock();ros.ok=Mock(return_value=False)
    ros.shutdown=Mock(side_effect=RuntimeError('context already shut down'))
    ros.try_shutdown=Mock()
    node_module=types.ModuleType('rclpy.node');node_module.Node=object
    messages=types.ModuleType('lifecycle_msgs.msg')
    messages.State=object;messages.Transition=object
    services=types.ModuleType('lifecycle_msgs.srv')
    services.ChangeState=object;services.GetState=object
    modules={'rclpy':ros,'rclpy.node':node_module,'lifecycle_msgs':types.ModuleType('lifecycle_msgs'),
             'lifecycle_msgs.msg':messages,'lifecycle_msgs.srv':services}
    path=Path(__file__).resolve().parents[2]/'ros2_ws/src/patrolbot-launch/lifecycle_mgr.py'
    spec=importlib.util.spec_from_file_location('smoother_lifecycle',path)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules): spec.loader.exec_module(module)
    node=Mock()
    with patch.object(module,'LifecycleManager',return_value=node): module.main()
    node.destroy_node.assert_called_once()
    ros.try_shutdown.assert_called_once()
    ros.shutdown.assert_not_called()
