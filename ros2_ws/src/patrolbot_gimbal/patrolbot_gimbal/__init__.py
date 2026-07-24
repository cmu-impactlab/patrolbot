"""PatrolBot camera gimbal control.

Isolated from the robot velocity chain by design: nothing in this package can
publish a command that reaches twist_mux or the bridge. See
test/test_isolation_contract.py, which enforces that mechanically.
"""
