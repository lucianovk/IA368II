# Bringup package for the didactic vacuum robot.
#
# This package intentionally uses:
# - LifecycleNode (rclpy) for all nodes (managed by Nav2 lifecycle manager)
# - An "operational state" published by behaviour_server so that nodes can
#   enable/disable their behavior without changing lifecycle state.
