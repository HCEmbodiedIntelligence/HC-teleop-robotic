# hc_adapter_x1

Simulated and hardware adapter components for the X1 humanoid dual-arm robot.

The simulator supports `instant_position_tracking`. When enabled by the X1
simulation profile it renders the command that has already passed the motion
backend and final RTC directly, avoiding an extra visualization-only smoothing
layer. The default remains velocity-limited for standalone use.
