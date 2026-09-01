# hc_dashboard

Independent operator UI for the decomposed HC runtime. The ROS bridge only
subscribes to typed telemetry and calls the safety enable/reset services; it
never publishes command candidates or final actuator commands.

The browser receives a 4 Hz aggregate snapshot over WebSocket. High-rate ROS
messages remain inside the bridge and are reduced to stream frequency, age,
controller state and latest per-group values.

The operator console is split into three focused views:

- **Overview** shows the five-stage VR-to-robot pipeline, safety ownership,
  controller inputs and authoritative command groups.
- **Joint monitor** aligns command and feedback values in radians/degrees and
  highlights tracking error.
- **Diagnostics** reports stream rate, age, maximum gap, Cartesian targets and
  motion-backend candidates.

```bash
./bootstrap_colcon.sh deps
./bootstrap_colcon.sh build
source install/setup.bash
ros2 run hc_dashboard dashboard --ros-args \
  -r __ns:=/robots/x1 \
  -p robot_id:=x1 -p profile:=x1 -p mode:=sim -p port:=7877
```

Open `http://127.0.0.1:7877/dashboard/`. The standard bringup launch starts the
dashboard by default; pass `start_dashboard:=false` for a control-only launch.
