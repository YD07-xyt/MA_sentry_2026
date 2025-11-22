# MA sentry 2026

ubuntu22.04无法识别串口

```bash
sudo apt purge brltty 
```
```bash
ls /dev/tty*
```

## run

```bash
    source install/setup.bash
    ros2 launch planner_manager plan.launch.py
```
```bash
    source install/setup.bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
```bash
    source install/setup.bash
    ros2 launch fast_lio mapping.launch.py
```

```bash
    source install/setup.bash
    ros2 launch pb2025_robot_description robot_description_launch.py
```