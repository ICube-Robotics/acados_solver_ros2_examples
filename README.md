# acados_solver_ros2_examples

Example application for the acados_solver_ros2 package.

__Credits:__ the simulated robot is adapted from the tutorial package [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos).

**For more information, please check the demo section of the [documentation of acados_driver_ros2](https://icube-robotics.github.io/acados_solver_ros2/).**

## Installation ##

1) Install dependencies

```bash
source /opt/ros/jazzy/setup.bash

mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ICube-Robotics/acados_solver_ros2_examples.git
vcs import . < acados_solver_ros2_examples/acados_solver_ros2_examples.repos

PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --ignore-src --from-paths . -y -r
```

2) Build and install ros2 packages

```bash
cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

3) Export and build the Acados solver plugins

```bash
cd ~/ros2_ws/
source install/setup.bash

cd src/acados_solver_ros2_examples
echo "y" | python3 ./example_acados_controller/script/export_acados_solver_plugin.py
cd ../..

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

4) Launch demo

```bash
cd ~/ros2_ws/
source install/setup.bash

ros2 launch example_acados_bringup launch_demo.launch.py
```

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Thibault Poignonec:__ [thibault.poignonec@gmail.com](mailto:thibault.poignonec@gmail.com), @github: [tpoignonec](https://github.com/ICube-Robotics)
