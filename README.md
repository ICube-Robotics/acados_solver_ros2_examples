# acados_solver_ros2_examples

Example application for the acados_solver_ros2 package.


**For more information, please check the [documentation](https://icube-robotics.github.io/acados_solver_ros2/) of `acados_driver_ros2`.**

## Installation ##

```bash
source /opt/ros/humble/setup.bash

mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ICube-Robotics/acados_solver_ros2_examples.git
vcs import . < acados_solver_ros2_examples/acados_solver_ros2_examples.repos
rosdep install --ignore-src --from-paths . -y -r

cd ..
colcon build && colcon build
source install/setup.bash
```

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Thibault Poignonec:__ [tpoignonec@unistra.fr](mailto:tpoignonec@unistra.fr), @github: [tpoignonec](https://github.com/ICube-Robotics)
