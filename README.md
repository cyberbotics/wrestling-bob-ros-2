# Humanoid Robot Wrestling Controller Example

## ⚠️ WARNING ⚠️

The current Vulcanexus images used in this example are based on Webots R2023a and hence won't work with the current version of the IROS competition.
Please wait until Vulcanexus integrates Webots R2023b (which should happen soon) before you can participate in the competition with a ROS 2 controller.
Alternatively, you can build your own ROS 2 + Webots R2023b docker image and use it.

[![webots.cloud - Competition](https://img.shields.io/badge/webots.cloud-Competition-007ACC)][1]

## Bob ROS 2 controller

This ROS 2 controller is a simple example of how to use the Motion_library class from the [motion.py](./controllers/utils/motion.py) module to load all motion files from the folder [motions](./controllers/motions) and play one of them.

It beats [Alice](https://github.com/cyberbotics/wrestling-alice) by moving forwards and therefore having a higher coverage.

The Docker image used in the competition is a lightweight iron image that does not have colcon installed so we pre-build the package using the [build_controller.sh](./controllers/build_controller.sh) script.

Here is the [nao_controller.py](./controllers/participant/participant/nao_controller.py) file using Webots' [Robot API](https://cyberbotics.com/doc/reference/robot):

``` Python
import rclpy

# This is a workaround so that Webots' Python controller classes can be used
# in this case, we need it to import MotionLibrary which needs the Motion class
import os
from ament_index_python.packages import get_package_prefix
os.environ['WEBOTS_HOME'] = get_package_prefix('webots_ros2_driver')
from utils.motion_library import MotionLibrary

class NaoDriver:
    def init(self, webots_node, properties):
        # we get the robot instance from the webots_node
        self.__robot = webots_node.robot
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.__library = MotionLibrary()

        # we initialize the shoulder pitch motors using the Robot.getDevice() function:
        self.__RShoulderPitch = self.__robot.getDevice("RShoulderPitch")
        self.__LShoulderPitch = self.__robot.getDevice("LShoulderPitch")

        # to control a motor, we use the setPosition() function:
        self.__RShoulderPitch.setPosition(1.3)
        self.__LShoulderPitch.setPosition(1.3)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
        # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

        rclpy.init(args=None)
        self.__node = rclpy.create_node('nao_driver')

    def step(self):
        # Mandatory function to go to the next simulation step
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__robot.getTime() == 1: # We wait a bit for the robot to stabilise
            # to play a motion from the library, we use the play() function as follows:
            self.__library.play('Forwards50')

```

And here is the [ROS 2 launch file](./controllers/participant/launch/robot_launch.py) using [webots_ros2_driver](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_driver):

``` Python
import os
import pathlib
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('participant')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_controller.urdf')).read_text()

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': os.environ['WEBOTS_CONTROLLER_URL']},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        my_robot_driver,
    ])
```

Here is the [Dockerfile](./controllers/Dockerfile) used by the controller:

``` Dockerfile
# We use the eprosima/vulcanexus:iron-simulation image because it is light
# It has ROS2 and webots_ros2 installed
FROM eprosima/vulcanexus:iron-simulation-3.0.0

WORKDIR /usr/local/webots-project/controllers/participant

# Copies all the files of the controllers folder into the docker container
RUN mkdir -p /usr/local/webots-project/controllers
COPY . /usr/local/webots-project/controllers

# The eprosima/vulcanexus:iron-simulation Docker image does not have colcon installed
# We install it and build the participant package
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build

# Environment variable needed to connect to Webots instance
ARG WEBOTS_CONTROLLER_URL
ENV WEBOTS_CONTROLLER_URL=${WEBOTS_CONTROLLER_URL}
ENV USER=root

# Source the ROS iron setup file and run the participant package
CMD . /opt/ros/iron/setup.sh && . /usr/local/webots-project/controllers/participant/install/setup.sh && ros2 launch participant robot_launch.py
```

[Charlie](https://github.com/cyberbotics/wrestling-charlie) is a more advanced robot controller able to win against Bob.

[1]: https://webots.cloud/run?version=R2022b&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwrestling%2Fblob%2Fmain%2Fworlds%2Fwrestling.wbt&type=competition "Leaderboard"
