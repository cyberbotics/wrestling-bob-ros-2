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

        # we add a variable to wait for the robot to stabilise
        self.wait = True

        rclpy.init(args=None)
        self.__node = rclpy.create_node('nao_driver')

    def step(self):
        # Mandatory function to go to the next simulation step
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__robot.getTime() >= 1 and self.wait: # We wait a bit for the robot to stabilise
            # to play a motion from the library, we use the play() function as follows:
            self.__library.play('Forwards50')

            self.wait = False
