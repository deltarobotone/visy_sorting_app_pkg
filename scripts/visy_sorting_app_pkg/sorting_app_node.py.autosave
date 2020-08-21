#!/usr/bin/env python3
"""Sorting Application Node for Vision System."""

from visy_sorting_app_pkg.srv import StartSorting,StartSortingResponse
from visy_sorting_app_pkg.srv import StopSorting,StopSortingResponse
from visy_sorting_app_pkg.srv import StartGraspPlanner
from visy_sorting_app_pkg.srv import StopGraspPlanner
from visy_sorting_app_pkg.srv import PickAndPlace
from visy_neopixel_pkg.srv import StatusBar,StatusBarRequest
from visy_neopixel_pkg.srv import LightRing,LightRingRequest
from one_easy_protocol_pkg.srv import RobotMove
from one_easy_protocol_pkg.srv import RobotLight,RobotLightRequest
from one_easy_protocol_pkg.srv import RobotExtMotor
from one_easy_protocol_pkg.srv import RobotGripper
from one_easy_protocol_pkg.srv import RobotConnect
from one_easy_protocol_pkg.srv import RobotDisconnect
from visy_detector_pkg.srv import DetectConveyorSystem
from visy_detector_pkg.srv import StartMetalChipDetector
from visy_detector_pkg.srv import StopMetalChipDetector
from visy_detector_pkg.msg import ConveyorSystem
import rospy

class SortingAppNode:

    def __init__(self):
        """Class provides ROS Node to control sorting applicaion using Vision System, Delta-Robot One and conveyor system."""
        self.__start_srv = rospy.Service('start_sorting', StartSorting, self.__startCB)
        self.__stop_srv = rospy.Service('stop_sorting', StopSorting, self.__stopCB)

        self.__move_cli = rospy.ServiceProxy('ctrl_robot_move',RobotMove)
        self.__light_cli = rospy.ServiceProxy('ctrl_robot_light',RobotLight)
        self.__extmotor_cli = rospy.ServiceProxy('ctrl_robot_extmotor',RobotExtMotor)
        self.__gripper_cli = rospy.ServiceProxy('ctrl_robot_gripper',RobotGripper)
        self.__connect_cli = rospy.ServiceProxy('ctrl_robot_connect',RobotConnect)
        self.__disconnect_cli = rospy.ServiceProxy('ctrl_robot_disconnect',RobotDisconnect)

        self.__lightring_cli = rospy.ServiceProxy('ctrl_light_ring',LightRing)
        self.__statusbar_cli = rospy.ServiceProxy('ctrl_status_bar',StatusBar)

        self.__detect_conveyor_cli = rospy.ServiceProxy('/detect_conveyor_system',DetectConveyorSystem)
        self.__start_detector_cli = rospy.ServiceProxy('/start_metalchip_detector',StartMetalChipDetector)
        self.__stop_detector_cli = rospy.ServiceProxy('/stop_metalchip_detector',StopMetalChipDetector)

        self.__start_grasp_planner_cli = rospy.ServiceProxy('start_grasp_planner',StartGraspPlanner)
        self.__stop_grasp_planner_cli = rospy.ServiceProxy('stop_grasp_planner',StopGraspPlanner)

        self.__pick_and_place_cli = rospy.ServiceProxy('pick_and_place',PickAndPlace)

        self.__conveyor_system_sub = rospy.Subscriber('conveyor_system_rect',ConveyorSystem,self.__conveyorSystemCB)

        self.__start = False
        self.__startUpState = False
        self.__startAppState = False
        self.__serviceState = False
        self.__ctrl_state = 0

        return None

    #Check services
    @classmethod
    def checkServices(cls):
        rospy.loginfo("check services")
        rospy.loginfo("#######################################")
        rospy.loginfo("move robot service...")
        rospy.wait_for_service('ctrl_robot_move')
        rospy.loginfo("robot light service...")
        rospy.wait_for_service('ctrl_robot_light')
        rospy.loginfo("external motor service...")
        rospy.wait_for_service('ctrl_robot_extmotor')
        rospy.loginfo("robot gripper service...")
        rospy.wait_for_service('ctrl_robot_gripper')
        rospy.loginfo("connect robot service...")
        rospy.wait_for_service('ctrl_robot_connect')
        rospy.loginfo("disconnect robot service...")
        rospy.wait_for_service('ctrl_robot_disconnect')
        rospy.loginfo("light ring service...")
        rospy.wait_for_service('ctrl_light_ring')
        rospy.loginfo("status bar service...")
        rospy.wait_for_service('ctrl_status_bar')
        rospy.loginfo("detect conveyor system service...")
        rospy.wait_for_service('/detect_conveyor_system')
        rospy.loginfo("metalchip detector services...")
        rospy.wait_for_service('/start_metalchip_detector')
        rospy.wait_for_service('/stop_metalchip_detector')
        rospy.loginfo("pick and place service...")
        rospy.wait_for_service('pick_and_place')
        rospy.loginfo("grasp planner services...")
        rospy.wait_for_service('start_grasp_planner')
        rospy.wait_for_service('stop_grasp_planner')
        rospy.loginfo("...ready!")
        rospy.loginfo("#######################################")
        return True

    #Reset
    def __reset(self):
        self.__start = False
        self.__startUpState = False
        self.__serviceState = False
        self.__startAppState = False
        self.__ctrl_state = 0
        return True

    #Start
    def __startCB(self,req):
        self.__reset()
        self.__serviceState = self.checkServices()
        self.__startUpState = self.__startUp()
        self.__start = True
        return StartSortingResponse("sorting application is running...")

    #Stop
    def __stopCB(self,req):
        self.__start = False
        self.__startUpState = False
        self.__startAppState = False
        rospy.loginfo("system shutdown")
        rospy.loginfo("#######################################")
        rospy.loginfo("stop grasp planner...")
        self.__stop_grasp_planner_cli("")
        rospy.loginfo("stop metal chip detector...")
        self.__stop_detector_cli("")
        rospy.loginfo("stop conveyor system...")
        self.__extmotor_cli(False,100.0)
        rospy.loginfo("disable light ring...")
        self.__lightring_cli(LightRingRequest.FULL,0,0,0,0)
        rospy.loginfo("disable status bar...")
        self.__statusbar_cli(StatusBarRequest.FULL,0,0,0,0)
        rospy.loginfo("disable robot light...")
        self.__light_cli(RobotLightRequest.OFF,100.0)
        rospy.loginfo("disconnect robot...")
        self.__disconnect_cli("")
        rospy.loginfo("#######################################")
        return StopSortingResponse("sorting application stopped!")

    #Startup
    def __startUp(self):
        rospy.loginfo("system startup")
        rospy.loginfo("#######################################")
        rospy.loginfo("connect robot...")
        res = self.__connect_cli("")
        rospy.loginfo(res.res)
        rospy.loginfo("move robot to start position...")
        res = self.__move_cli(0.0,0.0,70.0,50.0)
        rospy.loginfo(res.res)
        rospy.loginfo("enable robot light...")
        res = self.__light_cli(RobotLightRequest.WHITE,100.0)
        rospy.loginfo(res.res)
        rospy.loginfo("enable light ring...")
        self.__lightring_cli(LightRingRequest.FULL,0,0,0,5)
        rospy.loginfo("enable status bar...")
        self.__statusbar_cli(StatusBarRequest.FLOW_DOUBLE_TOP,0,0,0,255)
        rospy.loginfo("detect conveyor system...")
        autodetected = self.__detect_conveyor_cli("")
        rospy.loginfo("#######################################")
        self.__startUpState = True
        return True

    #StartApp
    def __startApp(self,autodetected):
        rospy.loginfo("start app")
        rospy.loginfo("#######################################")
        rospy.loginfo("start metal chip detector...")
        self.__start_detector_cli("")
        rospy.loginfo(autodetected)
        if autodetected == True:
            self.__statusbar_cli(StatusBarRequest.FLOW_DOUBLE_TOP,0,255,0,0)
        else:
            self.__statusbar_cli(StatusBarRequest.FLOW_DOUBLE_TOP,255,255,0,0)
        rospy.loginfo("start grasp planner...")
        self.__start_grasp_planner_cli("")
        rospy.loginfo("start conveyor system...")
        self.__extmotor_cli(True,200.0)
        self.__light_cli(RobotLightRequest.WHITE,100.0)
        rospy.loginfo("#######################################")
        return True

    def __conveyorSystemCB(self,data):
        if self.__startAppState == False and self.__start == True and self.__startUpState == True:
            self.__startAppState = True
            self.__startApp(data.autodetected)
        return True

    @classmethod
    def run(cls):
        rospy.init_node("sorting_app_node")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.checkServices()
    sortingAppNode.run()
