#!/usr/bin/env python3
"""Sorting Application Node for Vision System."""

import rospy
import actionlib
from visy_sorting_app_pkg.srv import StartSorting,StartSortingResponse
from visy_sorting_app_pkg.srv import StopSorting,StopSortingResponse
from visy_sorting_app_pkg.srv import StartGraspPlanner
from visy_sorting_app_pkg.srv import StopGraspPlanner
from visy_neopixel_pkg.srv import LightCtrl,LightCtrlRequest
from visy_neopixel_pkg.srv import PixelCtrl
from visy_neopixel_pkg.msg import Neopixel
from one_easy_protocol_pkg.srv import RobotMove
from one_easy_protocol_pkg.srv import RobotLight,RobotLightRequest
from one_easy_protocol_pkg.srv import RobotExtMotor
from one_easy_protocol_pkg.srv import RobotGripper
from one_easy_protocol_pkg.srv import RobotConnect
from one_easy_protocol_pkg.srv import RobotDisconnect
from visy_detector_pkg.srv import StartMetalChipDetector
from visy_detector_pkg.srv import StopMetalChipDetector
from visy_detector_pkg.msg import DetectConveyorAction, DetectConveyorGoal

class SortingAppNode:

    def __init__(self):
        """Class provides ROS Node to control sorting applicaion using Vision System, Delta-Robot One and conveyor system."""
        rospy.init_node("sorting_app_node")
        #Service to start sorting application for metal chips including Delta-Robot One, conveyor system and vision system.
        self.__start_srv = rospy.Service('start_sorting', StartSorting, self.__startCB)
        #Service to stop sorting application for metal chips.
        self.__stop_srv = rospy.Service('stop_sorting', StopSorting, self.__stopCB)
        #Client to move robot for sorting app.
        self.__move_cli = rospy.ServiceProxy('ctrl_robot_move',RobotMove)
        #Client to control robot light for sorting app.
        self.__light_cli = rospy.ServiceProxy('ctrl_robot_light',RobotLight)
        #Client to start and stop conveyor system.
        self.__extmotor_cli = rospy.ServiceProxy('ctrl_robot_extmotor',RobotExtMotor)
        #Client to control robot gripper.
        self.__gripper_cli = rospy.ServiceProxy('ctrl_robot_gripper',RobotGripper)
        #Client to connect robot.
        self.__connect_cli = rospy.ServiceProxy('ctrl_robot_connect',RobotConnect)
        #Client to disconnect robot.
        self.__disconnect_cli = rospy.ServiceProxy('ctrl_robot_disconnect',RobotDisconnect)
        #Client to control visy light ring. Activate light while detecting.
        self.__lightring_cli = rospy.ServiceProxy('/light_ring_node/light_ctrl',LightCtrl)
        #Client to control visy status bar.
        self.__statusbar_cli = rospy.ServiceProxy('/status_bar_node/light_ctrl',LightCtrl)
        #Client to control pixel of visy light ring for boot routine.
        self.__lightringpixel_cli = rospy.ServiceProxy('/light_ring_node/pixel_ctrl',PixelCtrl)
        #Client to control pixel of visy status bar for boot routine.
        self.__statusbarpixel_cli = rospy.ServiceProxy('/status_bar_node/pixel_ctrl',PixelCtrl)
        #Client to start conveyor system detetor. Action client for non blocking services.
        self.__detect_conveyor_cli = actionlib.SimpleActionClient('/detect_conveyor', DetectConveyorAction)
        #Client to start metal chip detector.
        self.__start_detector_cli = rospy.ServiceProxy('/start_metalchip_detector',StartMetalChipDetector)
        #Client to stop metal chip detector
        self.__stop_detector_cli = rospy.ServiceProxy('/stop_metalchip_detector',StopMetalChipDetector)
        #Client to start grasp planner.
        self.__start_grasp_planner_cli = rospy.ServiceProxy('start_grasp_planner',StartGraspPlanner)
        #Client to stop grasp planner.
        self.__stop_grasp_planner_cli = rospy.ServiceProxy('stop_grasp_planner',StopGraspPlanner)

        self.__start = False
        self.__startUpState = False
        self.__startAppState = False
        self.__serviceState = False
        self.__ctrl_state = 0
        return None

    #Check services
    def checkServices(self):
        rospy.loginfo("check services")
        rospy.loginfo("#######################################")
        rospy.loginfo("status bar services...")
        rospy.wait_for_service('/status_bar_node/light_ctrl')
        rospy.wait_for_service('/status_bar_node/pixel_ctrl')
        self.__statusbar_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,0))
        rospy.sleep(1)
        self.__statusbarpixel_cli(1,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("light ring services...")
        rospy.wait_for_service('/light_ring_node/light_ctrl')
        rospy.wait_for_service('/light_ring_node/pixel_ctrl')
        self.__lightring_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,0))
        self.__statusbarpixel_cli(2,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("move robot service...")
        rospy.wait_for_service('ctrl_robot_move')
        rospy.loginfo("robot light service...")
        rospy.wait_for_service('ctrl_robot_light')
        self.__statusbarpixel_cli(3,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("external motor service...")
        rospy.wait_for_service('ctrl_robot_extmotor')
        rospy.loginfo("robot gripper service...")
        rospy.wait_for_service('ctrl_robot_gripper')
        self.__statusbarpixel_cli(4,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("connect robot service...")
        rospy.wait_for_service('ctrl_robot_connect')
        rospy.loginfo("disconnect robot service...")
        rospy.wait_for_service('ctrl_robot_disconnect')
        self.__statusbarpixel_cli(5,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("detect conveyor system service...")
        self.__detect_conveyor_cli.wait_for_server()
        self.__statusbarpixel_cli(6,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("metalchip detector services...")
        rospy.wait_for_service('/start_metalchip_detector')
        rospy.wait_for_service('/stop_metalchip_detector')
        self.__statusbarpixel_cli(7,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("grasp planner services...")
        rospy.wait_for_service('start_grasp_planner')
        rospy.wait_for_service('stop_grasp_planner')
        self.__statusbarpixel_cli(8,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
        rospy.loginfo("...ready!")
        rospy.loginfo("#######################################")
        return True

    #LightRing start
    def lightRingStart(self):
        self.__lightring_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,0))
        self.__lightringpixel_cli(1,Neopixel(255,0,0,0),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(2,Neopixel(0,0,0,255),False)
        self.__lightringpixel_cli(12,Neopixel(0,0,0,255),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(3,Neopixel(0,0,0,255),False)
        self.__lightringpixel_cli(11,Neopixel(0,0,0,255),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(4,Neopixel(0,0,0,255),False)
        self.__lightringpixel_cli(10,Neopixel(0,0,0,255),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(5,Neopixel(0,0,0,255),False)
        self.__lightringpixel_cli(9,Neopixel(0,0,0,255),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(6,Neopixel(0,0,0,255),False)
        self.__lightringpixel_cli(8,Neopixel(0,0,0,255),False)
        rospy.sleep(0.3)
        self.__lightringpixel_cli(1,Neopixel(0,255,0,0),False)
        self.__lightringpixel_cli(7,Neopixel(0,0,255,0),False)
        rospy.sleep(0.3)
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
        self.__lightring_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,0))
        rospy.loginfo("disable status bar...")
        self.__statusbar_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,0))
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
        self.__lightring_cli(LightCtrlRequest.FULL,Neopixel(0,0,0,1))
        rospy.loginfo("detect conveyor system...")
        goal = DetectConveyorGoal()
        goal.loops = 10
        self.__detect_conveyor_cli.send_goal(goal,active_cb=self.__callback_active,feedback_cb=self.__callback_feedback,done_cb=self.__callback_done)
        rospy.loginfo("#######################################")
        self.__startUpState = True
        return True

    def __callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def __callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        self.__startApp()

    def __callback_feedback(self, feedback):
        rospy.loginfo("Detect conveyor:%s" % str(feedback))

    #StartApp
    def __startApp(self):
        rospy.loginfo("start app")
        rospy.loginfo("#######################################")
        rospy.loginfo("start metal chip detector...")
        self.__start_detector_cli("")
        rospy.loginfo("start grasp planner...")
        self.__start_grasp_planner_cli("")
        rospy.loginfo("start conveyor system...")
        self.__extmotor_cli(True,200.0)
        self.__light_cli(RobotLightRequest.WHITE,100.0)
        rospy.loginfo("#######################################")
        return True

    @classmethod
    def run(cls):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.checkServices()
    sortingAppNode.lightRingStart()
    sortingAppNode.run()
