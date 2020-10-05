#!/usr/bin/env python3
"""Grasp Planner Node for Vision System."""

from visy_sorting_app_pkg.srv import StartGraspPlanner,StartGraspPlannerResponse
from visy_sorting_app_pkg.srv import StopGraspPlanner,StopGraspPlannerResponse
from visy_sorting_app_pkg.srv import PickAndPlace,PickAndPlaceRequest
from visy_sorting_app_pkg.msg import GraspData
from visy_detector_pkg.msg import MetalChip
from visy_neopixel_pkg.srv import LightCtrl,LightCtrlRequest
from visy_neopixel_pkg.srv import PixelCtrl
from visy_neopixel_pkg.msg import Neopixel
import rospy
import math

class GraspPlannerNode:

    def __init__(self):
        """Class provides ROS Node to plan the grasp timing for Delta-Robot One based on metal chip message of metal chip detector node."""
        rospy.init_node("grasp_planner_node")
        #Service starts the grasp planner. This node is waiting for detected metal chips.
        self.__start_srv = rospy.Service('start_grasp_planner', StartGraspPlanner, self.__startCB)
        #Service stops the grasp planner.
        self.__stop_srv = rospy.Service('stop_grasp_planner', StopGraspPlanner, self.__stopCB)
        #Client to call the pick and place service at the right time after a grasp was detected.
        self.__pick_and_place_cli = rospy.ServiceProxy('pick_and_place',PickAndPlace)
        #Client to inform user about detection state (spin green) or detetcted grasp (full light in red, blue or yellow).
        self.__statusbar_cli = rospy.ServiceProxy('/status_bar_node/light_ctrl',LightCtrl)
        #Publishes grasp data including the time until grasp, colour of chip, number of detetcted chips, latency, velocity, last chip position, etc.
        self.__grasp_data_pub = rospy.Publisher('grasp_data', GraspData, queue_size=1)
        #Subscribes metal chip message for grasp time calculation using position, timestamps and colour.
        self.__metal_chip_sub = rospy.Subscriber("/metal_chip", MetalChip, self.__metalChipCB, queue_size=1)

        self.__graspDataMsg = GraspData()
        self.__start = False
        self.__calculated = False
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
        self.__robotGraspPosDistance = 0
        self.__roi_min = 0
        self.__roi_max = 0
        self.__case_1_hue_min = 0
        self.__case_1_hue_max = 0
        self.__case_2_hue_min = 0
        self.__case_2_hue_max = 0
        self.__case_3_hue_min = 0
        self.__case_3_hue_max = 0
        return None

    def __getParams(self):
        try:
            #Distance to robot in pixel.
            self.__robotGraspPosDistance = rospy.get_param('~robot_distance') #Pixels
            #Pixelposition where the detector uses the first metal chip data if it is detected.
            self.__roi_min = rospy.get_param('~roi_min') #Pixels
            #Pixelposition where the detector uses the last metal chip data if it is detected.
            self.__roi_max = rospy.get_param('~roi_max') #Pixels
            return True
        except Exception:
            rospy.logerr("get params failed at grasp_planner_node")
            return False

    #Reset
    def __reset(self):
        self.__metalChips = []
        self.__metalChipLast = MetalChip()

    #Start
    def __startCB(self,req):
        self.__start = True
        self.__reset()
        return StartGraspPlannerResponse("grasp planner is running...")

    #Stop
    def __stopCB(self,req):
        self.__start = False
        self.__reset()
        return StopGraspPlannerResponse("grasp planner stopped!")

    def __metalChipCB(self,metalChipMsg):

        pos = math.sqrt(pow(metalChipMsg.pos[0],2)+pow(metalChipMsg.pos[1],2))

        if self.__start == True and pos < self.__roi_max and pos > self.__roi_min:

            self.__lastPos = math.sqrt(pow(self.__metalChipLast.pos[0],2)+pow(self.__metalChipLast.pos[1],2))

            if pos > self.__lastPos:
                self.__metalChips.append(metalChipMsg)
                self.__metalChipLast = metalChipMsg
                self.__calculated = False
            else:
                rospy.loginfo("new chip detected")
                self.__reset()

        if pos >= self.__roi_max and self.__calculated == False:
            velocityArray = []
            metalChip1 = MetalChip()
            for metalChip2 in self.__metalChips:
                if metalChip1.pos[0] != 0 and metalChip1.pos[1] != 0:
                    durationTime = metalChip2.imageTime - metalChip1.imageTime
                    duration = durationTime.nsecs/1000000000
                    posChip1 = math.sqrt(pow(metalChip1.pos[0],2)+pow(metalChip1.pos[1],2))
                    posChip2 = math.sqrt(pow(metalChip2.pos[0],2)+pow(metalChip2.pos[1],2))
                    distance = posChip2 - posChip1
                    velocityTmp = distance/duration
                    velocityArray.append(velocityTmp)
                metalChip1 = metalChip2

            velocity = 0
            for velTmp in velocityArray:
                velocity = velocity + velTmp

            velocity = velocity/(len(velocityArray))

            rospy.loginfo("velocity")
            rospy.loginfo(velocity)
            self.__graspDataMsg.velocity=velocity

            rospy.loginfo("detected metal chips")
            rospy.loginfo(len(self.__metalChips))
            self.__graspDataMsg.detectedMetalChips=len(self.__metalChips)

            rospy.loginfo("last chip position")
            rospy.loginfo(self.__lastPos)
            self.__graspDataMsg.lastDetectedPosition=self.__lastPos

            case = 0
            hue = self.__metalChipLast.hue
            self.__graspDataMsg.hue=hue

            rospy.loginfo("hue")
            rospy.loginfo(hue)

            # case 1 red
            if ((hue >= 0 and hue < 10) or (hue <= 180 and hue > 160)): case = 1
            # case 2 yellow
            if (hue >= 15 and hue < 45): case = 2
            # case 3 blue
            if (hue >= 100 and hue < 130): case = 3

            if case == 1:
                self.__statusbar_cli(LightCtrlRequest.FULL,Neopixel(255,0,0,0))
                self.__graspDataMsg.colour="Red"
            if case == 2:
                self.__statusbar_cli(LightCtrlRequest.FULL,Neopixel(255,255,0,0))
                self.__graspDataMsg.colour="Yellow"
            if case == 3:
                self.__statusbar_cli(LightCtrlRequest.FULL,Neopixel(0,0,255,0))
                self.__graspDataMsg.colour="Blue"

            self.__now = rospy.get_rostime()
            self.__last = self.__metalChipLast.imageTime

            latency = self.__now-self.__last
            latencySec = (latency.nsecs/1000000000)

            rospy.loginfo("latency")
            rospy.loginfo(latencySec)
            self.__graspDataMsg.latencyMilliseconds=latencySec

            latencyDistance = velocity*latencySec

            rospy.loginfo("latency distance")
            rospy.loginfo(latencyDistance)
            self.__graspDataMsg.latencyDistance=latencyDistance

            time = (self.__robotGraspPosDistance - self.__lastPos - latencyDistance)/velocity

            rospy.loginfo("calculated time duration")
            rospy.loginfo(time)
            self.__graspDataMsg.delayTime=time

            self.__grasp_data_pub.publish(self.__graspDataMsg)

            rospy.sleep(time)

            rospy.loginfo("execute grasp")

            if case == 1: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_1)
            if case == 2: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_2)
            if case == 3: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_3)

            self.__reset()
            rospy.loginfo("reset")
            self.__statusbar_cli(LightCtrlRequest.SPIN_DOUBLE_TOP,Neopixel(0,255,0,0))
            self.__calculated = True

    def run(self):
        rate = rospy.Rate(10)
        if self.__getParams() == True:
            while not rospy.is_shutdown():
                rate.sleep()
        else:
            rospy.logerr("failed to start grasp_planner_node")

if __name__ == "__main__":
    graspPlannerNode = GraspPlannerNode()
    graspPlannerNode.run()
