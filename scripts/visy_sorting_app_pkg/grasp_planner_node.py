#!/usr/bin/env python3

from visy_sorting_app_pkg.srv import *
from visy_detector_pkg.msg import *
from visy_neopixel_pkg.srv import *
import rospy
import math

class SortingAppNode:

    def __init__(self):
        self.__start_srv = rospy.Service('start_grasp_planner', StartGraspPlanner, self.__startCB)
        self.__stop_srv = rospy.Service('stop_grasp_planner', StopGraspPlanner, self.__stopCB)
        self.__pick_and_place_cli = rospy.ServiceProxy('pick_and_place',PickAndPlace)
        self.__statusbar_cli = rospy.ServiceProxy('ctrl_status_bar',StatusBar)
        self.__grasp_data_pub = rospy.Publisher('grasp_data', GraspData, queue_size=1)

        rospy.Subscriber("/metal_chip", MetalChip, self.__metalChipCB, queue_size=1)

        self.__graspDataMsg = GraspData()
        self.__start = False
        self.__calculated = False
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
        self.__robotGraspPosDistance = 590 #Pixels
        return None

    #Reset
    def __reset(self):
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
        self.__statusbar_cli(StatusBarRequest.FLOW_DOUBLE_TOP,0,255,0,0)

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

        if self.__start == True and pos < 300 and pos > 50:

            self.__lastPos = math.sqrt(pow(self.__metalChipLast.pos[0],2)+pow(self.__metalChipLast.pos[1],2))

            if pos > self.__lastPos:
                self.__metalChips.append(metalChipMsg)
                self.__metalChipLast = metalChipMsg
                self.__calculated = False
            else:
                rospy.loginfo("new chip detected")
                self.__reset()

        if pos >= 300 and self.__calculated == False:
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

            rospy.loginfo("detected metal chips")
            rospy.loginfo(len(self.__metalChips))
            self.__graspDataMsg.detectedMetalChips=len(self.__metalChips)

            velocity = velocity/(len(velocityArray))

            rospy.loginfo("last chip position")
            rospy.loginfo(self.__lastPos)
            self.__graspDataMsg.lastDetectedPosition=self.__lastPos

            rospy.loginfo("velocity")
            rospy.loginfo(velocity)
            self.__graspDataMsg.velocity=velocity


            case = 0
            hue = self.__metalChipLast.hue
            self.__graspDataMsg.hue=hue

            # case 1 red
            if ((hue >= 0 and hue < 10) or (hue <= 180 and hue > 160)): case = 1
            # case 2 yellow
            if (hue >= 15 and hue < 45): case = 2
            # case 3 blue
            if (hue >= 100 and hue < 130): case = 3

            if case == 1:
                self.__statusbar_cli(StatusBarRequest.FULL,255,0,0,0)
                self.__graspDataMsg.colour="Red"
            if case == 2:
                self.__statusbar_cli(StatusBarRequest.FULL,255,255,0,0)
                self.__graspDataMsg.colour="Yellow"
            if case == 3:
                self.__statusbar_cli(StatusBarRequest.FULL,0,0,255,0)
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
            self.__calculated = True

    def run(self):
        rospy.init_node("grasp_planner_node")
        rospy.spin()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.run()
