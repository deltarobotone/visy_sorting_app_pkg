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
        rospy.Subscriber("/metal_chip", MetalChip, self.__metalChipCB, queue_size=1)
        self.__start = False
        self.__calculated = False
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
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
                rospy.loginfo("reset 1")
                self.__reset()

        if pos >= 300 and self.__calculated == False:
            velocityArray = []
            metalChip1 = MetalChip()
            for metalChip2 in self.__metalChips:
                if metalChip1.pos[0] != 0 and metalChip1.pos[1] != 0:
                    durationTime = metalChip2.imageTime - metalChip1.imageTime
                    duration = durationTime.nsecs/10000000
                    #posChip1 = math.sqrt(pow(metalChip1.pos[0],2)+pow(metalChip1.pos[1],2))
                    #posChip2 = math.sqrt(pow(metalChip2.pos[0],2)+pow(metalChip2.pos[1],2))
                    posChip1 = metalChip1.pos[0]
                    posChip2 = metalChip2.pos[0]
                    distance = posChip2 - posChip1
                    velocityTmp = distance/duration
                    rospy.loginfo("%d,%d,%d,%d",metalChip1.pos[0],metalChip1.pos[1],metalChip2.pos[0],metalChip2.pos[1])
                    rospy.loginfo("%f,%f,%f",distance,duration,velocityTmp)
                    velocityArray.append(velocityTmp)
                metalChip1 = metalChip2

            velocity = 0
            for velTmp in velocityArray:
                velocity = velocity + velTmp

            rospy.loginfo("length velocity array")
            rospy.loginfo(len(velocityArray))
            rospy.loginfo("length metal chips array")
            rospy.loginfo(len(self.__metalChips))

            velocity = velocity/(len(velocityArray))
            velocity = velocity*100

            rospy.loginfo("last chip position")
            rospy.loginfo(self.__lastPos)

            rospy.loginfo("velocity")
            rospy.loginfo(velocity)

            case = 0
            hue = self.__metalChipLast.hue
            #case=1 rot
            if ((hue >= 0 and hue < 10) or (hue <= 180 and hue > 160)): case = 1
            #case=2 gelb
            if (hue >= 15 and hue < 45): case = 2
            #case=3 blau
            if (hue >= 100 and hue < 130): case = 3

            if case == 1: self.__statusbar_cli(StatusBarRequest.FULL,255,0,0,0)
            if case == 2: self.__statusbar_cli(StatusBarRequest.FULL,255,255,0,0)
            if case == 3: self.__statusbar_cli(StatusBarRequest.FULL,0,0,255,0)

            self.__now = rospy.get_rostime()
            self.__last = self.__metalChipLast.imageTime

            latency = self.__now-self.__last
            rospy.loginfo("latency")

            latencyMsec = (latency.nsecs/10000000)
            rospy.loginfo(latencyMsec)

            latencyDistance = velocity*latencyMsec

            rospy.loginfo("latency distance")
            rospy.loginfo(latencyDistance/100)

            time = (1200 - self.__lastPos - latencyDistance/100)/velocity

            rospy.loginfo("time")
            rospy.loginfo(time/3)

            rospy.sleep(time/3)

            if case == 1: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_1)
            if case == 2: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_2)
            if case == 3: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_3)

            self.__reset()
            rospy.loginfo("reset 2")
            self.__calculated = True

    def run(self):
        rospy.init_node("grasp_planner_node")
        rospy.spin()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.run()
