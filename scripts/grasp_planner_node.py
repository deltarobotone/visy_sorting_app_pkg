#!/usr/bin/env python3

from visy_sorting_app_pkg.srv import *
from visy_detector_pkg.msg import *
import rospy

class SortingAppNode:

    def __init__(self):
        self.__start_srv = rospy.Service('start_grasp_planner', StartGraspPlanner, self.__startCB)
        self.__stop_srv = rospy.Service('stop_grasp_planner', StopGraspPlanner, self.__stopCB)
        self.__pick_and_place_cli = rospy.ServiceProxy('pick_and_place',PickAndPlace)
        rospy.Subscriber("/metal_chip", MetalChip, self.__metalChipCB, queue_size=1)
        self.__start = False
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
        return None

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
        rospy.loginfo("callback")
        if self.__start == True and len(self.__metalChips) < 30:
            if metalChipMsg.pos > self.__metalChipLast.pos:
                self.__metalChips.append(metalChipMsg)
                self.__metalChipLast = metalChipMsg
            else:
                self.__reset()

        if len(self.__metalChips) >= 30:
            vel = 0
            for metalChip in self.__metalChips:
                vel = vel + metalChip.vel
            vel = vel/len(self.__metalChips)

            time = ((4000 - self.__metalChipLast.pos)/(vel*1000.0))/10.0
            rospy.loginfo("pos")
            rospy.loginfo(self.__metalChipLast.pos)

            rospy.loginfo("vel")
            rospy.loginfo(vel)

            rospy.loginfo("time")
            rospy.loginfo(time)

            rospy.sleep(time)

            case = 0
            hue = self.__metalChipLast.hue
            #case=1 rot
            if ((hue >= 0 and hue < 10) or (hue <= 180 and hue > 160)): case = 1
            #case=2 gelb
            if (hue >= 15 and hue < 45): case = 2
            #case=3 blau
            if (hue >= 100 and hue < 130): case = 3


            if case == 1: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_1)
            if case == 2: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_2)
            if case == 3: self.__pick_and_place_cli(PickAndPlaceRequest.CASE_3)

            self.__reset()

    def run(self):
        rospy.init_node("grasp_planner_node")
        rospy.spin()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.run()
