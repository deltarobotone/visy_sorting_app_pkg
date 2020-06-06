#!/usr/bin/env python3

from visy_sorting_app_pkg.srv import *
from visy_detector_pkg.msg import *
import rospy

class SortingAppNode:

    def __init__(self):
        self.__start_srv = rospy.Service('start_grasp_planner', StartGraspPlanner, self.__startCB)
        self.__stop_srv = rospy.Service('stop_grasp_planner', StopGraspPlanner, self.__stopCB)
        self.__pick_and_place_cli = rospy.ServiceProxy('pick_and_place',PickAndPlace)
        rospy.Subscriber("/metal_chip", MetalChip, self.__metalChipCB)
        self.__start = False
        self.__metalChips = []
        self.__metalChipLast = MetalChip()
        return None

    #Reset
    def __reset():
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
        #metalChipMsg.hue
        #metalChipMsg.pos
        #metalChipMsg.vel
        if self.__start == True and self.__metalChips.len() < 10:
            if metalChipMsg.hue == metalChipLast.hue and metalChipMsg.pos > metalChipLast.pos:
                self.__metalChips.append(metalChipMsg)
                self.__metalChipLast = metalChipMsg
            else:
                self.__reset()

        if self.__metalChips.len() >= 10:
            vel = 0
            for metalChip in self.__metalChips:
                vel = vel + metalChip.vel
            vel = vel/self.__metalChips.len()

            time = (2000 - self.__metalChipLast.pos)/vel

            rospy.loginfo("vel")
            rospy.loginfo(vel)

            rospy.loginfo("time")
            rospy.loginfo(time)

            rospy.sleep(time)

    def run(self):
        rospy.init_node("grasp_planner_node")
        rospy.spin()

if __name__ == "__main__":
    sortingAppNode = SortingAppNode()
    sortingAppNode.run()
