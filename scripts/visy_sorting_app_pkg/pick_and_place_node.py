#!/usr/bin/env python3
"""Pick and place Node for Vision System."""

from visy_sorting_app_pkg.srv import PickAndPlace,PickAndPlaceResponse
from one_easy_protocol_pkg.srv import RobotMove
from one_easy_protocol_pkg.srv import RobotLight,RobotLightRequest
from one_easy_protocol_pkg.srv import RobotExtMotor
from one_easy_protocol_pkg.srv import RobotGripper
import rospy

class PickAndPlaceNode:

    def __init__(self):
        """Class provides ROS Node executes pick and place for metal chips using one ctrl node to sort the chips in the cases of conveyor system."""
        rospy.init_node("pick_and_place_node")
        #Service to start the pick and place routine to sort the metal chips into a specific case of conveyor system.
        self.__pick_and_place_srv = rospy.Service('pick_and_place', PickAndPlace, self.__pickandPlaceCB)
        #Client to move robot for pick and place.
        self.__move_cli = rospy.ServiceProxy('ctrl_robot_move',RobotMove)
        #Client to change robot light for pick and place.
        self.__light_cli = rospy.ServiceProxy('ctrl_robot_light',RobotLight)
        #Client to start and stop the conveyor system for successfull grasping.
        self.__extmotor_cli = rospy.ServiceProxy('ctrl_robot_extmotor',RobotExtMotor)
        #Clinet to control robot gripper for pick and place.
        self.__gripper_cli = rospy.ServiceProxy('ctrl_robot_gripper',RobotGripper)
        self.__robotVel = 90.0
        self.__posCase1 = [25.0, 25.0, 100.0]
        self.__posCase2 = [0.0, 25.0, 100.0]
        self.__posCase3 = [-25.0, 25.0, 100.0]
        self.__posConveyor = [0.0, -22.0, 118.0]
        self.__posHome = [0.0, 0.0, 70.0]
        return None

    #Check services
    @classmethod
    def checkServices(cls):
        rospy.wait_for_service('ctrl_robot_move')
        rospy.wait_for_service('ctrl_robot_light')
        rospy.wait_for_service('ctrl_robot_extmotor')
        rospy.wait_for_service('ctrl_robot_gripper')
        return True

    #Pick and place
    def __pickandPlaceCB(self,req):
        self.__extmotor_cli(False,100.0)
        self.__move_cli(self.__posHome[0],self.__posHome[1],self.__posHome[2],self.__robotVel)
        self.__gripper_cli(False)

        if req.case == req.CASE_1: self.__light_cli(RobotLightRequest.RED,100.0)
        if req.case == req.CASE_2: self.__light_cli(RobotLightRequest.YELLOW,100.0)
        if req.case == req.CASE_3: self.__light_cli(RobotLightRequest.BLUE,100.0)

        rospy.sleep(0.5)

        self.__move_cli(self.__posConveyor[0],self.__posConveyor[1],self.__posConveyor[2]-10.0,self.__robotVel)
        rospy.sleep(0.5)
        self.__move_cli(self.__posConveyor[0],self.__posConveyor[1],self.__posConveyor[2],self.__robotVel)
        self.__gripper_cli(True)
        rospy.sleep(0.5)
        self.__move_cli(self.__posConveyor[0],self.__posConveyor[1],self.__posConveyor[2]-10.0,self.__robotVel)
        self.__move_cli(self.__posHome[0],self.__posHome[1],self.__posHome[2],self.__robotVel)

        if req.case == req.CASE_1:
            self.__move_cli(self.__posCase1[0],self.__posCase1[1],self.__posCase1[2]-10.0,self.__robotVel)
            self.__move_cli(self.__posCase1[0],self.__posCase1[1],self.__posCase1[2],self.__robotVel)
            self.__gripper_cli(False)
            self.__move_cli(self.__posCase1[0],self.__posCase1[1],self.__posCase1[2]-10.0,self.__robotVel)
        if req.case == req.CASE_2:
            self.__move_cli(self.__posCase2[0],self.__posCase2[1],self.__posCase2[2]-10.0,self.__robotVel)
            self.__move_cli(self.__posCase2[0],self.__posCase2[1],self.__posCase2[2],self.__robotVel)
            self.__gripper_cli(False)
            self.__move_cli(self.__posCase2[0],self.__posCase2[1],self.__posCase2[2]-10.0,self.__robotVel)
        if req.case == req.CASE_3:
            self.__move_cli(self.__posCase3[0],self.__posCase3[1],self.__posCase3[2]-10.0,self.__robotVel)
            self.__move_cli(self.__posCase3[0],self.__posCase3[1],self.__posCase3[2],self.__robotVel)
            self.__gripper_cli(False)
            self.__move_cli(self.__posCase3[0],self.__posCase3[1],self.__posCase3[2]-10.0,self.__robotVel)

        self.__light_cli(RobotLightRequest.WHITE,100.0)
        rospy.sleep(0.5)
        self.__move_cli(self.__posHome[0],self.__posHome[1],self.__posHome[2],self.__robotVel)
        self.__extmotor_cli(True,200.0)

        return PickAndPlaceResponse(req.case)

    @classmethod
    def run(cls):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    pickAndPlaceNode = PickAndPlaceNode()
    pickAndPlaceNode.checkServices()
    pickAndPlaceNode.run()
