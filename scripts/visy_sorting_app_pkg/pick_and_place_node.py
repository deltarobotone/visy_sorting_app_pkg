#!/usr/bin/env python3

from visy_sorting_app_pkg.srv import PickAndPlace,PickAndPlaceResponse
from one_easy_protocol_pkg.srv import RobotMove,RobotMoveResponse
from one_easy_protocol_pkg.srv import RobotLight,RobotLightResponse,RobotLightRequest
from one_easy_protocol_pkg.srv import RobotExtMotor,RobotExtMotorResponse
from one_easy_protocol_pkg.srv import RobotGripper,RobotGripperResponse
from one_easy_protocol_pkg.srv import RobotConnect,RobotConnectResponse
from one_easy_protocol_pkg.srv import RobotDisconnect,RobotDisconnectResponse
import rospy

class PickAndPlaceNode:

    def __init__(self):
        self.__pick_and_place_srv = rospy.Service('pick_and_place', PickAndPlace, self.__pickandPlaceCB)
        self.__move_cli = rospy.ServiceProxy('ctrl_robot_move',RobotMove)
        self.__light_cli = rospy.ServiceProxy('ctrl_robot_light',RobotLight)
        self.__extmotor_cli = rospy.ServiceProxy('ctrl_robot_extmotor',RobotExtMotor)
        self.__gripper_cli = rospy.ServiceProxy('ctrl_robot_gripper',RobotGripper)
        self.__connect_cli = rospy.ServiceProxy('ctrl_robot_connect',RobotConnect)
        self.__disconnect_cli = rospy.ServiceProxy('ctrl_robot_disconnect',RobotDisconnect)
        self.__robotVel = 90.0
        self.__posCase1 = [25.0, 25.0, 100.0]
        self.__posCase2 = [0.0, 25.0, 100.0]
        self.__posCase3 = [-25.0, 25.0, 100.0]
        self.__posConveyor = [0.0, -22.0, 118.0]
        self.__posHome = [0.0, 0.0, 70.0]
        return None
    
    #Check services
    def checkServices(self):
        rospy.wait_for_service('ctrl_robot_move')
        rospy.wait_for_service('ctrl_robot_light')
        rospy.wait_for_service('ctrl_robot_extmotor')
        rospy.wait_for_service('ctrl_robot_gripper')
        rospy.wait_for_service('ctrl_robot_connect')
        rospy.wait_for_service('ctrl_robot_disconnect')
        return True

    #Pick and place
    def __pickandPlaceCB(self,req):
        if self.checkServices():
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


    def run(self):
        rospy.init_node("pick_and_place_node")
        rospy.spin()

if __name__ == "__main__":
    pickAndPlaceNode = PickAndPlaceNode()
    pickAndPlaceNode.checkServices()
    pickAndPlaceNode.run()
