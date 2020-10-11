# visy_sorting_app_pkg

ROS package for vision system (visy) includes the main application to grasp metal chips from conveyor system with Delta-Robot One 

![CI](https://github.com/deltarobotone/visy_sorting_app_pkg/workflows/CI/badge.svg?branch=master) [![Codacy Badge](https://app.codacy.com/project/badge/Grade/d43687c2dcf84a65bf7eb469d79e4eae)](https://www.codacy.com/gh/deltarobotone/visy_sorting_app_pkg?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=deltarobotone/visy_sorting_app_pkg&amp;utm_campaign=Badge_Grade)

This package is part of the Visy ROS workspace

[<img src="https://raw.githubusercontent.com/deltarobotone/image_database/master/visy_doc/visy_doc%20(1).PNG" width="1000">](https://raw.githubusercontent.com/deltarobotone/image_database/master/visy_doc/visy_doc%20(1).PNG)

- [visy_sorting_app_pkg](https://github.com/deltarobotone/visy_sorting_app_pkg)
- [visy_detector_pkg](https://github.com/deltarobotone/visy_detector_pkg)
- [one_easy_protocol_pkg](https://github.com/deltarobotone/one_easy_protocol_pkg)
- [one_easy_protocol_pkg](https://github.com/deltarobotone/one_easy_protocol_pkg)
- [visy_neopixel_pkg](https://github.com/deltarobotone/visy_neopixel_pkg)
- [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)

## Nodes

### grasp_planner_node.py

ROS node calculates time to stop conveyor system based on detected metal chips. If grasp was detected this node calls the pick and place service.

#### Advertised Services

##### start_grasp_planner (visy_sorting_app_pkg/StartGraspPlanner)

Service starts the grasp planner. This node is waiting for detected metal chips.

##### stop_grasp_planner (visy_sorting_app_pkg/StopGraspPlanner)

Service stops the grasp planner.

#### Parameters

##### ~robot_distance (Required)

Distance to robot in pixel.

##### ~roi_max (Required)

Pixelposition where the detector uses the last metal chip data if it is detected.

##### ~roi_min (Required)

Pixelposition where the detector uses the first metal chip data if it is detected.

#### Published topics

##### grasp_data (visy_sorting_app_pkg/GraspData)

Publishes grasp data including the time until grasp, colour of chip, number of detetcted chips, latency, velocity, last chip position, etc.

#### Service Clients

##### /status_bar_node/light_ctrl (visy_neopixel_pkg/LightCtrl)

Client to inform user about detection state (spin green) or detetcted grasp (full light in red, blue or yellow).

##### pick_and_place (visy_sorting_app_pkg/PickAndPlace)

Client to call the pick and place service at the right time after a grasp was detected.

#### Subscribed topics

##### /metal_chip (visy_detector_pkg/MetalChip)

Subscribes metal chip message for grasp time calculation using position, timestamps and colour.

### pick_and_place_node.py

ROS node to provide the pick and place routine for Detal-Robot One. A Service could be used to start the routine which stops the conveyor system directly, moves the gripper to the metal chip, grasp it and sort it into a specific case which could be set within this service.

#### Advertised Services

##### pick_and_place (visy_sorting_app_pkg/PickAndPlace)

Service to start the pick and place routine to sort the metal chips into a specific case of conveyor system.

#### Service Clients

##### ctrl_robot_extmotor (one_easy_protocol_pkg/RobotExtMotor)

Client to start and stop the conveyor system for successfull grasping.

##### ctrl_robot_gripper (one_easy_protocol_pkg/RobotGripper)

Clinet to control robot gripper for pick and place.

##### ctrl_robot_light (one_easy_protocol_pkg/RobotLight)

Client to change robot light for pick and place.

##### ctrl_robot_move (one_easy_protocol_pkg/RobotMove)

Client to move robot for pick and place.

### sorting_app_node.py

ROS node to control the whole sorting application. This is the main node. Check for all services. Connect the robot and start all services to detected conveyor system and metalchips and starts the grasp planner. This node is connected to the user interface so it is possible to start and stop the whole application with two buttons.

#### Action Clients

##### /detect_conveyor (visy_detector_pkg/DetectConveyorAction)

Client to start conveyor system detetor. Action client for non blocking services.

#### Advertised Services

##### start_sorting (visy_sorting_app_pkg/StartSorting)

Service to start sorting application for metal chips including Delta-Robot One, conveyor system and vision system.

##### stop_sorting (visy_sorting_app_pkg/StopSorting)

Service to stop sorting application for metal chips.

#### Service Clients

##### /light_ring_node/light_ctrl (visy_neopixel_pkg/LightCtrl)

Client to control visy light ring. Activate light while detecting.

##### /light_ring_node/pixel_ctrl (visy_neopixel_pkg/PixelCtrl)

Client to control pixel of visy light ring for boot routine.

##### /start_metalchip_detector (visy_detector_pkg/StartMetalChipDetector)

Client to start metal chip detector.

##### /stop_metalchip_detector (visy_detector_pkg/StopMetalChipDetector)

Client to stop metal chip detector

##### ctrl_robot_connect (one_easy_protocol_pkg/RobotConnect)

Client to connect robot.

##### ctrl_robot_disconnect (one_easy_protocol_pkg/RobotDisconnect)

Client to disconnect robot.

##### ctrl_robot_extmotor (one_easy_protocol_pkg/RobotExtMotor)

Client to start and stop conveyor system.

##### ctrl_robot_gripper (one_easy_protocol_pkg/RobotGripper)

Client to control robot gripper.

##### ctrl_robot_light (one_easy_protocol_pkg/RobotLight)

Client to control robot light for sorting app.

##### ctrl_robot_move (one_easy_protocol_pkg/RobotMove)

Client to move robot for sorting app.

##### start_grasp_planner (visy_sorting_app_pkg/StartGraspPlanner)

Client to start grasp planner.

##### stop_grasp_planner (visy_sorting_app_pkg/StopGraspPlanner)

Client to stop grasp planner.

