# MyHand

To use MyHand driver with autograsp function

`mkdir ws_Mia`

`cd ws_Mia`

`catkin init`


`sudo apt install libserial-dev`

`mkdir src`

`git clone src -b autograsp https://github.com/AnlageM/MyHand.git`

`catkin_make_isolated`

`souce devel_isolated/setup.bash`

`roscore`

In a new sourced terminal:

`rosrun mia_hand_driver mia_hand_node`


# Commands

You can easily command the whole hand by setting:

- grasp_type: "C" for cylindrical, "P" for pinch, "L" for lateral, "S" for spherical and "T" for tridigital;
- grasp_mode: "a" for full autoclose, "A" for full autoopen, "M" for partial hand opening;
- step_time: in case of full opening or closing, you set the duration (0-999); in case of partial opening or closing, you set the percentage of closing (range:0-99);

Examples:
Enter those commands in a new sourced terminal.

CYL CLOSE

`rostopic pub /Autograsp/command mia_hand_msgs/AutoGraspData '{grasp_type: "C", grasp_mode: "a", step_time: 80}'`

NOTE: Time here ranges from 0 to 999;

CYL OPEN

`rostopic pub /Autograsp/command mia_hand_msgs/AutoGraspData '{grasp_type: "C", grasp_mode: "A", step_time: 80}'`

NOTE: Time here ranges from 0 to 999;

CYL MANUAL ~40%

`rostopic pub /Autograsp/command mia_hand_msgs/AutoGraspData '{grasp_type: "C", grasp_mode: "M", step_time: 40}'`

NOTE: Step ranges from 0 to 99;
