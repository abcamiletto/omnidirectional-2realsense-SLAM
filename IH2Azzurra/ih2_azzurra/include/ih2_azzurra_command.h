#include "libserial/SerialPort.h"
#include "ros/ros.h"

#include "trajectory_msgs/JointTrajectory.h"

#include <thread> 
#include <chrono>

class IH2Command
{
    public:
        IH2Command();

        ~IH2Command();
 

        //void setSerial(std::shared_ptr<LibSerial::SerialPort> serial_port);
        
        void sendPositionCommand(std::vector<std::string> joint_names, std::vector<trajectory_msgs::JointTrajectoryPoint> pos);                         
        // rostopic pub /position_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["THU_A", "IND_F"], points:[ {positions: [23.0]}, {positions: [17.0]} ] }'
        // rostopic pub /position_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["THU_A", "THU_F", "IND_F", "MID_F", "RIL_F"], points:[ {positions: [255.0]}, {positions: [255.0]}, {positions: [255.0]},{positions: [0.0]}, {positions: [255.0]} ] }'

        void sendAutoGraspCommand(std::string graspType, int graspStep, int graspForce);
        // rostopic pub /autograsp_controller/command ih2_azzurra/AutoGrasp '{grasp_type: "CYL", grasp_step: 23, grasp_force: 50 }'
        // grasp force gestisce forza finale del grasp. Step sopra il 110 chiudono o aprono più velocemente, step tra 30 e 90 aprono di un tot.
        // facciamo che si parte da stato casuale (2) va impostata preshape (0), ho chiuso (1), ho riaperto (2) 

        void enableStreaming(int streaming_mode);
        // rostopic pub /streaming_enabler/command std_msgs/Int8 '{data: 6}'

        //void streamPublisher();


        std::shared_ptr<LibSerial::SerialPort> ih2_serial1_;

    private:
        int ag_status_ = 102;
        std::string ag_grasp_ = "RND";

        bool streaming_status_ = false;
        int remap_lohi(int val, int low_l, int high_l);

        std::string rx_msg_;
        //char readStream();

};