#include "ros/ros.h"
#include "libserial/SerialPort.h"
#include "boost/bind.hpp"
#include "ih2_azzurra_command.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "ih2_azzurra/AutoGrasp.h"
#include "std_msgs/Int8.h"

#include <iostream>
#include <string>
#include <thread> 
#include <chrono>

class IH2Driver //:  public ros::NodeHandle     
// IH2Driver è una classe derivata, costruita a partire dalla classe ros:NodeHandle
// quindi eredita le caratteristiche public (e protected) della classe base
{
    public:

        IH2Driver(ros::NodeHandle nh, std::string serial_port_name);

        ~IH2Driver();   



    private:
        
        ros::NodeHandle nh_;

        ros::Subscriber pos_sub;
        ros::Subscriber aug_sub;
        ros::Subscriber str_sub;     

        ros::Publisher str_pub;

        bool connected_;    
        std::shared_ptr<LibSerial::SerialPort> ih2_serial_;
        std::thread serial_read_thread_;
        IH2Command ih2_command_;


        void connect(std::string serial_port_name);
        void disconnect();

        void pubSubInit();
        void ctrlCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg, int controllerID);
        void auGrCallback(const ih2_azzurra::AutoGrasp::ConstPtr& msg, int controllerID);
        void enStCallback(const std_msgs::Int8::ConstPtr& msg);
        void serialPoll();
        void streamPublisher();
        char readStream();
};