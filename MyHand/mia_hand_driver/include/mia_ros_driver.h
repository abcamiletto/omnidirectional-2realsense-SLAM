#ifndef MIA_ROS_DRIVER_H
#define MIA_ROS_DRIVER_H

#include "mia_finger_driver.h"
#include "mia_autograsp_driver.h"
#include "ros/ros.h"

#include "mia_hand_msgs/FingersData.h"
#include "mia_hand_msgs/FingersStrainGauges.h"
#include "mia_hand_msgs/AutoGraspData.h"

#include "std_msgs/Float64.h"

#include "mia_hand_msgs/str_req.h"
#include "mia_hand_msgs/str_resp.h"
#include "std_srvs/Empty.h"

#include <thread>


class MiaROSDriver: public ros::NodeHandle
{
	public:
	
		MiaROSDriver();
		
		~MiaROSDriver();      
	
	
	private:

		//subscribers
		ros::Subscriber thu_pos; 
		ros::Subscriber ind_pos;
		ros::Subscriber mrl_pos;
		
		ros::Subscriber thu_spe;
		ros::Subscriber ind_spe;
		ros::Subscriber mrl_spe;
		
		ros::Subscriber thu_for;
		ros::Subscriber ind_for;
		ros::Subscriber mrl_for;

		ros::Subscriber grasp;
		

		//services
		ros::ServiceServer open_serial;
		ros::ServiceServer close_serial;
		
		ros::ServiceServer pos_stream_on;
		ros::ServiceServer spe_stream_on;
		ros::ServiceServer ana_stream_on;
		ros::ServiceServer cur_stream_on;
		
		ros::ServiceServer pos_stream_off;
		ros::ServiceServer spe_stream_off;
		ros::ServiceServer ana_stream_off;
		ros::ServiceServer cur_stream_off;
		
		ros::ServiceServer stream_off;
		
		
		//publisher
		ros::Publisher fingers_pos;
		ros::Publisher fingers_spe;
		ros::Publisher fingers_cur;
		ros::Publisher fingers_sg;

		std::shared_ptr<MiaSerialPort> serial_;
		std::thread serial_read_thread_;
		
		MiaFingerDriver thumb_;
		MiaFingerDriver index_;
		MiaFingerDriver mrl_;

		MiaAutograspDriver grasp_;
	
    void connect(std::string port_name);
    void disconnect();  
    
    bool connected_;

		void initializePublishers();
		void initializeSubscribers();
		void initializeServices();		

		void serialPoll();
		void publishStreamData(std::string stream_msg);


		//Subscribers Callback Functions
		
		void thuPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void indPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void mrlPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
			
		void thuSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void indSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void mrlSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		
		void thuForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void indForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);
		void mrlForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg);

		void graspSubscriberCallback(const mia_hand_msgs::AutoGraspData::ConstPtr& msg);


		//Services Callback Functions
		
		bool frmwRevServiceCallback(mia_hand_msgs::str_resp::Request& request, mia_hand_msgs::str_resp::Response& response);

		bool openSerialServiceCallback(mia_hand_msgs::str_req::Request& request, mia_hand_msgs::str_req::Response& response);
		bool closeSerialServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		bool posStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool speStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool anaStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool curStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		bool posStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool speStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool anaStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool curStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		bool streamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

#endif  // MIA_ROS_DRIVER_H
