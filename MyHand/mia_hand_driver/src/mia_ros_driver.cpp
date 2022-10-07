#include "mia_ros_driver.h"
#include "mia_serial_port.h"
#include <iostream>
#include <string>


MiaROSDriver::MiaROSDriver():    // Class constructor

	thumb_(1), index_(3), mrl_(2),              // Initializing finger drivers by assigning finger IDs
  	connected_(false)
{
	ros::NodeHandle mia_p("~");
	std::string serial_port_name;    	
	mia_p.param<std::string>("port", serial_port_name, "/dev/ttyUSB0");
	this->serial_ = std::make_shared<MiaSerialPort>();  // Initializing serial port object

	initializePublishers();                             // Initializing topic publishers
	initializeSubscribers();                            // Initializing topic subscribers
	initializeServices();                               // Initializing services

	//ROS_INFO("Please specify Mia Hand serial port name (e.g. /dev/ttyUSB0):\n");
	//std::cin >> serial_port_name;
	
	this->connect(serial_port_name);                    // Connecting to serial port
  	
}



MiaROSDriver::~MiaROSDriver()
{
  this->connected_ = false;
}



void MiaROSDriver::connect(std::string port_name)
{
	bool no_exceptions = true;

	try
	{
		this->serial_->Open(port_name);                                             // Opening specified serial port
	}
	catch (std::invalid_argument& e)
	{
		ROS_ERROR("Invalid serial port name\n");
		no_exceptions = false;
	}
	catch (std::logic_error& e)
	{
		ROS_INFO("The selected serial port is already open\n");
		no_exceptions = false;
	}
	catch (std::runtime_error& e)
	{
		ROS_ERROR("Could not open the selected serial port\n");
		no_exceptions = false;
	}
	
	if (no_exceptions)                                                          // If serial port opened successfully:
	{
		this->serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);               // Setting baud rate (115200 bit/s)
		this->serial_->FlushIOBuffers();                                            // Clearing serial input and output buffers
		
    	this->connected_ = true;

		ROS_INFO("Mia Hand connected\n");                                           // Signaling successful opening
		
		this->thumb_.setSerial(this->serial_);                                      // Connecting thumb driver to the same serial port
		this->index_.setSerial(this->serial_);                                      // Connecting index driver to the same serial port
		this->mrl_.setSerial(this->serial_);                                        // Connecting mrl driver to the same serial port

		this->grasp_.setSerial(this->serial_);										// Connecting grasp driver to the same serial port

		this->serial_read_thread_ = std::thread(&MiaROSDriver::serialPoll,this);    // Initializing thread for countinuosly reading hand outputs
		this->serial_read_thread_.detach();                                         // Detaching thread

		//this->serial_->sendCommand("@AK.............");
		this->serial_->sendCommand("@Ad............."); 

	}
}



void MiaROSDriver::disconnect()
{
	this->serial_->Close();                          // Disconnecting Mia

	if (!this->serial_->IsOpen())
	{
		this->connected_ = false;
		ROS_INFO("Mia Hand disconnected\n");
	}
	else
	{
		ROS_ERROR("Could not close Mia Hand serial port\n");
	}
}



//Initialize Subscribers

void MiaROSDriver::initializeSubscribers()
{
	this->thu_pos = this->subscribe("/MCP1_position_controller/command", 1000, &MiaROSDriver::thuPosSubscriberCallback, this);
	this->ind_pos = this->subscribe("/MCP2_position_controller/command", 1000, &MiaROSDriver::indPosSubscriberCallback, this);
	this->mrl_pos = this->subscribe("/MCP3_position_controller/command", 1000, &MiaROSDriver::mrlPosSubscriberCallback, this);

	this->thu_spe = this->subscribe("/MCP1_velocity_controller/command", 1000, &MiaROSDriver::thuSpeSubscriberCallback, this);
	this->ind_spe = this->subscribe("/MCP2_velocity_controller/command", 1000, &MiaROSDriver::indSpeSubscriberCallback, this);
	this->mrl_spe = this->subscribe("/MCP3_velocity_controller/command", 1000, &MiaROSDriver::mrlSpeSubscriberCallback, this);

	this->thu_for = this->subscribe("/MCP1_effort_controller/command", 1000, &MiaROSDriver::thuForSubscriberCallback, this);      
	this->ind_for = this->subscribe("/MCP2_effort_controller/command", 1000, &MiaROSDriver::indForSubscriberCallback, this);
	this->mrl_for = this->subscribe("/MCP3_effort_controller/command", 1000, &MiaROSDriver::mrlForSubscriberCallback, this);

	this->grasp = this->subscribe("/Autograsp/command", 1000, &MiaROSDriver::graspSubscriberCallback, this);
}



//initialize services

void MiaROSDriver::initializeServices()
{      
	this->open_serial  = this->advertiseService("/mia/open_serial",  &MiaROSDriver::openSerialServiceCallback,  this);
	this->close_serial = this->advertiseService("/mia/close_serial", &MiaROSDriver::closeSerialServiceCallback, this);
        
	this->pos_stream_on = this->advertiseService("/mia/pos_stream_on", &MiaROSDriver::posStreamOnServiceCallback, this);   
	this->spe_stream_on = this->advertiseService("/mia/spe_stream_on", &MiaROSDriver::speStreamOnServiceCallback, this);
	this->ana_stream_on = this->advertiseService("/mia/ana_stream_on", &MiaROSDriver::anaStreamOnServiceCallback, this);
	this->cur_stream_on = this->advertiseService("/mia/cur_stream_on", &MiaROSDriver::curStreamOnServiceCallback, this);
        
	this->pos_stream_off = this->advertiseService("/mia/pos_stream_off", &MiaROSDriver::posStreamOffServiceCallback, this);  
	this->spe_stream_off = this->advertiseService("/mia/spe_stream_off", &MiaROSDriver::speStreamOffServiceCallback, this);  
	this->ana_stream_off = this->advertiseService("/mia/ana_stream_off", &MiaROSDriver::anaStreamOffServiceCallback, this);  
	this->cur_stream_off = this->advertiseService("/mia/cur_stream_off", &MiaROSDriver::curStreamOffServiceCallback, this);
        
	this->stream_off = this->advertiseService("/mia/stream_off", &MiaROSDriver::streamOffServiceCallback, this);
}



//initialize publishers

void MiaROSDriver::initializePublishers()
{
	this->fingers_pos = this->advertise<mia_hand_msgs::FingersData>("/mia/fin_pos", 1000);
	this->fingers_spe = this->advertise<mia_hand_msgs::FingersData>("/mia/fin_spe", 1000);
	this->fingers_cur = this->advertise<mia_hand_msgs::FingersData>("/mia/fin_cur", 1000);
	this->fingers_sg  = this->advertise<mia_hand_msgs::FingersStrainGauges>("/mia/fin_sg", 1000);
}



//Subscribers callback functions

void MiaROSDriver::thuPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                         // If Mia Hand is connected:
	{
		this->thumb_.setTargetPos((int)msg->data);     // Setting specified thumb target position
	}
}

void MiaROSDriver::indPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                         // If Mia Hand is connected:
	{
		this->index_.setTargetPos((int)msg->data);     // Setting specified index target position
	}
}

void MiaROSDriver::mrlPosSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                         // If Mia Hand is connected:
	{
		this->mrl_.setTargetPos((int)msg->data);      // Setting specified mrl target position
	}
}



void MiaROSDriver::thuSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->thumb_.setTargetSpe((int)msg->data);     // Setting specified thumb target speed
	}
}

void MiaROSDriver::indSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->index_.setTargetSpe((int)msg->data);     // Setting specified index target speed
	}
}

void MiaROSDriver::mrlSpeSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->mrl_.setTargetSpe((int)msg->data);       // Setting specified mrl target speed
	}
}



void MiaROSDriver::thuForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->thumb_.setTargetFor((int)msg->data);     // Setting specified thumb target force
	}
}

void MiaROSDriver::indForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->index_.setTargetFor((int)msg->data);     // Setting specified index target force
	}
}

void MiaROSDriver::mrlForSubscriberCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if (this->connected_)                          // If Mia Hand is connected:
	{
		this->mrl_.setTargetFor((int)msg->data);       // Setting specified mrl target force
	}
}



void MiaROSDriver::graspSubscriberCallback(const mia_hand_msgs::AutoGraspData::ConstPtr& msg)
{
	if (this->connected_)
	{
		
		const char* grasp_type_char = msg->grasp_type.c_str();
		const char* grasp_mode_char = msg->grasp_mode.c_str();
		char grasp_char[2] = {*grasp_type_char, *grasp_mode_char};

		this->grasp_.setTargetGrasp(grasp_char, msg->step_time);
	}
}



//Services Callback Functions

bool MiaROSDriver::openSerialServiceCallback(mia_hand_msgs::str_req::Request& request, mia_hand_msgs::str_req::Response& response)
{
	if (!this->serial_->IsOpen())
	{
		this->connect(request.req);
	}
	else
	{
		ROS_INFO("Mia Hand already connected\n");
	}
		
	return true;
}

bool MiaROSDriver::closeSerialServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)
	{
		this->disconnect();
	}
	else
	{
		ROS_INFO("Mia Hand already disconnected\n");
	}

		return true;
}



bool MiaROSDriver::posStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{	
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADP1...........");    // Sending command to enable position streaming
	}
	
	return true;
}

bool MiaROSDriver::speStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADS1...........");    // Sending command to enable speed streaming
	}
	
	return true;
}

bool MiaROSDriver::anaStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADA1...........");    // Sending command to enable analog streaming
	}
	
	return true;
}



bool MiaROSDriver::curStreamOnServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADC1...........");    // Sending command to enable current streaming
	}
	
	return true;
}


bool MiaROSDriver::posStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADP0...........");    // Sending command to disable position streaming
	}
	
	return true;
}



bool MiaROSDriver::speStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADS0...........");    // Sending command to disable speed streaming
	}
	
	return true;
}



bool MiaROSDriver::anaStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADA0...........");    // Sending command to disable analog streaming
	}
	
	return true;
}



bool MiaROSDriver::curStreamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@ADC0...........");    // Sending command to disable general states streaming
	}
	
	return true;
}



bool MiaROSDriver::streamOffServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (this->connected_)                              // If Mia Hand is connected:
	{
		this->serial_->sendCommand("@Ad.............");    // Sending command to stop data streaming
	}
	
	return true;
}



void MiaROSDriver::publishStreamData(std::string stream_msg)
{
	switch (stream_msg[0])
	{
		case 'e':
    {
      auto msg = mia_hand_msgs::FingersData();
    
      msg.thu = (stream_msg[9]  - 48)*100 + (stream_msg[10] - 48)*10 + stream_msg[11] - 48;
      msg.ind = (stream_msg[27] - 48)*100 + (stream_msg[28] - 48)*10 + stream_msg[29] - 48;
      msg.mrl = (stream_msg[18] - 48)*100 + (stream_msg[19] - 48)*10 + stream_msg[20] - 48;
     
      this->fingers_pos.publish(msg);
    
      break;
    }

		case 's':
    {
			auto msg = mia_hand_msgs::FingersData();
				
			msg.thu = (stream_msg[10] - 48)*10 + stream_msg[11] - 48;
			msg.ind = (stream_msg[28] - 48)*10 + stream_msg[29] - 48;
			msg.mrl = (stream_msg[19] - 48)*10 + stream_msg[20] - 48;
			
			this->fingers_spe.publish(msg);
		
			break;
    }
			
		case 'c':
    {	
			auto msg = mia_hand_msgs::FingersData();
			
			msg.thu = (stream_msg[8]  - 48)*1000 + (stream_msg[9]  - 48)*100 + (stream_msg[10] - 48)*10 + stream_msg[11] - 48;
			msg.ind = (stream_msg[17] - 48)*1000 + (stream_msg[18] - 48)*100 + (stream_msg[19] - 48)*10 + stream_msg[20] - 48;
			msg.mrl  = (stream_msg[26] - 48)*1000 + (stream_msg[27] - 48)*100 + (stream_msg[28] - 48)*10 + stream_msg[29] - 48;
			
      this->fingers_cur.publish(msg);
			
			break;
    }
			
		case 'a':
    {
			auto msg = mia_hand_msgs::FingersStrainGauges();
			
			msg.thu[0] = (stream_msg[35] - 48)*1000 + (stream_msg[36] - 48)*100 + (stream_msg[37] - 48)*10 + stream_msg[38] - 48;
			msg.thu[1] = (stream_msg[44] - 48)*1000 + (stream_msg[45] - 48)*100 + (stream_msg[46] - 48)*10 + stream_msg[47] - 48;
			
			msg.ind[0] = (stream_msg[26] - 48)*1000 + (stream_msg[27] - 48)*100 + (stream_msg[28] - 48)*10 + stream_msg[29] - 48;
			msg.ind[1] = (stream_msg[17] - 48)*1000 + (stream_msg[18] - 48)*100 + (stream_msg[19] - 48)*10 + stream_msg[20] - 48;
			
			msg.mrl[0] = (stream_msg[8]  - 48)*1000 + (stream_msg[9]  - 48)*100 + (stream_msg[10] - 48)*10 + stream_msg[11] - 48;
			msg.mrl[1] = (stream_msg[53] - 48)*1000 + (stream_msg[54] - 48)*100 + (stream_msg[55] - 48)*10 + stream_msg[56] - 48;
			
			this->fingers_sg.publish(msg);
			
			break;
    } 
	}
}



void MiaROSDriver::serialPoll()
{
	while (this->connected_)
	{
		if (this->serial_->IsDataAvailable())
		{
      char msg_id = this->serial_->readStream();
      std::string info_msg;

			switch (msg_id)
			{
				case 0:  // Streaming command acknowledge

					if (this->serial_->rx_msg_[2] == 'D')
          {
            switch (this->serial_->rx_msg_[3])
            {
              case 'P':
                
                info_msg = "Position streaming ";
                break;

              case 'S':

                info_msg = "Speed streaming ";
                break;

              case 'C':

                info_msg = "Current streaming ";
                break;

              case 'A':

                info_msg = "Strain gauge state streaming ";
                break;
            }

            if (this->serial_->rx_msg_[4] == '1')
            {
              info_msg += "enabled\n";
            }
            else
            {
              info_msg += "disabled\n";
            }
          }
          else if (this->serial_->rx_msg_[2] == 'd')
          {
            info_msg = "Data streaming stopped\n";
          }
					
          ROS_INFO("%s", info_msg.c_str());

					break;
				
        case 1:  // Fingers command acknowledge
        {
          std::string finger;
          
          switch (this->serial_->rx_msg_[1])
          {
            case '1':
              
              finger = "thumb";
              break;

            case '2':
              
              finger = "mrl";
              break;

            case '3':
              
              finger = "index";
              break;
          }

          switch (this->serial_->rx_msg_[2])
          {
            case 'P':

              info_msg = "Moving " + finger + " to " + this->serial_->rx_msg_.substr(5,3) + "\n";
              break;

            case 'S':

              if (this->serial_->rx_msg_[3] == '-')
              {
                info_msg = "Opening " + finger + " at " + this->serial_->rx_msg_.substr(8,2) + " counts/16ms\n";
              }
              else
              {
                info_msg = "Closing " + finger + " at " + this->serial_->rx_msg_.substr(8,2) + " counts/16ms\n";
              }

              break;

            case 'F':

              info_msg = "Exerting " + this->serial_->rx_msg_.substr(8,2) + " N at " + finger + "\n";
              break;
          }

          ROS_INFO("%s", info_msg.c_str());

          break;
        }

		case 2:  // Streaming message
		
			this->publishStreamData(this->serial_->rx_msg_);

			break;
      }
    }
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mia");
	MiaROSDriver mia_hand;
	ros::spin();
	
 	return 0;
}
