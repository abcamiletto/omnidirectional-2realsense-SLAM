// ROS WRAPPER:
// Qui devono esserci solo cose che non riguardano i driver nello specifico, 
// devono esserci però tutte le call a publisher, subscriber e parametri
// https://roboticsbackend.com/create-a-ros-driver-package-introduction-what-is-a-ros-wrapper-1-4/


#include "ih2_azzurra_driver.h"


IH2Driver::IH2Driver (ros::NodeHandle nh, std::string serial_port_name):            //Constructor
    nh_(nh),
    connected_(false)
{
    connect(serial_port_name);
    pubSubInit();
}


IH2Driver::~IH2Driver()                                         // Destructor
{
    disconnect();
}


void IH2Driver::connect(std::string serial_port_name)
{
    bool no_exceptions = true;

    try{        
        this->ih2_serial_ = std::make_shared<LibSerial::SerialPort>(serial_port_name);
        ROS_INFO("Trying to connect\n");                                           
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
    
    if (no_exceptions)                                                          
    {
        this->ih2_serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);               // Setting baud rate (115200 bit/s)
        this->ih2_serial_->FlushIOBuffers();                                            // Clearing serial input and output buffers

        this->ih2_serial_->WriteByte((char) 0x46);                                      // Fast calibration
        std::this_thread::sleep_for(std::chrono::seconds(4));
        this->ih2_serial_->WriteByte((char) 0x47);                                      // Stop any active streaming
        std::this_thread::sleep_for(std::chrono::seconds(2));

        this->connected_ = true;
        ROS_INFO("Azzurra Hand connected\n");                                           // Successful opening
        

        //this->ih2_command_.setSerial(this->ih2_serial_);
        this->ih2_command_.ih2_serial1_= this->ih2_serial_;                             // assign finger serial port

        this->serial_read_thread_ = std::thread(&IH2Driver::serialPoll,this);    // Initializing thread for countinuosly reading hand outputs
		this->serial_read_thread_.detach();  
                                                  

    }

} 


void IH2Driver::disconnect()
{
    this->ih2_serial_->WriteByte((char) 0x47);           // Closing Streaming
    this->ih2_serial_->Close();                          // Disconnecting Azzurra

    if (!this->ih2_serial_->IsOpen())
    {
        this->connected_ = false;
        ROS_INFO("Azzurra Hand disconnected\n");
    }
    else
    {
        ROS_ERROR("Could not close Mia Hand serial port\n");
    }
}


void IH2Driver::pubSubInit()
{
    // Position, speed, force callback
    this->pos_sub = this->nh_.subscribe<trajectory_msgs::JointTrajectory>("/position_controller/command",1000, boost::bind(&IH2Driver::ctrlCallback, this, _1, 1));
    // Autograsp callback
    this->aug_sub = this->nh_.subscribe<ih2_azzurra::AutoGrasp>("/autograsp_controller/command",1000, boost::bind(&IH2Driver::auGrCallback, this, _1, 1));
    //Enable Streaming callback
    this->str_sub = this->nh_.subscribe<std_msgs::Int8>("/streaming_enabler/command",1000, boost::bind(&IH2Driver::enStCallback, this, _1));
    //Streaming Publisher
    this->str_pub = this->nh_.advertise<trajectory_msgs::JointTrajectory>("/streaming/state", 1000);

}


//a couple of callbacks

void IH2Driver::ctrlCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg, int controllerID)
{
    if (this->connected_) {

        switch (controllerID)
        {
            case 1:
            {
                this->ih2_command_.sendPositionCommand(msg->joint_names, msg->points);
                break;
            }
            case 2:
            {
                //speed
                ROS_INFO("ciao");
                break;
            }
            case 3:
            {
                //frce
                ROS_INFO("ciao");

                break;
            }
        }

    }


}


void IH2Driver::auGrCallback(const ih2_azzurra::AutoGrasp::ConstPtr& msg, int controllerID)
{
    if (this->connected_) 
    {
        this->ih2_command_.sendAutoGraspCommand(msg->grasp_type, msg->grasp_step, msg->grasp_force);
    }
}


void IH2Driver::enStCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (this->connected_) 
    {
        this->ih2_command_.enableStreaming(msg->data);
    }
}


void IH2Driver::serialPoll()
{
    while (this->connected_)
    {   
        //this->ih2_command_.
        streamPublisher();
    }
}
//Service&StreamPoll

void IH2Driver::streamPublisher()
{
	if (this->ih2_serial_->IsDataAvailable())
	{   
        char msg_id = readStream();
        if (msg_id == 0)
        {

        }
    }

}
char IH2Driver::readStream()
{   
    std::string circular_buffer ="";
    int data_buffer[16];
    unsigned char newchar;   
   
    for (int i=0; i<4;i++)
    {
        try{
            this->ih2_serial_->ReadByte(newchar, 60);
            //ROS_INFO("newchar |%c|", newchar);
        }catch (std::runtime_error& e){}//ROS_INFO("exception ");}
        catch (LibSerial::NotOpen){ROS_INFO("exception no");}
        circular_buffer += newchar;
        
    }

    //ROS_INFO("CIRCUBUF |%s|", circular_buffer.c_str());

    while(circular_buffer.compare({(char)0x55,(char)0xAA,(char)0xAA,(char)0x55 }) != 0)
    {
        circular_buffer.erase(0,1);
        try{
            this->ih2_serial_->ReadByte(newchar, 60);
            //ROS_INFO("newchar you lck here? |%c|", newchar);
            //ROS_INFO("CIRCUBUF ||%s||", circular_buffer.c_str());
        }catch (std::runtime_error& e){}//ROS_INFO("exception ");}
        catch (LibSerial::NotOpen){ROS_INFO("exception no");}
        circular_buffer += newchar;
    }
       
    for (int i=0; i<16;i++)
    {
        try{
            this->ih2_serial_->ReadByte(newchar, 60);
            //ROS_INFO("newchar |%c|", newchar);
        }catch (std::runtime_error& e){}//ROS_INFO("exception ");}
        catch (LibSerial::NotOpen){ROS_INFO("exception no");}
        if(i>2){
            data_buffer[i-3] = (unsigned int) newchar;
        }
    }

    
    std::vector<int> positions = {data_buffer[0], data_buffer[3], data_buffer[6], data_buffer[9], data_buffer[12]};
    std::vector<int> tensions = {0, (data_buffer[2] | (data_buffer[1] <<8)), (data_buffer[5] | (data_buffer[4] <<8)),
                        (data_buffer[8] | (data_buffer[7] <<8)), (data_buffer[11] | (data_buffer[10] <<8))};
    
    ROS_INFO("positions Ta %i Tf %i If %i Mf %i Rf %i ", positions[0],positions[1],positions[2],positions[3],positions[4]);
    ROS_INFO("tensions        Tf %i If %i Mf %i Rf %i ", tensions[1],tensions[2],tensions[3],tensions[4]);

    
    std::vector<double> positions_float (5);
    std::vector<double> tensions_float (5);
   
    std::copy_n( positions.begin(), 5, positions_float.begin() );
    std::copy_n( tensions.begin(), 5, tensions_float.begin() );

    
    trajectory_msgs::JointTrajectory str_msg; 
    str_msg.points.resize(1);
    str_msg.points[0].positions.resize(5);
    

    str_msg.joint_names = {"THU_A", "THU_F", "IND_F", "MID_F", "RIL_F"};
    //str_msg.points[0].positions = {(double) positions[0], (double) positions[1], (double) positions[2],(double) positions[3], (double)positions[4]};
    str_msg.points[0].positions = positions_float;
    str_msg.points[0].effort = tensions_float;

    this->str_pub.publish(str_msg);
 
    return 0;
}




int main(int argc, char **argv)                                 //main
{
	ros::init(argc, argv, "ih2_azzurra");
    ros::NodeHandle ih2_nh;
    ros::NodeHandle ih2_nh_private("~");
   
    std::string serial_port_name;
    ih2_nh_private.param<std::string>("port", serial_port_name, "/dev/ttyUSB0");
 
    
    IH2Driver ih2_hand(ih2_nh, serial_port_name);
    
    ros::Rate loop_rate(3);
    while (ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }

	//ros::spin();
	
 	return 0;
}

