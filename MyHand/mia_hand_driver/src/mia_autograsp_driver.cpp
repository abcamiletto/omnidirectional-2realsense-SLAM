#include "mia_autograsp_driver.h"



MiaAutograspDriver::MiaAutograspDriver() //std::shared_ptr<MiaSerialPort> serial_port)
{
}

MiaAutograspDriver::~MiaAutograspDriver()
{
}

void MiaAutograspDriver::setTargetGrasp(char* grasp_mode, int step_time) //(1) CLPST (2) MAa (3)step [000-099] or time [000-999]
{
    //limit step_time
    if (grasp_mode[1] == 'M') {
        if( step_time < 0)
            step_time = 0;
        if (step_time > 99)
            step_time = 99;
    }
    if ( grasp_mode[1] == 'A' | grasp_mode[1] == 'a' ) {
        if (step_time < 0)
            step_time = 0;
        if (step_time > 999)
            step_time = 999;
    }

    //print step_time to 3 digit char
    char step_time_buffer [4];
    sprintf(step_time_buffer, "%03d", step_time);

    std::string base_grasp_command = "@AG";
    base_grasp_command += grasp_mode[0];
    base_grasp_command += grasp_mode[1];
    base_grasp_command += step_time_buffer[0];
    base_grasp_command += step_time_buffer[1];
    base_grasp_command += step_time_buffer[2];
    base_grasp_command += "30......";

    ROS_INFO("The command is %s", base_grasp_command.c_str());
    
    this->serial_->sendCommand(base_grasp_command);
}


void MiaAutograspDriver::setSerial(std::shared_ptr<MiaSerialPort> serial_port)
{
	this->serial_ = serial_port;
}