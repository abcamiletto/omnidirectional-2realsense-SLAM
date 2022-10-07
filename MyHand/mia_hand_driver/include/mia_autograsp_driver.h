#ifndef MIA_AUTOGRASP_DRIVER_H
#define MIA_AUTOGRASP_DRIVER_H

#include <cstdint>               //per int16
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "mia_serial_port.h"     //devo utilizzare sendCommand

class MiaAutograspDriver
{

    public:

        MiaAutograspDriver();  //(std::shared_ptr<MiaSerialPort> serial_);

        ~MiaAutograspDriver();


        void setTargetGrasp(char* grasp_mode_char, int step_time); // come argomento un array 
        void setSerial(std::shared_ptr<MiaSerialPort> serial_port);
        

    private:

        std::shared_ptr<MiaSerialPort> serial_;

        std::string base_grasp_command_;

};

#endif 