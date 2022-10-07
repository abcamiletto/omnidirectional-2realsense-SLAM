#ifndef MIA_FINGER_DRIVER_H
#define MIA_FINGER_DRIVER_H

#include "mia_serial_port.h"


class MiaFingerDriver
{
	public:
		
		MiaFingerDriver(char finger_id);
		
		void setTargetPos(int pos);
		void setTargetSpe(char speed);
		void setTargetFor(char force);
		
		void setSerial(std::shared_ptr<MiaSerialPort> serial_port);
	
	private:
	
		std::shared_ptr<MiaSerialPort> serial_;
		
		int min_pos_;
		
		char pos_command_id_;
		char spe_command_id_;
		char for_command_id_;
		
		std::string base_pos_command_;
		std::string base_spe_command_;
		std::string base_for_command_;
};

#endif  // MIA_FINGER_DRIVER_H
