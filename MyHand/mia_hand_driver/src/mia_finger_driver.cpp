#include "mia_finger_driver.h"


MiaFingerDriver::MiaFingerDriver(char finger_id)
{
	this->base_pos_command_ = "@" + std::to_string(finger_id) + "P+0xxx50......";
	this->base_spe_command_ = "@" + std::to_string(finger_id) + "S+....xx80....";
	this->base_for_command_ = "@" + std::to_string(finger_id) + "F+....xx80....";
	
  if (finger_id == 3)
  {
    this->min_pos_ = -300;
  }
  else
  {
    this->min_pos_ = 0;
  }
}



void MiaFingerDriver::setTargetPos(int pos)
{
	std::string pos_command = this->base_pos_command_;

	if (pos > 300)
	{
		pos = 300;
	}
	
	if (pos < this->min_pos_)
	{
		pos = this->min_pos_;
	}
	
	if (pos > 99)
	{
		pos_command.replace(5,3, std::to_string(pos));
	}
	else if (pos > 9)
	{
		pos_command.replace(5,3, "0" + std::to_string(pos));
	}
	else if (pos > 0)
	{
		pos_command.replace(5,3, "00" + std::to_string(pos));
	}
	else if (pos > -10)
	{
		pos_command.replace(3,1,"-").replace(5,3, "00" + std::to_string(-pos));
	}
	else if (pos > -100)
	{
		pos_command.replace(3,1,"-").replace(5,3, "0" + std::to_string(-pos));
	}
	else
	{
		pos_command.replace(3,1,"-").replace(5,3, std::to_string(-pos));
	}
	
	this->serial_->sendCommand(pos_command);
}



void MiaFingerDriver::setTargetSpe(char spe)
{
	std::string spe_command = this->base_spe_command_;
	
	if (spe > 99)
	{
		spe = 99;
	}
	
	if (spe < -99)
	{
		spe = -99;
	}
	
	if (spe > 9)
	{
		spe_command.replace(8,2, std::to_string(spe));
	}
	else if (spe > 0)
	{
		spe_command.replace(8,2, "0" + std::to_string(spe));
	}
	else if (spe > -10)
	{
		spe_command.replace(3,1,"-").replace(8,2, "0" + std::to_string(-spe));
	}
	else
	{
		spe_command.replace(3,1,"-").replace(8,2, std::to_string(-spe));
	}
	
	this->serial_->sendCommand(spe_command);
}



void MiaFingerDriver::setTargetFor(char force)
{
	std::string for_command = this->base_for_command_;
	
	if (force > 99)
	{
		force = 99;
	}
	
	if (force < -99)
	{
		force = -99;
	}
	
	if (force > 9)
	{
		for_command.replace(8,2, std::to_string(force));
	}
	else if (force > 0)
	{
		for_command.replace(8,2, "0" + std::to_string(force));
	}
	else if (force > -10)
	{
		for_command.replace(3,1,"-").replace(8,2, "0" + std::to_string(-force));
	}
	else
	{
		for_command.replace(3,1,"-").replace(8,2, std::to_string(-force));
	}
	
	this->serial_->sendCommand(for_command);
}



void MiaFingerDriver::setSerial(std::shared_ptr<MiaSerialPort> serial_port)
{
	this->serial_ = serial_port;
}


