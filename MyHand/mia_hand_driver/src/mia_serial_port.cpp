#include "mia_serial_port.h"


MiaSerialPort::~MiaSerialPort()
{
  if (this->IsOpen())
  {
    this->Close();
  }
}


void MiaSerialPort::sendCommand(std::string command)
{
	this->Write(command);           //Write is a libserial method
	this->WriteByte((char) 0x2A);
	this->WriteByte((char) 0x0D);
}


char MiaSerialPort::readStream()
{
  try
  {
    this->ReadLine(this->rx_msg_, '\n', 1000);
  }
  catch (std::runtime_error& e)
  {

  }

  if (this->rx_msg_[0] == '<')       // If input message is an acknowledgment:
  {  
    if (this->rx_msg_[1] == 'A')       // If hand replied to a command involving streaming:
    {
      return 0;                            // Returning a message ID of 0
    }
    else                                 // If hand replied to a command for the fingers:
    {
      return 1;                            // Returning a message ID of 1
    }
  }
  else                                 // If input message is a streaming message:
  {
    return 2;                            // Returning a message ID of 2
  }
}
