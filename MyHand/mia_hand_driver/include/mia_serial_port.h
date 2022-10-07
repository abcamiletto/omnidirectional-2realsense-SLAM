#ifndef MIA_SERIAL_PORT_H
#define MIA_SERIAL_PORT_H

#include "libserial/SerialPort.h"


class MiaSerialPort: public LibSerial::SerialPort
{
	public:

    ~MiaSerialPort();

		void sendCommand(std::string command);
		
		char readStream();

    std::string rx_msg_;
};

#endif  // MIA_SERIAL_PORT_H
