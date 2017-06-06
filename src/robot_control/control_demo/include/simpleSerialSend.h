#ifndef _SIMPLE_SERIAL_SEND_H
#define _SIMPLE_SERIAL_SEND_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

#define NEXT_POSE 0x0807

union byte_2 
{
	uint8 byte[2];
	int16 data;
}

union byte_4
{
	uint8 byte[4];
	float data;
}

class simpleSerialSend
{
	public:
		simpleSerialSend(string port = string("ttyUSB0"), unsigned int baud = 19200)
		{
			shared_ptr<serial_port> sp = new serial_port(iosev, port);
			// config
			sp->set_option(serial_port::baud_rate(19200));
			sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
			sp->set_option(serial_port::parity(serial_port::parity::none));
			sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
			sp->set_option(serial_port::character_size(8));
		};

		void sendPose(const int pdata[], const int ndata[])
		{
			union byte_2 data2;
			union byte_4 data4;
			data2.data = NEXT_POSE;
			sp->write(*sp, buffer(data2.byte[0]));
			sp->write(*sp, buffer(data2.byte[1]));
			
			data2.data = length;
			sp->write(*sp, buffer(data2.byte[0]));
			sp->write(*sp, buffer(data2.byte[1]));
	
			sp->write(*sp, buffer(pdata));
			sp->write(*sp, buffer(ndata));

			data4.data = switch_time;
			sp->write(*sp, buffer(data4.byte[0]));
			sp->write(*sp, buffer(data4.byte[1]));
			sp->write(*sp, buffer(data4.byte[2]));
			sp->write(*sp, buffer(data4.byte[3]));
		}

	private:
		io_service iosev;
		serial_port *sp;
		float switch_time = 0.5;
		int16_t length = 104;
	
}

#endif
