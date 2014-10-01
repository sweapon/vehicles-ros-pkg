/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef BMOTOR_HPP_
#define BMOTOR_HPP_
#include <std_msgs/Float32MultiArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <string>
#include <queue>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class implements the videoray brushless motor driver.
		 *
		 * \todo Add selection between N motor feedbacks in one cycle
		 * and N motor feedbacks in N cycles
		 * \todo Add configuration of motor IDs ?
		 */
		class BMotor
		{
			enum {header_len = 10};
			enum {header_byte = 0,
				id_byte=2,
				flags_byte,
				csr_byte,
				size_byte,
				checksum_byte};
			enum {syncin = 0x0FF0, syncin2 = 0x0FE0, workaroundSync = 0xE0, syncout = 0x5FF5};
			enum {chk_len = 4};

		public:
			/**
			 * Main constructor
			 */
			BMotor();
			/**
			 * Generic destructor.
			 */
			~BMotor();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			///Handle the header detection.
			void onHeader(const boost::system::error_code& e, std::size_t size);
			///Handle the incoming data stream.
			void onData(const boost::system::error_code& e, std::size_t size);
			///Handle data sent signal
			void onSent(const boost::system::error_code& e, std::size_t size);

			///The serial port setup helper method.
			bool setup_port();
			///The start receive helper function.
			void start_receive();
			///Send the a single queue element
			void sendSingle();

			///Diagnostics publisher
			ros::Publisher diagnostic;
			///Power input
			ros::Subscriber thrustIn;

			///Thrust input handler.
			void onThrustIn(const std_msgs::Float32MultiArray::ConstPtr& thrust);

			///Hardware i/o service.
			boost::asio::io_service io;
			///The serial input port.
			boost::asio::serial_port port;
			///The io service thread.
			boost::thread runner;
			///The ring buffer.
			std::vector<uint8_t> ringBuffer;
			///The input buffer.
			boost::asio::streambuf buffer;
			///The send queue
			std::queue<std::string> output;

			///Thruster IDs
			std::vector<uint8_t> thrusterId;
			///Network/Group id
			uint8_t networkId;
			///Maximum and mininum value
			double max, min;
			///Maximum temperature flag
			int maxTemp;
			///Debug time
			ros::Time newmsg;
			///Mutex for the queue
			boost::mutex queueMux, serialMux;
			///Diagnostics array
			diagnostic_msgs::DiagnosticArray diagnosticArray;
		};
	}
}

/* BMOTOR_HPP_ */
#endif
