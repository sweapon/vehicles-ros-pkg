/*********************************************************************
 * ethernetRelayDriver.hpp
 *
 *  Created on: Aug 26, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#ifndef ETHERNETRELAYDRIVER_HPP_
#define ETHERNETRELAYDRIVER_HPP_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <boost/asio.hpp>

#include <misc_msgs/RelayControl.h>

using boost::asio::ip::tcp;

namespace labust{
	namespace drivers {

		class EthernetRelayDriver{

		public:

		    EthernetRelayDriver(const std::string& host, int port);

			~EthernetRelayDriver();

			/* Configure TCP/IP connection. */
			void configure(const std::string& host, int port);

			/* The method encodes the given into the output buffer. */
			bool encode(int command, uint8_t relay = 0, uint8_t timeout = 0);

			/* The method decodes the data in the buffer.*/
			bool decode();

			/* Receive data from TCP/IP connection */
			size_t read();

			/* Send data using TCP/IP connection */
			void write();

			/* ROS callback for relay control */
			void onRelayRequest(const std_msgs::UInt8::ConstPtr& data);

			/** Relay control service */
			bool relayService(misc_msgs::RelayControl::Request &req, misc_msgs::RelayControl::Response &res);

			/* The input buffer. */
			std::vector<unsigned char> inputBuffer;

			/* The output buffer. */
			std::vector<unsigned char> outputBuffer;

			/* Relay states */
			std::vector<uint8_t> relayState;

			/* TCP/IP I/O service */
			boost::asio::io_service io_service;

			/* TCP/IP socket */
			tcp::socket socket;

			/** Subscribers */
			ros::Subscriber subRelayRequest;

			/** Services */
			ros::ServiceServer srvRelayCommand;

			enum {digitalActive = 0, digitalInactive, digitalSet, digitalGet, getVolts};
		};

		/*************************************************************
		 * Ethernet relay information
		 * http://www.robot-electronics.co.uk/htm/eth008tech.htm
		 *
		 ************************************************************/

		/*************************************************************
		 *  Command 								Action
		 *  dec hex
		 *	16 	10 	Get Module Info, returns 3 bytes. Module Id (19 for ETH008),
		 *				Hardware version, Firmware version.
		 *	32 	20 	Digital active - follow with 1-8 to set relay on, then a time for
		 *			pulsed output from 1-255 (100ms resolution) or 0 for permanent
		 *			Board will return 0 for success, 1 for failure
		 *	33 	21 	Digital inactive - follow with 1-8 to turn relay off, then a time for
		 *			pulsed output from 1-255 (100ms resolution) or 0 for permanent
		 *			Board will return 0 for success, 1 for failure
		 *	35 	23 	Digital set outputs - the next single byte will set all relays states,
		 *			All on = 255 (11111111) All off = 0
		 *			Board will return 0 for success, 1 for failure
		 *	36 	24 	Digital get outputs - sends a single byte back to the controller,
		 *			bit high meaning the corresponding relay is powered
		 *	58 	3A 	ASCII text commands (V4+) - allows a text string to switch outputs,
		 *			see section below
		 *	119 77 	Get serial number - Returns the unique 6 byte MAC address of the module.
		 *	120 78 	Get Volts - returns relay supply voltage as byte, 125 being 12.5V DC
		 *	121 79 	Password entry - see TCP/IP password
		 *	122 7A 	Get unlock time - see section below
		 *	123 7B 	Log out - immediately re-enables TCP/IP password protection
		 *
		*************************************************************/

		EthernetRelayDriver::EthernetRelayDriver(const std::string& host, int port):
						socket(io_service){

			outputBuffer.resize(1);
			inputBuffer.resize(1);

			relayState.resize(8);

			configure(host, port);

			ros::NodeHandle nh;

			/** Subscribers */
			subRelayRequest = nh.subscribe<std_msgs::UInt8>("relayRequest",1,&EthernetRelayDriver::onRelayRequest,this);

			/** Services */
			srvRelayCommand = nh.advertiseService("ethernet_relay", &EthernetRelayDriver::relayService,this);
		}

		EthernetRelayDriver::~EthernetRelayDriver(){

		}

		void EthernetRelayDriver::configure(const std::string& host, int port)
		{
			//Port value to string
			std::stringstream out;
			out<<port;
			//Resolve by IP or host name
			tcp::resolver resolver(io_service);
			tcp::resolver::query query = tcp::resolver::query(tcp::v4(), host, out.str());
			tcp::endpoint endpoint = *resolver.resolve(query);
			//tcp::endpoint endpoint(boost::asio::ip::address::from_string(host.c_str()), port);
			socket.connect(endpoint);

			if(socket.is_open()){
				ROS_INFO("TCP/IP socket opened.");
			}
		}

		bool EthernetRelayDriver::encode(int command, uint8_t relay, uint8_t timeout){

			/* Define protocol commands */
			enum {digitalActiveCmd = 32, digitalInactiveCmd = 33, digitalSetCmd = 35, digitalGetCmd = 36};
			enum {getVoltsCmd = 120, passwordEntryCmd, unlockTimeCmd, logOutCmd};

			unsigned char setBits(0);

			outputBuffer.clear();

			if(!((command == digitalActiveCmd || command == digitalInactiveCmd) && relay == 0)){
				switch(command){

					case digitalActive: /* Turn on relay with timeout */

						outputBuffer.push_back(static_cast<int>(digitalActiveCmd));
						outputBuffer.push_back(relay);
						outputBuffer.push_back(timeout);
						break;

					case digitalInactive: /* Turn off relay with timeout */

						outputBuffer.push_back(static_cast<int>(digitalInactiveCmd));
						outputBuffer.push_back(relay);
						outputBuffer.push_back(timeout);
						break;

					case digitalSet: /* Set relay state */

						outputBuffer.push_back(static_cast<int>(digitalSetCmd));

						for(std::vector<bool>::size_type i = 0; i != relayState.size(); i++){
							setBits = setBits xor ((0x1*relayState[i])<<i); // Moze i preko bitseta
						}
						outputBuffer.push_back(setBits);
						break;

					case digitalGet: /* Get relay state */

						outputBuffer.push_back(static_cast<int>(digitalGetCmd));
						break;

					case getVolts: /* Read relay supply voltage */

						outputBuffer.push_back(static_cast<int>(getVoltsCmd));
						break;

					default:
						break;
				}

				write();

				return true;
			} else {

				ROS_ERROR("Irregular command.");
				return false;
			}
		}

		bool EthernetRelayDriver::decode(){
			read();
			ROS_ERROR("Message received: %d", inputBuffer[0]);
			return true;
		}

		size_t EthernetRelayDriver::read(){
			return boost::asio::read(socket, boost::asio::buffer(inputBuffer,1));
			// \TODO Dodaj asinkrono citanje ili citanje s timeoutom
		}

		void EthernetRelayDriver::write(){
			boost::asio::write(socket, boost::asio::buffer(outputBuffer, outputBuffer.size()));
		}

		/** Topic for relay control */
		void EthernetRelayDriver::onRelayRequest(const std_msgs::UInt8::ConstPtr& data){

			uint8_t tmp = 0;

			for(std::vector<uint8_t>::size_type i = 0; i != relayState.size(); i++){

				tmp = (data->data) & (0x1<<i);
				relayState[i] = (tmp)?1:0;
			}

			if(encode(digitalSet)){
				//decode();
			}
		}

		/** Service for relay control */
		bool EthernetRelayDriver::relayService(misc_msgs::RelayControl::Request &req, misc_msgs::RelayControl::Response &res){

			res.result = encode((req.relayState)?digitalActive:digitalInactive, req.relayNum, req.timeout);
			return true;
		}
	}
}

#endif /* ETHERNETRELAYDRIVER_HPP_ */
