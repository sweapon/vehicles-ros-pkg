/*********************************************************************
 * videoray_node.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: Filip Mandic
 *
 *********************************************************************/

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
#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <sensor_msgs/Joy.h>

#include <labust/vehicles/videorayComm.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/vehicles/Allocation.hpp>

#include <boost/system/system_error.hpp>
#include <boost/asio.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class implements the vehicle driver for the VideoRay family.
		 */
		class VideoRay
		{
		public:

			VideoRay(std::string portName, int baud, int flowControl, int parity, int stopBits, int dataBits);

			~VideoRay();


			void setTAU(auv_msgs::BodyForceReq tau);

			void getState(auv_msgs::NavSts& state);

			void onJoy(const sensor_msgs::Joy::ConstPtr& data);

		private:

			void configure();

			void handleInput(const boost::system::error_code& error, const size_t transferred);

			void start_receive();

			/**
			 * The method calculates the depth scale parameters.
			 *
			 * \param low The low pressure value.
			 * \param high The high pressure value.
			 * \param psiHigh The high calibration pressure.
			 * \param psiLow The low calibration pressure.
			 */
			void depthScale(int low, int high, double psiHigh, double psiLow = 0);

			void serial_configure(boost::asio::serial_port& port, std::string& portName, int baud, int flowControl, int parity, int stopBits, int dataBits );

			/**
			 * Serial port I/O service.
			 */
			boost::asio::io_service io;
			/**
			 * Serial port.
			 */
			boost::asio::serial_port port;
			/**
			 * The communication layer.
			 */
			VRCommsPtr comms;
			/**
			 * The thruster vector.
			 */
			VRComms::ThrustVec thrusters;
			/**
			 * Revolution limiter.
			 */
			labust::math::Limit<int> revLimit;
			/**
			 * The light limit.
			 */
			labust::math::Limit<int> lightLimit;
			/**
			 * Thruster affine mappings.
			 */
			double Tnn,_Tnn;
			/**
			 * The vehicle state
			 */
			VRComms::StateVecPtr states;
			/**
			 * Depth scale parameters.
			 */
			double xDepth, yDepth;
			/**
			 * The heading unwrapper.
			 */
			labust::math::unwrap unwrap;
			/**
			 * Depth calculation adjustment.
  			 */
			//bool depthVR4;
			/**
			 * Lights accessor.
			 */
			uint16_t lights;

		public:

			bool flag;

			auv_msgs::BodyForceReq lastTau;

		};


		VideoRay::VideoRay(std::string portName, int baud, int flowControl, int parity, int stopBits, int dataBits):io(),
				port(io),flag(false){

			serial_configure(port, portName, baud, flowControl, parity, stopBits, dataBits);
			ROS_ERROR("Serial port configured.");
			configure();
		}

		VideoRay::~VideoRay(){

			io.stop();
		}

		void VideoRay::configure(){

			//Read rev and light limits
			//revLimit.min = reader->value<double>("revLimit/@min");
			//revLimit.max = reader->value<double>("revLimit/@max");
			//lightLimit.min = reader->value<double>("lightLimit/@min");
			//lightLimit.max = reader->value<double>("lightLimit/@max");

			revLimit.min = -500;
			revLimit.max = 500;
			lightLimit.min = 0;
			lightLimit.max = 100;

			//Read params
			//Tnn = reader->value<double>("thruster-param/@an");
			//_Tnn = reader->value<double>("thruster-param/@bn");

			Tnn = 0.0003;
			_Tnn = 0.0001389;

			//Read depth-mappings
			//this->depthScale(reader->value<int>("depth-map/@low"),
			//		reader->value<int>("depth-map/@high"),
			//		reader->value<int>("depth-map/@psiHigh"));

			depthScale(1017,313,50,0);

			comms.reset(new VRSerialComms());

			/* Send initial stop message */
			auv_msgs::BodyForceReq tau;
			tau.wrench.force.x = 0;
			tau.wrench.force.z = 0;
			tau.wrench.torque.z = 0;
			setTAU(tau);
		}

		void VideoRay::depthScale(int low, int high, double psiHigh, double psiLow){
			xDepth = (psiHigh - psiLow)/(high-low);
			yDepth = psiLow - (low*xDepth);
		}

		void VideoRay::start_receive(){
			//boost::asio::async_read(port, boost::asio::buffer(comms->inputBuffer),
			//		boost::asio::transfer_all(),boost::bind(&VideoRay::handleInput,this,_1,_2));
			//io.poll();

			boost::asio::read(port, boost::asio::buffer(comms->inputBuffer,7));

			handleInput(boost::system::error_code(), comms->inputBuffer.size());
		}

		void VideoRay::setTAU(auv_msgs::BodyForceReq tau){

			using labust::math::coerce;
			using labust::vehicles::AffineThruster;

			try{
				//thrusters[VRComms::port] = 0;
				//thrusters[VRComms::stbd] = 0;
				//thrusters[VRComms::vert] = 0;
				thrusters[VRComms::light] = 0;

				thrusters[VRComms::port] = coerce(AffineThruster::getRevs((tau.wrench.force.x + tau.wrench.torque.z)/2, Tnn, _Tnn),revLimit);
				thrusters[VRComms::stbd] = coerce(AffineThruster::getRevs((tau.wrench.force.x - tau.wrench.torque.z)/2, Tnn, _Tnn),revLimit);
				thrusters[VRComms::vert] = -coerce(AffineThruster::getRevs(tau.wrench.force.z, Tnn, _Tnn),revLimit);
			//	thrusters[VRComms::light] = coerce(thrusters[VRComms::light],lightLimit);
			}

			catch (std::exception& e){
				std::cerr<<e.what()<<std::endl;
				thrusters[VRComms::port] = thrusters[VRComms::stbd] =
						thrusters[VRComms::vert] = thrusters[VRComms::light] = 0;
			}

			comms->encode(thrusters);
			boost::asio::write(port, boost::asio::buffer(comms->outputBuffer));

			start_receive();
		}


		void VideoRay::getState(auv_msgs::NavSts& state){

			//state.position.depth = (comms->depthPressure*xDepth + yDepth)/1.46; // provjeri točnost ovoga
			state.position.depth = comms->depthPressure;

			//state.orientation.yaw = comms->heading; // ovo je još uvijek u stupnjevima samo za test
			state.orientation.yaw = labust::math::wrapRad(comms->heading*M_PI/180);


		}

		void VideoRay::handleInput(const boost::system::error_code& error, const size_t transferred){

			if (!error && (transferred == comms->inputBuffer.size())){

				comms->decode(states);
				flag = true;

			} else {
				std::cerr<<"Communication error. Received:"<<transferred<<", expected"
						<<comms->inputBuffer.size()<<std::endl;
			}
		}

		 /**
		 * Bind socket based on configuration file.
		 */
		void VideoRay::serial_configure(boost::asio::serial_port& port, std::string& portName, int baud, int flowControl, int parity, int stopBits, int dataBits ){

			using namespace boost::asio;

			port.open(portName);
			if (port.is_open()){
				ROS_ERROR("Port successfully opened.");
				port.set_option(serial_port::baud_rate(baud));
				port.set_option(serial_port::flow_control(serial_port::flow_control::none));
				port.set_option(serial_port::parity(serial_port::parity::none));
				port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
				port.set_option(serial_port::character_size(dataBits));
			}
		}

		void VideoRay::onJoy(const sensor_msgs::Joy::ConstPtr& data){

			// Potrebno jos skalirati ulaz od joysitcka
			auv_msgs::BodyForceReq tau;
			tau.wrench.force.z = data->axes[3];
			tau.wrench.force.x = data->axes[1];
			tau.wrench.torque.z = -data->axes[2];

			setTAU(tau);
			lastTau = tau;
		}
	}
}


int main(int argc, char* argv[]){

	ros::init(argc,argv,"videoray_node");
	ros::NodeHandle nh;

	/*<!-- Serial config parameters -->
	<param name="PortName" value="/dev/ttyUSB0"/>
	<param name="BaudRate" value="9600"/>
	<param name="FlowControl" value="none"/>
	<param name="Parity" value="none"/>
	<param name="StopBits" value="1"/>
	<param name="DataBits" value="8"/>*/

	/* Start serial communication with VideoRay */
	labust::vehicles::VideoRay VR("/dev/ttyUSB0", 9600, 0, 0, 1, 8);

	/* Subscribers */
	ros::Subscriber subJoy = nh.subscribe<sensor_msgs::Joy>("joy",1,&labust::vehicles::VideoRay::onJoy,&VR);

	/* Publishers */
	ros::Publisher pubTau = nh.advertise<auv_msgs::BodyForceReq>("tauAch", 1);
	ros::Publisher pubMeas = nh.advertise<auv_msgs::NavSts>("meas", 1);

	auv_msgs::NavSts state;

	ros::Rate rate(1/0.1);

	while(ros::ok()){

		if(VR.flag){
			VR.getState(state);
			VR.flag = false;
		}

		ROS_ERROR("Orientation %f", state.orientation.yaw);
		ROS_ERROR("depth %f", state.position.depth);

		pubTau.publish(VR.lastTau);
		pubMeas.publish(state);

		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


