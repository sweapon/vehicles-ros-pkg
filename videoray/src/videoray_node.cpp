/*
 * videoray_node.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: Filip Mandic
 */

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
			/**
			 * Main constructor. Take a XML reader pointer and configures the controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
			 */
			VideoRay(std::string portName, int baud, int flowControl, int parity, int stopBits, int dataBits);
			/**
			 * Generic destructor.
			 */
			~VideoRay();

			/**
			 * \overload labust::vehicles::Driver::setTAU
			 */
			void setTAU(auv_msgs::BodyForceReq tau);
			/**
			 * \overload labust::vehicles::Driver::getState
			 */
			void getState(auv_msgs::NavSts& state);
      /**
       *	\overload labust::vehicles::Driver::setGuidance
       */
		//	void setGuidance(const labust::vehicles::guidanceMapRef guidance);
      /**
       * \overload labust:::vehicles::Driver::setCommand
       */
		//	void setCommand(const labust::apps::stringRef commands);
      /**
       * \overload labust::vehicles::Driver::getData
       */
		//	void getData(const labust::apps::stringPtr data);

			void onJoy(const sensor_msgs::Joy::ConstPtr& data);

		private:

			/**
			 * The method configures the driver through a XML config file.
			 *
			 * \param reader Pointer to the XMLReader object containing the parameters.
			 * \param id Identification class.
			 */
			void configure();
			/**
			 * The method handles incoming data.
			 *
			 * \param e The error that occured during transfer.
			 * \param transferred Number of bytes that were transfered.
			 */
			void handleInput(const boost::system::error_code& error, const size_t transferred);
			/**
			 * The method starts the receiving loop.
			 */
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
			//serial::Serial::Serial port;
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
			//labust::vehicles::stateMapPtr states;
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
			bool depthVR4;
			/**
			 * Lights accessor.
			 */
			uint16_t lights;

		public:
			bool flag;
		};



		VideoRay::VideoRay(std::string portName, int baud, int flowControl, int parity, int stopBits, int dataBits):io(),
				port(io),flag(false){

			serial_configure(port, portName, baud, flowControl, parity, stopBits, dataBits);
			ROS_ERROR("Serial port configured.");
			configure();
		}

		VideoRay::~VideoRay()
		{
			io.stop();
		}

		void VideoRay::configure()
		{
//			//Configure the serial port.
//			labust::comms::serial_configure(*reader,port);
//			serial_configure(*reader,port);
//
//			//Read rev and light limits
//			revLimit.min = reader->value<double>("revLimit/@min");
//			revLimit.max = reader->value<double>("revLimit/@max");
//			lightLimit.min = reader->value<double>("lightLimit/@min");
//			lightLimit.max = reader->value<double>("lightLimit/@max");

			revLimit.min = -220;
			revLimit.max = 220;
			lightLimit.min = 0;
			lightLimit.max = 100;

//
//			//Read params
//			Tnn = reader->value<double>("thruster-param/@an");
//			_Tnn = reader->value<double>("thruster-param/@bn");

			Tnn = 0.0003;
			_Tnn = 0.0001389;
//
//			//Read depth-mappings
//			this->depthScale(reader->value<int>("depth-map/@low"),
//					reader->value<int>("depth-map/@high"),
//					reader->value<int>("depth-map/@psiHigh"));

			depthScale(1017,313,50,0);
//
//			//Identify protocol
//			if (reader->try_expression("vr4"))
//			{
//				comms.reset(new VRFutaba(reader->value<int>("vr4")));
//				depthVR4 = true;
//			}
//			else
//			{
				comms.reset(new VRSerialComms());
//			}
//
//			//Send initial stop message
//			labust::vehicles::tauMap tau;
//			this->setTAU(tau);

			/* Send initial stop message */
			auv_msgs::BodyForceReq tau;
			tau.wrench.force.x = 0;
			tau.wrench.force.z = 0;
			tau.wrench.torque.z = 0;
			setTAU(tau);
		}

		void VideoRay::depthScale(int low, int high, double psiHigh, double psiLow)
		{
			xDepth = (psiHigh - psiLow)/(high-low);
			yDepth = psiLow - (low*xDepth);
		}

		void VideoRay::start_receive()
		{
			//boost::asio::async_read(port, boost::asio::buffer(comms->inputBuffer),
			//		boost::asio::transfer_all(),boost::bind(&VideoRay::handleInput,this,_1,_2));
			//io.poll();
			//ROS_ERROR("DEBUG1-3.5");

			//int preneseno = boost::asio::read(port, boost::asio::buffer(comms->inputBuffer));
			boost::asio::read(port, boost::asio::buffer(comms->inputBuffer,7));
			//ROS_ERROR("Preneseno: %d", preneseno);
			//ROS_ERROR("DEBUG1-4");

			handleInput(boost::system::error_code(), comms->inputBuffer.size());
			//ROS_ERROR("DEBUG1-5");

		}

		void VideoRay::setTAU(auv_msgs::BodyForceReq tau){

//			//using namespace labust::vehicles::tau;
			using labust::math::coerce;
			using labust::vehicles::AffineThruster;

			try
			{
//				thrusters[VRComms::port] = 0;
//				thrusters[VRComms::stbd] = 0;
//				thrusters[VRComms::vert] = 0;
				thrusters[VRComms::light] = 0;

				thrusters[VRComms::port] = coerce(AffineThruster::getRevs((tau.wrench.force.x + tau.wrench.torque.z)/2, Tnn, _Tnn),revLimit);
				thrusters[VRComms::stbd] = coerce(AffineThruster::getRevs((tau.wrench.force.x - tau.wrench.torque.z)/2, Tnn, _Tnn),revLimit);
				thrusters[VRComms::vert] = -coerce(AffineThruster::getRevs(tau.wrench.force.z, Tnn, _Tnn),revLimit);
			//	thrusters[VRComms::light] = coerce(thrusters[VRComms::light],lightLimit);
			}
			catch (std::exception& e)
			{
				std::cerr<<e.what()<<std::endl;
				thrusters[VRComms::port] = thrusters[VRComms::stbd] =
						thrusters[VRComms::vert] = thrusters[VRComms::light] = 0;
			}

//			ROS_ERROR("DEBUG1-1");
//
//
			comms->encode(thrusters);
			boost::asio::write(port, boost::asio::buffer(comms->outputBuffer));
			//std::cout<<"Data:";
			//for (size_t i=0; i<comms->outputBuffer.size(); ++i) std::cout<<int(comms->outputBuffer[i])<<",";
			//std::cout<<std::endl;
			//ROS_ERROR("DEBUG1-2");

			start_receive();
			//ROS_ERROR("DEBUG1-3");

		}


		void VideoRay::getState(auv_msgs::NavSts& state)
		{
			//std::cout<<"Get state."<<std::endl;
			//Recalculate depth from pressure ?.
			//using namespace labust::vehicles::state;
			//state = (*this->states);
			//if (depthVR4)
			//{
			//	state[z] = ((*states)[depthPressure] - 98.0)/10;
			//}
			//else
			//{
			//	state[z] = ((*states)[depthPressure]*xDepth + yDepth)/1.46;
			//}
			//state[yaw] = unwrap((*states)[yaw]);

			//state.position.depth = ((*states)[VRComms::depthPressure]*xDepth + yDepth)/1.46;
			//state.orientation.yaw = (*states)[VRComms::heading]; // ovo je još uvijek u stupnjevima samo za test

			state.position.depth = (comms->depthPressure*xDepth + yDepth)/1.46; // provjeri točnost ovoga
			state.orientation.yaw = comms->heading; // ovo je još uvijek u stupnjevima samo za test

			//ROS_ERROR("Orjentacija iz getState: %f", state.orientation.yaw);

		}
//
//		void VideoRay::setGuidance(const labust::vehicles::guidanceMapRef guidance)
//		{
//			throw std::runtime_error("VRPro::setGuidance not implemented.");
//		}
//		void VideoRay::setCommand(const labust::apps::stringRef commands){
//		try
//		{
//			if (this->unwrapFromXml(commands))
//			{
//				thrusters[VRComms::light] = lights;
//			}
//			//throw std::runtime_error("VRPro::setCommand not implemented.");
//			//std::cout<<"Have the lights:"<<thrusters[VRComms::light]<<std::endl;
//		}
//		catch (std::exception& e)
//		{
//			std::cerr<<e.what()<<std::endl;
//			std::cerr<<"RejectedCommand:"<<commands<<std::endl;
//		}
//		}

//		void VideoRay::getData(const labust::apps::stringPtr data)
//		{
//			(*data) = *this->wrapInXml();
//			//throw std::runtime_error("VRPro::getData not implemented.");
//		}

		void VideoRay::handleInput(const boost::system::error_code& error, const size_t transferred)
		{
			ROS_ERROR("DEbug1");
			//std::cout<<"Input."<<std::endl;
			if (!error && (transferred == comms->inputBuffer.size()))
			{
				ROS_ERROR("DEbug1-1");
				comms->decode(states);
				flag = true;
				ROS_ERROR("DEbug1-2");
			}
			else
			{
				std::cerr<<"Communication error. Received:"<<transferred<<", expected"
						<<comms->inputBuffer.size()<<std::endl;
			}
			ROS_ERROR("DEbug2");
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

			auv_msgs::BodyForceReq tau;
			ROS_ERROR("joystic");


				//if(data->buttons[1]){

					tau.wrench.force.z = data->axes[3];

				//} else if(data->axes[0]){

					tau.wrench.force.x = data->axes[1];

				//} else if(data->axes[2]){

					tau.wrench.torque.z = data->axes[2];
				//}

				setTAU(tau);
			}

	}
}


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"videoray_node");
	ros::NodeHandle nh;



	/*<!-- Serial config parameters -->
	<param name="PortName" value="/dev/ttyUSB2"/>
	<param name="BaudRate" value="115200"/>
	<param name="FlowControl" value="none"/>
	<param name="Parity" value="none"/>
	<param name="StopBits" value="1"/>
	<param name="DataBits" value="8"/>*/

	/* Start serial communication with VideoRay */
	//labust::vehicles::VideoRay VR("/dev/ttyUSB0", 115200, 0, 0, 1, 8);
	labust::vehicles::VideoRay VR("/dev/ttyUSB0", 9600, 0, 0, 1, 8);
	//labust::vehicles::VideoRay VR("/dev/pts/5", 115200, 0, 0, 1, 8);
	//labust::vehicles::VideoRay VR("/dev/pts/5", 9600, 0, 0, 1, 8);

	ros::Subscriber subJoy = nh.subscribe<sensor_msgs::Joy>("joy",1,&labust::vehicles::VideoRay::onJoy,&VR);

	auv_msgs::NavSts state;

	ros::Rate rate(1/0.1);

	while(ros::ok()){

		//ROS_ERROR("DEBUG1");

	//	auv_msgs::BodyForceReq tau;

	//	tau.wrench.force.x = 0;
	//	tau.wrench.force.z = 0;
	//	tau.wrench.torque.z = 0;

		//VR.setTAU(tau);

		//ROS_ERROR("DEBUG2");

		if(VR.flag){
			VR.getState(state);
			VR.flag = false;
		}

		//std::cout<<"Heading:"<<state.orientation.yaw<<std::endl;
		ROS_ERROR("Orientation %f", state.orientation.yaw);
		ROS_ERROR("depth %f", state.position.depth);

		//std::cout<<"Pitch:"<<state[labust::vehicles::state::pitch]<<std::endl;
		//std::cout<<"Roll:"<<state[labust::vehicles::state::roll]<<std::endl;
		//std::cout<<"z:"<<state.position.depth<<std::endl;
		//std::cout<<"Pressure:"<<state[labust::vehicles::state::depthPressure]<<std::endl;

		rate.sleep();
		ros::spinOnce();
	}

	//labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	//reader->useNode(reader->value<_xmlNode*>("//UVApp[@id='vr']"));
	//labust::vehicles::VRPro vr(reader,"");

	//labust::tools::wait_until_ms delay(100);

//	int i = 0;
//
//	while(true)
//	{
//		labust::vehicles::tauMap tau;
//		tau[labust::vehicles::tau::X] = 0;
//		tau[labust::vehicles::tau::Z] = 0;
//		tau[labust::vehicles::tau::N] = 0;
//		std::string temp = "0";
//		vr.setCommand(temp);
//		vr.setTAU(tau);
//		labust::vehicles::stateMap state;
//		vr.getState(state);
//
//		std::cout<<"Heading:"<<state[labust::vehicles::state::heading]<<std::endl;
//		std::cout<<"Pitch:"<<state[labust::vehicles::state::pitch]<<std::endl;
//		std::cout<<"Roll:"<<state[labust::vehicles::state::roll]<<std::endl;
//		std::cout<<"z:"<<state[labust::vehicles::state::z]<<std::endl;
//		std::cout<<"Pressure:"<<state[labust::vehicles::state::depthPressure]<<std::endl;
//
//		std::cout.precision(6);
//		std::cout<<"Time:"<<std::fixed<<labust::tools::unix_time()<<std::endl;
//
//		delay();
//		++i;
//	}
//
//	labust::vehicles::tauMap tau;
//	vr.setTAU(tau);
	//ros::spin();
	return 0;
}


