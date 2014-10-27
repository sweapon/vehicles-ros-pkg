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
 *
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/vehicles/Pro4Driver.hpp>
#include <labust/vehicles/CSRMap.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/StringUtilities.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>

#include <boost/bind.hpp>
#include <boost/crc.hpp>
#include <boost/serialization/collections_save_imp.hpp>

#include <iosfwd>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)
PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(boost::archive::binary_oarchive)

using namespace labust::vehicles;

Pro4Driver::Pro4Driver():
		port(io),
		ringBuffer(header_len,0),
		thrusterId(1,0),
		networkId(0x01),
		max(0.15),
		min(-max),
		maxTemp(70),
		nextId(0),
		lastId(0)
{
	this->onInit();
}

Pro4Driver::~Pro4Driver()
{
	io.stop();
	runner.join();
}

void Pro4Driver::onInit()
{
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = this->setup_port()) && ros::ok())
	{
		ROS_ERROR("Pro4Driver::Failed to open port.");
		r.sleep();
	}

	//Read configuration
	XmlRpc::XmlRpcValue data;
	std::string name("thrusterId");
	if (ph.hasParam(name))
	{
		ph.getParam(name, data);
		if (data.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			thrusterId.clear();
			for(size_t i=0; i<data.size(); ++i)
				thrusterId.push_back(uint8_t(static_cast<int>(data[i])));
		}
	}

	name = "thrusterDir";
	if (ph.hasParam(name))
	{
		ph.getParam(name, data);
		if (data.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			thrusterDir.clear();
			for(size_t i=0; i<data.size(); ++i)
				thrusterDir.push_back(static_cast<int>(data[i]));
		}
	}
	else
	{
		thrusterDir.clear();
		for(int i=0; i<thrusterId.size(); ++i) thrusterDir.push_back(1);
	}

	ph.param("max", max, max);
	ph.param("min", min, min);

	//Setup publisher
	diagnostic = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",1);
	imu = nh.advertise<sensor_msgs::Imu>("imu",1);
	pressure = nh.advertise<sensor_msgs::FluidPressure>("pressure",1);
	//Setup subscribers
	thrustIn = nh.subscribe<std_msgs::Float32MultiArray>("pwm_in",1,&Pro4Driver::onThrustIn,this);

	if (setupOk)
	{
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}
}

void Pro4Driver::start_receive()
{
	using namespace boost::asio;
	async_read(port, buffer.prepare(header_len),
			boost::bind(&Pro4Driver::onHeader, this, _1,_2));
}

bool Pro4Driver::setup_port()
{
	ros::NodeHandle ph("~");
	std::string portName("/dev/ttyUSB0");
	int baud(115200);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	using namespace boost::asio;
	port.open(portName);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

	return port.is_open();
}

void Pro4Driver::onHeader(const boost::system::error_code& e,
		std::size_t size)
{
	ROS_INFO("Send time header: %f", (ros::Time::now()-newmsg).toSec());
	if (!e)
	{
		ROS_INFO("Header read size: %d",size);

		buffer.commit(size);

		if (size == 1)
		{
			//Put the new byte on the end of the ring buffer
			ringBuffer.push_back(buffer.sbumpc());
			//Test framing
			//if (ringBuffer[0] == workaroundSync)
			//{
			//	ROS_WARN("Applying framing workaround.");
			//	ringBuffer[0] = 0xF0;
			//}
		}
		else
		{
			//Workaround is possible on each full read
			int peek = buffer.sgetc();
			ROS_WARN("FIRST BUFFER ELEMENT:%d", peek);
			buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);
			//Correct the framing error
			//if (peek == workaroundSync)
			//{
			//	ROS_WARN("Applying framing workaround.");
			//	ringBuffer[0] = 0xF0;
			//}
		}

		uint16_t* sync = reinterpret_cast<uint16_t*>(ringBuffer.data());
		//Check sync
		if ((*sync) == syncin)
		{
			ROS_INFO("Sync found.");
			//Check header checksum
			//boost::crc_32_type result;
			//result.process_block(&ringBuffer[header_byte], &ringBuffer[checksum_byte]);
			//uint32_t* chk = reinterpret_cast<uint32_t*>(&ringBuffer[checksum_byte]);
			const uint8_t* pt = reinterpret_cast<const uint8_t*>(&ringBuffer[header_byte]);
			uint8_t chkin = labust::tools::getChecksum(pt, header_len-1);
			//if (result.checksum() == (*chk))
			if (chkin == ringBuffer[checksum_byte])
			{
				ROS_INFO("Checksum ok. Payload size: %d", ringBuffer[size_byte]);
				boost::asio::async_read(port, buffer.prepare(ringBuffer[size_byte]+chk_len),
						boost::bind(&Pro4Driver::onData,this,_1,_2));
				return;
			}
			else
			{
				//ROS_ERROR("Checksum failed: got: %d, calc:%d", *chk, result.checksum());
				ROS_ERROR("Checksum failed: got: %d, calc:%d", ringBuffer[checksum_byte], chkin);
			}
		}
		else
		{
			ROS_ERROR("Check failed.");
			std::cout<<"RINGBUFFER:";
			std::cout.width(2);
			std::cout.fill('0');
			for(int i=0; i<ringBuffer.size(); ++i)
			{
				std::cout<<std::hex<<std::fixed<<int(ringBuffer[i])<<",";
			}
			std::cout<<std::endl;
			ringBuffer.erase(ringBuffer.begin());

			boost::asio::async_read(port,
					buffer.prepare(1),
					boost::bind(&Pro4Driver::onHeader,this,_1,_2));
			return;
		}
	}
	else
	{
		ROS_ERROR("Pro4Driver: %s",e.message().c_str());
	}
	this->start_receive();
}

void Pro4Driver::onData(const boost::system::error_code& e,
		std::size_t size)
{

	ROS_INFO("Send time: %f", (ros::Time::now()-newmsg).toSec());
	ROS_INFO("Received data size: %d",size);
	if (!e)
	{
		buffer.commit(size);
		boost::archive::binary_iarchive dataSer(buffer, boost::archive::no_header);
		std::istream is(&buffer);
		std::string payload(size-chk_len,'\0');
		is.read(&payload[0],size-chk_len);
		uint8_t chk;
		dataSer >> chk;

		//boost::crc_32_type result;
		//result.process_bytes(payload.data(), payload.size());
		const uint8_t* pt = reinterpret_cast<const uint8_t*>(payload.c_str());
		uint8_t chkin = labust::tools::getChecksum(pt, payload.size());
		//if (result.checksum() == chk)
		if (chkin == chk)
		{
			ROS_INFO("Payload checksum ok.");

			std::istringstream is;
			is.rdbuf()->pubsetbuf(&payload[0],payload.size());
			boost::archive::binary_iarchive paySer(is, boost::archive::no_header);
			VRStdResponse data;
			paySer >> data;

			//Check cheksum
			ROS_INFO("New update:");
			ROS_INFO("\tHeading=%d", data.heading);
			ROS_INFO("\tPitch=%d", data.pitch);
			ROS_INFO("\tRoll=%d", data.roll);
			ROS_INFO("\tPressure=%d", data.pressure);

			this->processStdReply(data);

//			std::string keys[]={"voltage", "current", "rpm", "temp", "fault"};
//			float values[]={data.bus_v, data.bus_i, data.rpm, data.temp, float(data.fault)};
//			diagnostic_msgs::DiagnosticStatus diag;
//			std::stringstream out;
//			//Number of thrusters - remaining in queue
//			// - single that was already sent
//			int lid = lastId;//thrusterId.size() - output.size() - 1;
//			if ((lid >= 0) && (lid < thrusterId.size()))
//			{
//				out<<"Pro4Driver"<<int(thrusterId[lid]);
//				diag.hardware_id=out.str();
//				diag.name = out.str();
//				if (data.temp >= maxTemp)
//				{
//					diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
//					diag.message = "I'm overheating, bitch!";
//				}
//				diag.message = ((data.rpm < 20)?"Doing static impressions":"Spinnin 'n shit");
//
//				if ((fabs(data.rpm) < 20) && (fabs(data.bus_i) > 0.5))
//				{
//					diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
//					diag.message = "I'm stuck, punk!";
//				}
//
//				for (int i=0; i<5;++i)
//				{
//					//Send diagnostic or add to queue
//					diagnostic_msgs::KeyValue pair;
//					std::ostringstream out;
//					out<<values[i];
//					pair.key = keys[i];
//					pair.value = out.str();
//					diag.values.push_back(pair);
//				}
//				diagnosticArray.status.push_back(diag);
//			}
//			else
//			{
//				ROS_WARN("Bad ID number deduction.");
//			}
		}
		else
		{
			//ROS_ERROR("Checksum failed: got: %d, calc:%d", chk, result.checksum());
			ROS_ERROR("Checksum failed: got: %d, calc:%d", chk, chkin);
		}
	}

	ROS_INFO("Remaining in buffer: %d",buffer.in_avail());

	//Empty queue whatever was received
	this->sendSingle();
	this->start_receive();
}

void Pro4Driver::onThrustIn(const std_msgs::Float32MultiArray::ConstPtr& thrust)
{
	ROS_WARN("Callback distance: %f", (ros::Time::now() - newmsg).toSec());
	newmsg = ros::Time::now();

	std::ostringstream t;
	boost::archive::binary_oarchive thrustSer(t, boost::archive::no_header);
	//Serialize the outgoing thrust for available thrusters
	for (int i=0; i<nthrust; ++i)
	{
		short int t(0);
		if (thrust->data.size() > i)
		{
			t = static_cast<short int>(thustmul*
					labust::math::coerce(thrust->data[i], min, max));
		}
		thrustSer<<t;
	}

	uint16_t light = 0;
	thrustSer<<light;

	//Add light and camera control here here
	//thustSer<<static_cast<uint8_t>(light);
	//thustSer<<static_cast<uint8_t>(tilt);
	//thustSer<<static_cast<uint8_t>(focus);
	//thustSer<<static_cast<uint8_t>(tilt control);
	//thustSer<<static_cast<uint8_t>(focus control);

	std::ostringstream out;
	boost::archive::binary_oarchive dataSer(out, boost::archive::no_header);
	VRHeader header;
	header.sync = syncout;
	header.csraddr = Command::csrstart;
	header.flags = Response::vrstdresponse;
	header.network_id = networkId;
	//Size of thruster info + header and ID bytes
	header.length = t.str().size();
	//Add the header
	dataSer<<header;
	//Calculate checksum
	const uint8_t* pt = reinterpret_cast<const unsigned char*>(out.str().c_str());
	uint8_t chk = labust::tools::getChecksum(pt, out.str().size());
	//boost::crc_32_type checksum;
	//checksum.process_bytes(out.str().c_str(), out.str().size());
	//Add the checksum
	//uint32_t chk = checksum.checksum();
	dataSer<<chk;

	std::ostringstream pay;
	boost::archive::binary_oarchive payload(pay, boost::archive::no_header);
	//Add the payload command
	//uint8_t hdr(0xAA);
	//payload<<hdr;
	//Add the node id
	//payload<<nextId;
	//Add all thruster values
	//clean_string_serialize(payload, t.str());
	//Add payload
	//clean_string_serialize(dataSer, pay.str());
	clean_string_serialize(dataSer, t.str());

	//Calculate checksum
	pt = reinterpret_cast<const unsigned char*>(out.str().c_str());
	chk = labust::tools::getChecksum(pt, out.str().size());
	//checksum.reset();
	//checksum.process_bytes(pay.str().c_str(), pay.str().size());
	//chk = checksum.checksum();
	dataSer<<chk;

	boost::mutex::scoped_lock l(queueMux);
	output.push(out.str());
	l.unlock();

	std::cout<<"Sending message:";
	for (int i=0; i<out.str().size(); ++i)
	{
		std::cout.width(2);
		std::cout.fill('0');
		std::cout<<std::hex<<std::fixed<<uint32_t(uint8_t(out.str()[i]));
	}
	std::cout<<std::endl;

	//Start emptying queue
	ROS_INFO("Sending: %d for ID=%d", output.size(), nextId);
	//Advance Id
	lastId = nextId;
	nextId = (++nextId)%thrusterId.size();
	sendSingle();

	//Simulator for diagnostic output
	///\todo Remove this simulation code
//	std::string keys[]={"voltage", "current", "rpm", "temp", "fault"};
//	float values[]={10, 20, 30, 35, 40};
//	diagnostic_msgs::DiagnosticStatus diag;
//	diag.hardware_id="Pro4Driver0";
//	diag.name="Pro4Driver0";
//	diag.level = diagnostic_msgs::DiagnosticStatus::OK;
//
//	for (int i=0; i<5;++i)
//	{
//		//Send diagnostic or add to queue
//		diagnostic_msgs::KeyValue pair;
//		std::ostringstream out;
//		out<<values[i];
//		pair.key = keys[i];
//		pair.value = out.str();
//		diag.values.push_back(pair);
//	}
//
//	diagnosticArray.status.push_back(diag);
//	diag.hardware_id="Pro4Driver2";
//	diag.name="Pro4Driver2";
//	diag.message = "Overheating";
//	diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
//	diagnosticArray.status.push_back(diag);
//	diagnosticArray.header.stamp = ros::Time::now();
//	diagnostic.publish(diagnosticArray);

}

void Pro4Driver::sendSingle()
{
	boost::mutex::scoped_lock l(queueMux);
	if (output.empty())
	{
		//Close loop and publish diagnostics
		diagnosticArray.header.stamp = ros::Time::now();
		diagnostic.publish(diagnosticArray);
		diagnosticArray.status.clear();
		return;
	}

	std::string front = output.front();
	output.pop();

	boost::asio::async_write(port, boost::asio::buffer(front),
			boost::bind(&Pro4Driver::onSent, this, _1, _2));
}

void Pro4Driver::onSent(const boost::system::error_code& e, std::size_t size)
{
	ROS_INFO("Message sent time:%f",(ros::Time::now()-newmsg).toSec());
}

void Pro4Driver::processStdReply(const VRStdResponse& rep)
{
	sensor_msgs::Imu::Ptr imuout(new sensor_msgs::Imu());
	labust::tools::quaternionFromEulerZYX(M_PI*rep.roll/1800.,
			M_PI*rep.pitch/1800.,
			M_PI*rep.heading/1800.,
			imuout->orientation);
	imuout->header.frame_id = "base_link";
	imuout->header.stamp = ros::Time::now();
	imu.publish(imuout);

	sensor_msgs::FluidPressure::Ptr pressureout(new sensor_msgs::FluidPressure());
	pressureout->fluid_pressure = rep.pressure;
	pressureout->header.frame_id = "base_link";
	pressureout->header.stamp = ros::Time::now();
	pressure.publish(pressureout);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"pro4_node");
	Pro4Driver node;
	ros::spin();

	return 0;
}


