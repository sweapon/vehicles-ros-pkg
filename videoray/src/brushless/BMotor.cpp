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
#include <labust/vehicles/BMotor.hpp>
#include <labust/vehicles/CSRMap.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <boost/bind.hpp>
#include <boost/crc.hpp>
#include <boost/serialization/collections_save_imp.hpp>

#include <iosfwd>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)
PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(boost::archive::binary_oarchive)

using namespace labust::vehicles;

BMotor::BMotor():
										port(io),
										ringBuffer(header_len,0),
										thrusterId(1,0),
										networkId(0x81),
										max(0.05),
										min(-max),
										maxTemp(70)
{
	this->onInit();
}

BMotor::~BMotor()
{
	io.stop();
	runner.join();
}

void BMotor::onInit()
{
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = this->setup_port()) && ros::ok())
	{
		ROS_ERROR("BMotor::Failed to open port.");
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
	ph.param("max", max, max);
	ph.param("min", min, min);

	//Setup publisher
	diagnostic = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",1);
	//Setup subscribers
	thrustIn = nh.subscribe<std_msgs::Float32MultiArray>("thrust_in",1,&BMotor::onThrustIn,this);

	if (setupOk)
	{
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}
}

void BMotor::start_receive()
{
	using namespace boost::asio;
	async_read(port, buffer.prepare(header_len),
			boost::bind(&BMotor::onHeader, this, _1,_2));
}

bool BMotor::setup_port()
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
			serial_port::flow_control::hardware));

	return port.is_open();
}

void BMotor::onHeader(const boost::system::error_code& e,
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
			if (ringBuffer[0] == workaroundSync)
			{
				ROS_WARN("Applying framing workaround.");
				ringBuffer[0] = 0xF0;
			}
		}
		else
		{
			//Workaround is possible on each full read
			int peek = buffer.sgetc();
			ROS_WARN("FIRST BUFFER ELEMENT:%d", peek);
			buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);
			//Correct the framing error
			if (peek == workaroundSync)
			{
				ROS_WARN("Applying framing workaround.");
				ringBuffer[0] = 0xF0;
			}
		}

		uint16_t* sync = reinterpret_cast<uint16_t*>(ringBuffer.data());
		//Check sync
		if ((*sync) == syncin)
		{
			ROS_INFO("Sync found.");
			//Check header checksum
			boost::crc_32_type result;
			result.process_block(&ringBuffer[header_byte], &ringBuffer[checksum_byte]);
			uint32_t* chk = reinterpret_cast<uint32_t*>(&ringBuffer[checksum_byte]);
			if (result.checksum() == (*chk))
			{
				ROS_INFO("Checksum ok. Payload size: %d", ringBuffer[size_byte]);
				boost::asio::async_read(port, buffer.prepare(ringBuffer[size_byte]+chk_len),
						boost::bind(&BMotor::onData,this,_1,_2));
				return;
			}
			else
			{
				ROS_ERROR("Checksum failed: got: %d, calc:%d", *chk, result.checksum());
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
					boost::bind(&BMotor::onHeader,this,_1,_2));
			return;
		}
	}
	else
	{
		ROS_ERROR("BMotor: %s",e.message().c_str());
	}
	this->start_receive();
}

void BMotor::onData(const boost::system::error_code& e,
		std::size_t size)
{

	ROS_INFO("Send time: %f", (ros::Time::now()-newmsg).toSec());
	ROS_INFO("Received data size: %d",size);
	if (!e)
	{
		buffer.commit(size);
		boost::archive::binary_iarchive dataSer(buffer, boost::archive::no_header);
		std::istream is(&buffer);
		std::string payload(size-4,'\0');
		is.read(&payload[0],size-4);
		uint32_t chk;
		dataSer >> chk;

		boost::crc_32_type result;
		result.process_bytes(payload.data(), payload.size());

		if (result.checksum() == chk)
		{
			ROS_INFO("Payload checksum ok.");

			std::istringstream is;
			is.rdbuf()->pubsetbuf(&payload[0],payload.size());
			boost::archive::binary_iarchive paySer(is, boost::archive::no_header);
			TStdResponse data;
			paySer >> data;

			//Check cheksum
			ROS_INFO("New update:");
			ROS_INFO("\tVoltage=%f", data.bus_v);
			ROS_INFO("\tCurrent=%f", data.bus_i);
			ROS_INFO("\tRPM=%f", data.rpm);
			ROS_INFO("\tTemperature=%f", data.temp);
			ROS_INFO("\tFault=%d", data.fault);

			std::string keys[]={"voltage", "current", "rpm", "temp", "fault"};
			float values[]={data.bus_v, data.bus_i, data.rpm, data.temp, float(data.fault)};
			diagnostic_msgs::DiagnosticStatus diag;
			std::stringstream out;
			//Number of thrusters - remaining in queue
			// - single that was already sent
			int lid = thrusterId.size() - output.size() - 1;
			if ((lid >= 0) && (lid < thrusterId.size()))
			{
				out<<"BMotor"<<thrusterId[lid];
				diag.hardware_id=out.str();
				diag.name = out.str();
				if (data.temp >= maxTemp)
				{
					diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
					diag.message = "I'm overheating, bitch!";
				}
				diag.message = ((data.rpm < 20)?"Doing static impressions":"Spinnin 'n shit");

				if ((data.rpm < 20) && (data.bus_i > 0.5))
				{
					diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
					diag.message = "I'm stuck, punk!";
				}

				for (int i=0; i<5;++i)
				{
					//Send diagnostic or add to queue
					diagnostic_msgs::KeyValue pair;
					std::ostringstream out;
					out<<values[i];
					pair.key = keys[i];
					pair.value = out.str();
					diag.values.push_back(pair);
				}
				diagnosticArray.status.push_back(diag);
			}
			else
			{
				ROS_WARN("Bad ID number deduction.");
			}
		}
		else
		{
			ROS_ERROR("Checksum failed: got: %d, calc:%d", chk, result.checksum());
		}
	}

	ROS_INFO("Remaining in buffer: %d",buffer.in_avail());

	//Empty queue whatever was received
	this->sendSingle();
	this->start_receive();
}

void BMotor::onThrustIn(const std_msgs::Float32MultiArray::ConstPtr& thrust)
{
	ROS_WARN("Callback distance: %f", (ros::Time::now() - newmsg).toSec());
	newmsg = ros::Time::now();
	//Clear queue and report warning if not empty
	if (!output.empty())
	{
		ROS_ERROR("Queue is not empty.");
		boost::mutex::scoped_lock l(queueMux);
		//Empty queue
		while (!output.empty()) output.pop();
	}

	int nthrust = thrusterId.size();
	if (thrust->data.size() < thrusterId.size())
	{
		nthrust = thrust->data.size();
		ROS_WARN("Less thruster values than thrusters. Setting remaining to zero.");
	}

	std::ostringstream t;
	boost::archive::binary_oarchive thrustSer(t, boost::archive::no_header);
	//Serialize the outgoing thrust for available thrusters
	for (int i=0; i<thrusterId.size(); ++i)
	{
		float t = ((i<nthrust)?thrust->data[i]:0);
		t = labust::math::coerce(t, min, max);
		thrustSer<<t;
	}

	//Queue up messages for each thruster
	for (int i=0;i<thrusterId.size();++i)
	{
		std::ostringstream out;
		boost::archive::binary_oarchive dataSer(out, boost::archive::no_header);
		VRHeader header;
		header.sync = syncout;
		header.csraddr = Command::custom;
		header.flags = Response::tstdresponse;
		header.network_id = networkId;
		//Size of thruster info + header and ID bytes
		header.length = t.str().size() + 2;
		//Add the header
		dataSer<<header;
		//Calculate checksum
		boost::crc_32_type checksum;
		checksum.process_bytes(out.str().c_str(), out.str().size());
		//Add the checksum
		uint32_t chk = checksum.checksum();
		dataSer<<chk;

		std::ostringstream pay;
		boost::archive::binary_oarchive payload(pay, boost::archive::no_header);
		//Add the payload command
		uint8_t hdr(0xAA);
		payload<<hdr;
		//Add the node id
		payload<<thrusterId[i];
		//Add all thruster values
		clean_string_serialize(payload, t.str());
		//Add payload
		clean_string_serialize(dataSer, pay.str());

		//Calculate checksum
		checksum.reset();
		checksum.process_bytes(pay.str().c_str(), pay.str().size());
		chk = checksum.checksum();
		dataSer<<chk;

		boost::mutex::scoped_lock l(queueMux);
		output.push(out.str());

		/*std::cout<<"Sending message:";
		for (int i=0; i<out.str().size(); ++i)
		{
			std::cout.width(2);
			std::cout.fill('0');
			std::cout<<std::hex<<std::fixed<<uint32_t(uint8_t(out.str()[i]));
		}
		std::cout<<std::endl;*/
	}

	//Start emptying queue
	ROS_INFO("Sending: %d", output.size());
	sendSingle();

	//Simulator for diagnostic output
	///\todo Remove this simulation code
//	std::string keys[]={"voltage", "current", "rpm", "temp", "fault"};
//	float values[]={10, 20, 30, 35, 40};
//	diagnostic_msgs::DiagnosticStatus diag;
//	diag.hardware_id="BMotor0";
//	diag.name="BMotor0";
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
//	diag.hardware_id="BMotor2";
//	diag.name="BMotor2";
//	diag.message = "Overheating";
//	diag.level = diagnostic_msgs::DiagnosticStatus::WARN;
//	diagnosticArray.status.push_back(diag);
//	diagnosticArray.header.stamp = ros::Time::now();
//	diagnostic.publish(diagnosticArray);

}

void BMotor::sendSingle()
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
			boost::bind(&BMotor::onSent, this, _1, _2));
}

void BMotor::onSent(const boost::system::error_code& e, std::size_t size)
{
	ROS_INFO("Message sent time:%f",(ros::Time::now()-newmsg).toSec());
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"bmotor_node");
	BMotor node;
	ros::spin();

	return 0;
}


