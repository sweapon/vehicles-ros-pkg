/********************************************************************
 * ethernet_relay_driver.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: Filip Mandić
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
*  "AS IS" AND ANExternalEventY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include <boost/system/system_error.hpp>
#include <boost/asio.hpp>

#include <labust/drivers/ethernetRelayDriver.hpp>



int main(int argc, char* argv[]){

	ros::init(argc,argv,"ethernet_relay_node");
	ros::NodeHandle nh;

	using namespace labust::drivers;

	std::vector<bool> test(8,0);
	test[1] = true;

	EthernetRelayDriver ERD("192.168.1.4", 17494);


//	ERD.relayState = test;
//
//	if(ERD.encode(EthernetRelayDriver::digitalSet)){
//		ROS_ERROR("Poslano");
//	}
//
//	ERD.decode();



//	/*<!-- Serial config parameters -->
//	<param name="PortName" value="/dev/ttyUSB0"/>
//	<param name="BaudRate" value="9600"/>
//	<param name="FlowControl" value="none"/>
//	<param name="Parity" value="none"/>
//	<param name="StopBits" value="1"/>
//	<param name="DataBits" value="8"/>*/
//
//	/* Start serial communication with VideoRay */
//	labust::vehicles::VideoRay VR("/dev/ttyUSB0", 9600, 0, 0, 1, 8);
//
//	ros::Subscriber subJoy = nh.subscribe<sensor_msgs::Joy>("joy",1,&labust::vehicles::VideoRay::onJoy,&VR);
//
//	auv_msgs::NavSts state;
//
//	ros::Rate rate(1/0.1);
//
//	while(ros::ok()){
//
//		if(VR.flag){
//			VR.getState(state);
//			VR.flag = false;
//		}
//
//		ROS_ERROR("Orientation %f", state.orientation.yaw);
//		ROS_ERROR("depth %f", state.position.depth);
//
//		rate.sleep();
//		ros::spinOnce();
//	}

	ros::spin();

	return 0;
}
