/*********************************************************************
 * uros_logger.cpp
 *
 *  Created on: Sep 3, 2014
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

#include <iostream>
#include <fstream>
#include <ctime>

#include <boost/date_time.hpp>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <auv_msgs/NavSts.h>

class UROSLogger{

public:

	UROSLogger():rhodamineData(0.0){

		ros::NodeHandle nh;

		subRhodamineData = nh.subscribe<std_msgs::Float32>("adc", 1, &UROSLogger::onRhodamineData, this);
		subPositionData = nh.subscribe<auv_msgs::NavSts>("state_out",1, &UROSLogger::onPositionData, this);

	}

	~UROSLogger(){

	}

	bool start(){

		std::stringstream ss;

		/* Generate log filename */
		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y-%m-%d_%H-%M");
		ss.imbue(std::locale(ss.getloc(), facet));
		ss << "/home/stdops/logs/lauv-lupis-2/log_" << boost::posix_time::second_clock::universal_time() << ".csv";

		/* Open log file */
		log_file.open(ss.str().c_str(), std::ios::out);

		if(log_file.is_open()){

			/* Generate header data */
			boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%m/%d/%Y %H:%M");

			ss.imbue(std::locale(ss.getloc(), facet));
			ss.str(std::string());
			ss << boost::posix_time::second_clock::universal_time();

			/* Write header data to log */
			log_file << "%" << "LAUV-LUPIS,Rhodamine,1" << std::endl;
			log_file << ss.str() << std::endl;
			log_file << "%-9999" << std::endl;
			log_file << "%Time (unix time), Latitude (decimal degree), Longitude(decimal degree), Depth (m), Rhodamine (ppb), Crude, Refined" << std::endl;

			// Test line
//			std::time_t t = std::time(0);
//			ss.str(std::string());
//			ss <<t<<","<<latLonData.global_position.latitude<<","<<latLonData.global_position.longitude<<","<<latLonData.position.depth<<","<<rhodamineData<<",0,0";
//			log_file << ss.str() << std::endl;

			return true;
		} else {
			return false;
		}
	}

	void writeLog(){

		std::stringstream ss;
		ss <<std::time(0)<<","<<latLonData.global_position.latitude<<","<<latLonData.global_position.longitude<<","<<latLonData.position.depth<<","<<rhodamineData<<",0,0";
		log_file << ss.str() << std::endl;

	}

	void onRhodamineData(const std_msgs::Float32::ConstPtr& data){
		rhodamineData = data->data;
	}

	void onPositionData(const auv_msgs::NavSts::ConstPtr& data){
		latLonData = *data;
		if(log_file.is_open())
			writeLog();
	}

	std::ofstream log_file;

	ros::Subscriber subRhodamineData, subPositionData;

	double rhodamineData;
	auv_msgs::NavSts latLonData;


};

int main(int argc, char* argv[]){

	ros::init(argc,argv,"uros_logger_node");

	UROSLogger logger;
	if(logger.start()){

		ros::spin();
		logger.log_file.close();
		return 0;

	} else {

		ROS_ERROR("Cannot open log file.");
		return -1;
	}
}



