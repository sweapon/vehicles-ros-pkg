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
#include <sensor_msgs/Temperature.h>

class UROSLogger{

public:

	UROSLogger():rhodamineData(-1.0), temperatureData(-1.0), counter(0), badReading(-1.0){

		ros::NodeHandle nh;

		subRhodamineData = nh.subscribe<std_msgs::Float32>("adc", 1, &UROSLogger::onRhodamineData, this);
		subTemperatureData = nh.subscribe<sensor_msgs::Temperature>("temp", 1, &UROSLogger::onTemperatureData, this);
		subPositionData = nh.subscribe<auv_msgs::NavSts>("state_out",1, &UROSLogger::onPositionData, this);

		pubChangeMission = nh.advertise<auv_msgs::NavSts>("change_mission", 1);

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
			log_file << "%" << "LAUV-LUPIS-1, Rhodamine, 10 Hz" << std::endl;
			log_file << "%" << ss.str() << std::endl;
			log_file << "%-1" << std::endl;
			log_file << "%Time (unix time), Latitude (decimal degree), Longitude(decimal degree), Depth (m), Rhodamine (ppb), Crude, Refined, Temperature (C)" << std::endl;

			return true;
		} else {
			return false;
		}
	}

	void writeLog(){

		std::stringstream ss;
		ss << std::fixed;
		ss <<std::time(0)<<","<<std::setprecision(6)<<latLonData.global_position.latitude<<","<<latLonData.global_position.longitude<<","<<latLonData.position.depth<<","<<rhodamineData<<",-1,-1,"<<temperatureData;
		log_file << ss.str() << std::endl;

	}

	void onRhodamineData(const std_msgs::Float32::ConstPtr& data){
		rhodamineData = data->data;
	}

	void onPositionData(const auv_msgs::NavSts::ConstPtr& data){
		latLonData = *data;
        if((counter++)%2 == 0){ /* Set logging frequency */
        	if(log_file.is_open())
            	writeLog();
		}
	}

	void onTemperatureData(const sensor_msgs::Temperature::ConstPtr& data){
		temperatureData = data->temperature;

	}

	std::ofstream log_file;

	ros::Subscriber subRhodamineData, subPositionData, subTemperatureData;

	ros::Publisher pubChangeMission;

	double rhodamineData;
	double temperatureData;
	auv_msgs::NavSts latLonData;
    unsigned int counter;
    const double badReading;

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



