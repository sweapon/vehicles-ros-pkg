/*
 * uros_logger.cpp
 *
 *  Created on: Sep 3, 2014
 *      Author: Filip Mandic
 */

#include <iostream>
#include <fstream>
#include <ctime>

#include <boost/date_time.hpp>

#include <ros/ros.h>

class UROSLogger{

public:

	UROSLogger(){

		ros::NodeHandle nh;


	}

	~UROSLogger(){

	}

	bool start(){

		std::stringstream ss;

		/* Generate log filename */
		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y-%m-%d_%H-%M");
		ss.imbue(std::locale(ss.getloc(), facet));
		ss << "log_" << boost::posix_time::second_clock::universal_time() << ".csv";

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
			log_file << "%Time (unix time), Latitude (decimal degree), Longitude(decimal degree), Depth (m), Rhodamine (ppb),Crude, Refined" << std::endl;

			// Test line
			std::time_t t = std::time(0);
			ss.str(std::string());
			ss << t;

			log_file << ss.str() << std::endl;

			return true;
		} else {
			return false;
		}
	}

	std::ofstream log_file;

	ros::Subscriber subRhodamineData;


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



