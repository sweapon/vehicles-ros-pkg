/*********************************************************************
 * uros_mission_change.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, LABUST, UNIZG-FER
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
#include <queue>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <auv_msgs/NavSts.h>
#include <misc_msgs/RhodamineAdc.h>


#include <labust/tools/GeoUtilities.hpp>

class UROSMissionChange{

public:

	enum {IN_PROGRESS=0, FINISHED};

	UROSMissionChange():rhodamineData(-1.0),
			              treshold(10.0),
			              n_avg(10),
			              avg(0),
			              smallRowsActive(false),
			              autoStart(false){

		ros::NodeHandle nh, ph("~");

		subRhodamineData = nh.subscribe<misc_msgs::RhodamineAdc>("adc", 1, &UROSMissionChange::onRhodamineData, this);
		subPositionData = nh.subscribe<auv_msgs::NavSts>("state_out",1, &UROSMissionChange::onPositionData, this);
		//subMissionState = nh.subscribe<auv_msgs::NavSts>("dune_mission_state",1, &UROSMissionChange::onMissionState, this);
		subCmd = nh.subscribe<std_msgs::UInt8>("cmd",1, &UROSMissionChange::onCmd, this);

		pubChangeMission = nh.advertise<auv_msgs::NavSts>("change_mission", 1);
		pubAbort = nh.advertise<std_msgs::Bool>("abort_in", 1);


		ph.param("autoStart", autoStart, autoStart);

		/*** Init Simple moving average filter ***/
		for(int i = 0; i < n_avg-1; i++){
			samples_avg.push(0);
		}
	}

	~UROSMissionChange(){

	}

	void onRhodamineData(const misc_msgs::RhodamineAdc::ConstPtr& data){
		rhodamineData = data->adc;

		/*** Calculate Simple moving average ***/
		avg += (rhodamineData - samples_avg.front())/n_avg;
		samples_avg.pop();
		samples_avg.push(rhodamineData);

		double diffLat = latLonData.latitude - lastRowsPos.latitude;
		double diffLon = latLonData.longitude - lastRowsPos.longitude;

		std::pair<double,double> distance = labust::tools::deg2meter(diffLat, diffLon, lastRowsPos.latitude);

		//if(abs(distance.first) > 20 && abs(distance.second) > 20){
			if(avg > treshold && !smallRowsActive && autoStart){
				changeMission();
			}
		//}
	}

	void onPositionData(const auv_msgs::NavSts::ConstPtr& data){
		latLonData = data->global_position;

	}

	void onMissionState(const auv_msgs::NavSts::ConstPtr& data){

		if(1 == FINISHED){
			smallRowsActive = false;
		}
	}

	void onCmd(const std_msgs::UInt8::ConstPtr& data){

		if((data->data) & 0x01){
			/*** Abort mission ***/
			std_msgs::Bool data;
			data.data = true;
			pubAbort.publish(data);
			ROS_ERROR("ABORT CMD RECEIVED");

		} else if((data->data) & 0x08){
			/*** Change mission ***/
			changeMission();
			ROS_ERROR("CHANGE MISSION CMD RECEIVED");
		}
	}

	void changeMission(){
		avg = 0;
		smallRowsActive = true;
		lastRowsPos = latLonData;

		auv_msgs::NavSts msg;
		msg.global_position.latitude = latLonData.latitude;
		msg.global_position.longitude = latLonData.longitude;

		pubChangeMission.publish(msg);
	}

	ros::Subscriber subRhodamineData, subPositionData, subMissionState, subCmd;
	ros::Publisher pubChangeMission, pubAbort;

	auv_msgs::DecimalLatLon latLonData;
	auv_msgs::DecimalLatLon lastRowsPos;
	double rhodamineData;
	double treshold;

	int n_avg;
	std::queue<double> samples_avg;
	double avg;

	bool smallRowsActive, autoStart;

};

int main(int argc, char* argv[]){

	ros::init(argc,argv,"uros_mission_change_node");
	UROSMissionChange MC;
	ros::spin();
	return 0;
}



