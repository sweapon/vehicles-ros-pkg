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
#include <labust/vehicles/PlaDyPosNode_v3.hpp>
#include <labust/vehicles/Allocation.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <string>
#include <sstream>

using namespace labust::vehicles;

PlaDyPosNode_v3::PlaDyPosNode_v3():
	Ub(18.5),
	Un(18.5),
	lastTau(ros::Time::now()),
	timeout(0.5),
	useWeighted(false),
	maxCap(0.7)
{
	/*
	//Seamor thruster
	posDir[0] = 1.376872219;
	negDir[0] = 1.097241348;

	posDir[1] = 1.908577382;
	negDir[1] = 0.740035409;

	posDir[2] = 2.088491416;
	negDir[2] = 0.718808411;

	posDir[3] = 1.922883428;
	negDir[3] = 0.82880469;
	*/

	posDir[0] = 1.0;
        negDir[0] = 1.0;

        posDir[1] = 1.0;
        negDir[1] = 1.0;

        posDir[2] = 1.0;
        negDir[2] = 1.0;

        posDir[3] = 1.0;
        negDir[3] = 1.0;

	/*for (int i=0; i<4; ++i)
	{
		posDir[i] /=2*sqrt(2);
		negDir[i] /=2*sqrt(2);
	}
	
	float divisor = negDir[2];
	for (int i=0; i<4; ++i)
	{
		posDir[i] /= divisor;
		negDir[i] /= divisor;
	}*/
}

PlaDyPosNode_v3::~PlaDyPosNode_v3()
{}

void PlaDyPosNode_v3::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Initialize subscribers and publishers
	tau = nh.subscribe("tauIn", 1, &PlaDyPosNode_v3::onTau,this);
	//batteryVoltage = nh.subscribe("battery_voltage", 1, &PlaDyPosNode_v3::onBatteryVoltage,this);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	revs = nh.advertise<std_msgs::Float32MultiArray>("pwm_out",1);
	
	//Initialize max/min tau
	double maxThrust(1),minThrust(-1);
	//ph.param("maxThrust",maxThrust,maxThrust);
	//ph.param("minThrust",minThrust,minThrust);
	ph.param("max_cap",maxCap,maxCap);
	ph.param("useWeighted",useWeighted, useWeighted);

	//Initialize the allocation matrix
	float cp(cos(M_PI/4)),sp(sin(M_PI/4));
	B<<cp,cp,-cp,-cp,
	   -sp,sp,-sp,sp,
	    -1,1,1,-1;

	BinvIdeal = B.transpose()*(B*B.transpose()).inverse();

	//Scaling allocation only for XYN
	allocator.configure(B,maxThrust,minThrust);
}

void PlaDyPosNode_v3::onBatteryVoltage(const std_msgs::Float32::ConstPtr voltage)
{
	if (voltage->data > 1) this->Ub = voltage->data-1;
}

void PlaDyPosNode_v3::onTau(const auv_msgs::BodyForceReq::ConstPtr tau)
{
	lastTau = ros::Time::now();
	//Perform allocation
	Eigen::Vector3f tauXYN,tauXYNsc;
	Eigen::Vector3f tauXY, tauN;
	Eigen::Vector4f tauI;
	Eigen::Vector4f tauTN, tauTXY;
	tauXYN<<tau->wrench.force.x,tau->wrench.force.y,tau->wrench.torque.z;

	tauXY<<tau->wrench.force.x,tau->wrench.force.y,0.0;
	tauTXY = BinvIdeal*tauXY;
	tauN<<0.0,0.0,tau->wrench.torque.z;
	tauTN = BinvIdeal*tauN;

	double tnmax((Ub*Ub)/(Un*Un)*maxCap / 2);
	double tnmin(-tnmax);
	double txymax(tnmax);
	double txymin(-txymax);

	bool satN = tauTN.cwiseAbs().maxCoeff() > tnmax;
	bool satXY = tauTXY.cwiseAbs().maxCoeff() > txymax;

	if (!satN && !satXY)
	{
	    //Everything is within limits
	    //ROS_INFO("Within limits.");
	}
	else if (satN && !satXY)
	{
	    //ROS_INFO("Yaw in saturation");
	    double delta = txymax - tauTXY.cwiseAbs().maxCoeff();

	    if (delta > 0)
	    {
	        tnmax = tnmax + delta;
	        tnmin = -tnmax;
	    }

	    for (int i=0; i<tauTN.size(); ++i)
      {
      	tauTN(i) = labust::math::coerce(tauTN(i), tnmin, tnmax);
      }

	    satN = tauTN.cwiseAbs().maxCoeff() > tnmax;
	}
	else if (satXY && !satN)
	{
	    //ROS_INFO("X-Y in saturation");
	    double delta = tnmax - tauTN.cwiseAbs().maxCoeff();

	    if (delta > 0)
	    {
	        txymax = txymax + delta;
	        txymin = -txymax;
	    }

	    allocator.configure(B,txymax,txymin);
	    double scale = allocator.scaleII(tauXY,&tauXYNsc,&tauTXY);
	    satXY = scale > 1;
	}
	else if (satXY && satN)
	{
	    //ROS_INFO("All in saturation");
	    for (int i=0; i<tauTN.size(); ++i)
      {
      	tauTN(i) = labust::math::coerce(tauTN(i), tnmin, tnmax);
      }
	    allocator.configure(B,txymax,txymin);
	    double scale = allocator.scaleII(tauXY,&tauXYNsc,&tauTXY);
	}

	tauI = tauTXY+tauTN;
	tauXYNsc = B*tauI;

	ROS_INFO("Available thrust: xy(%f), n(%f) - voltage = %f",txymax, tnmin, Ub);

	//Publish the scaled values if scaling occured
	auv_msgs::BodyForceReq t;
	t.wrench.force.x = tauXYNsc(0);
	t.wrench.force.y = tauXYNsc(1);
	t.wrench.force.z = 0;
	t.wrench.torque.x = 0;
	t.wrench.torque.y = 0;
	t.wrench.torque.z = tauXYNsc(2);
	t.header.stamp = ros::Time::now();

	if (satXY)
	{
	  t.disable_axis.x = 1;
	  if (tauXYN(0) > 0 ) t.windup.x = 1; else t.windup.x = -1;
	  t.disable_axis.y = 1;
	  if (tauXYN(1) > 0 ) t.windup.y = 1; else t.windup.y = -1;
	}

	if (satN)
	{
	  t.disable_axis.yaw = 1;
	  if (tauXYN(2) > 0 ) t.windup.yaw = 1; else t.windup.yaw = -1;
	}

	tauAch.publish(t);

	//Tau to Revs
	std_msgs::Float32MultiArray::Ptr pwm(new std_msgs::Float32MultiArray());
	pwm->data.resize(4);
	//Here we map the thrusts
	for (int i=0; i<pwm->data.size();++i)
	{
		pwm->data[i] = labust::vehicles::AffineThruster::getRevsD(tauI(i)/((Ub*Ub)/(Un*Un)),posDir[i],negDir[i]);
	}

	//The driver has 6 elements
	///pwm->data.push_back(0);
	//pwm->data.push_back(0);
	
	revs.publish(pwm);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"pladypos_node_v3");
	ros::NodeHandle nh,ph("~");

	labust::vehicles::PlaDyPosNode_v3 node;
	node.configure(nh,ph);

	ros::spin();

	return 0;
}
