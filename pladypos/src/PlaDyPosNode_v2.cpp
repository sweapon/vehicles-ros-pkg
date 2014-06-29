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
#include <labust/vehicles/PlaDyPosNode_v2.hpp>
#include <labust/vehicles/Allocation.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <string>
#include <sstream>

using namespace labust::vehicles;



PlaDyPosNode_v2::PlaDyPosNode_v2():
	Un(25.9),
	Ub(25.9),
	lastTau(ros::Time::now()),
	timeout(0.5),
	useWeighted(false)
{
	posDir[P0] = 1.376872219;
	negDir[P0] = 1.097241348;

	posDir[P1] = 1.908577382;
	negDir[P1] = 0.740035409;

	posDir[P2] = 2.088491416;
	negDir[P2] = 0.718808411;

	posDir[P3] = 1.922883428;
	negDir[P3] = 0.82880469;

	for (int i=0; i<4; ++i)
	{
		posDir[i] /=2*sqrt(2);
		negDir[i] /=2*sqrt(2);
	}
	float divisor = negDir[P2];
	for (int i=0; i<4; ++i)
	{
		posDir[i] /= divisor;
		negDir[i] /= divisor;
	}
}

PlaDyPosNode_v2::~PlaDyPosNode_v2()
{}

void PlaDyPosNode_v2::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Initialize subscribers and publishers
	tau = nh.subscribe("tauIn", 1, &PlaDyPosNode_v2::onTau,this);
	batteryVoltage = nh.subscribe("battery_voltage", 1, &PlaDyPosNode_v2::onBatteryVoltage,this);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	revs = nh.advertise<std_msgs::Int32MultiArray>("pwm_out",1);
	
	//Initialize max/min tau
	double maxThrust(1),minThrust(-1);
	//ph.param("maxThrust",maxThrust,maxThrust);
	//ph.param("minThrust",minThrust,minThrust);
	ph.param("useWeighted",useWeighted, useWeighted);

	//Initialize the allocation matrix
	float cp(cos(M_PI/4)),sp(sin(M_PI/4));
	B<<-cp,-cp,cp,cp,
	   sp,-sp,sp,-sp,
	    1,-1,-1,1;

	BinvIdeal = B.transpose()*(B*B.transpose()).inverse();

	//Scaling allocation only for XYN
	allocator.configure(B,maxThrust,minThrust);
}

void PlaDyPosNode_v2::onBatteryVoltage(const std_msgs::Float32::ConstPtr voltage)
{
	if (this->Ub != 0) this->Ub = voltage->data;
}

void PlaDyPosNode_v2::onTau(const auv_msgs::BodyForceReq::ConstPtr tau)
{
	lastTau = ros::Time::now();
	//Perform allocation
	Eigen::Vector3f tauXYN,tauXYNsc;
	Eigen::Vector4f tauI;
	tauXYN<<tau->wrench.force.x,tau->wrench.force.y,tau->wrench.torque.z;

	//Adapt maximum thrust to available battery voltage
	double maxThrust((Ub*Ub)/(Un*Un)),minThrust(-(Ub*Ub)/(Un*Un));
	double scale = 1;
	if (!useWeighted)
	{
		allocator.configure(B,maxThrust,minThrust);
		scale = allocator.scaleII(tauXYN,&tauXYNsc,&tauI);
	}
	else
	{
		for (int i=0; i<4; ++i)
		{
			posDirC[i] = (Ub*Ub)/(Un*Un) * posDir[i];
			negDirC[i] = (Ub*Ub)/(Un*Un) * negDir[i];
		}
		scale = weightedScaling(tauXYN,&tauXYNsc,&tauI);
	}

	ROS_INFO("Available thrust: (%f, %f) - voltage = %f",minThrust, maxThrust, Ub);

	//Publish the scaled values if scaling occured
	auv_msgs::BodyForceReq t;
	t.wrench.force.x = tauXYNsc(0);
	t.wrench.force.y = tauXYNsc(1);
	t.wrench.force.z = 0;
	t.wrench.torque.x = 0;
	t.wrench.torque.y = 0;
	t.wrench.torque.z = tauXYNsc(2);
	t.header.stamp = ros::Time::now();

	if (scale>1)
	{
	  t.disable_axis.x = 1;
	  if (tauXYN(0) > 0 ) t.windup.x = 1; else t.windup.x = -1;
	  t.disable_axis.y = 1;
	  if (tauXYN(1) > 0 ) t.windup.y = 1; else t.windup.y = -1;
	  t.disable_axis.yaw = 1;
	  if (tauXYN(2) > 0 ) t.windup.yaw = 1; else t.windup.yaw = -1;
	}

	tauAch.publish(t);

	//Tau to Revs
	std_msgs::Int32MultiArray::Ptr pwm(new std_msgs::Int32MultiArray());
	pwm->data.resize(4);
	//Here we map the thrusts
	for (int i=0; i<pwm->data.size();++i)
	{
		pwm->data[i] = 255*labust::vehicles::AffineThruster::getRevsD(tauI(i)/((Ub*Ub)/(Un*Un)),posDir[i],negDir[i]);
	}
	
	revs.publish(pwm);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"pladypos_node_v2");
	ros::NodeHandle nh,ph("~");

	labust::vehicles::PlaDyPosNode_v2 node;
	node.configure(nh,ph);

	ros::spin();

	return 0;
}
