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
#ifndef PLADYPOSNODEV2_HPP_
#define PLADYPOSNODEV2_HPP_
#include <labust/vehicles/ScaleAllocation.hpp>
#include <pladypos/ThrusterMappingConfig.h>

#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <boost/array.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The PlaDyPos V2 ROS node for allocation, control and monitoring of the surface platform.
		 *
		 * \todo When the stable version is reached spin off a general templated or
		 * dynamically loaded vehicle driver
		 */
		class PlaDyPosNode_v2
		{
			enum {P0=0,P1,P2,P3};
			const float Un;
		public:
			/**
			 * Main constructor.
			 */
			PlaDyPosNode_v2();

			/**
			 * General destructor.
			 */
			~PlaDyPosNode_v2();

			void configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		protected:
			/**
			 * The desired force and torque subscriber.
			 */
			ros::Subscriber tau, batteryVoltage;
			/**
			 * The achieved force and torque publisher.
			 */
			ros::Publisher tauAch, revs;

			/**
			 * Handles the arrived force and torque requests.
			 */
			void onTau(const auv_msgs::BodyForceReq::ConstPtr tau);
			/**
			 * Handles the arrived force and torque requests.
			 */
			void onBatteryVoltage(const std_msgs::Float32::ConstPtr tau);;

			/**
			 * The last TAU.
			 */
			ros::Time lastTau;
			/**
			 * The timeout.
			 */
			double timeout;

			/**
			 * Thruster parameters.
			 */
			boost::array<float,4> posDir, negDir;

			/**
			 * Allocation matrix and maximum force, torque.
			 */
			Eigen::Matrix<float, 3,4> B;
			/**
			 * The scale allocator.
			 */
			ScaleAllocation allocator;
			/**
			 * The battery voltage for thrust mapping.
			 */
			float Ub;
		};
	}
}
/* PLADYPOSNODEV2_HPP_ */
#endif
