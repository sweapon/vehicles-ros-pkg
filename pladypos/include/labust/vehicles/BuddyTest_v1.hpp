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
#ifndef BUDDYNODEV1_HPP_
#define BUDDYNODEV1_HPP_
#include <labust/vehicles/ScaleAllocation.hpp>

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
		class BuddyNode_v1
		{
			enum {P0=0,P1,P2,P3};
			const float Un;
		public:
			/**
			 * Main constructor.
			 */
			BuddyNode_v1();

			/**
			 * General destructor.
			 */
			~BuddyNode_v1();

			void configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		protected:

			///\todo Optimize the W and Binv calculation. There is a limited
			///number of combinations for W,Winv and Binv
			template <class Matrix, class Vector>
			double weightedScaling(const Eigen::VectorXf& virtualInput,
					Matrix* scaled, Vector* taui)
			{
				Eigen::VectorXf tauIdeal = BinvIdeal*virtualInput;
				Eigen::VectorXf tmax(tauIdeal.size());

				for (int i=0; i<tmax.size(); ++i)
				{
					tmax(i) = ((tauIdeal(i)>=0)?posDir[i]:fabs(negDir[i]));
				}

				tmax=(tmax/(tmax.minCoeff())).cwiseInverse();

				Eigen::MatrixXf W = tmax.asDiagonal();
				Eigen::MatrixXf Winv = W.inverse();
				Eigen::MatrixXf Binv = Winv*B.transpose()*(B*Winv*B.transpose()).inverse();
				std::cout<<Binv<<std::endl;
				Eigen::VectorXf tdes = Binv*virtualInput;

				double scale_max = 1;
				for (size_t i=0; i<tdes.rows();++i)
				{
					double scale = fabs((tdes(i)>0)?tdes(i)/posDirC[i]:tdes(i)/negDirC[i]);
					if (scale>scale_max) scale_max=scale;
				}
				tdes = tdes/scale_max;
				(*taui) = tdes;
				(*scaled) = B*tdes;

				std::cout<<"Input:"<<virtualInput<<std::endl;
				std::cout<<"Output:"<<*scaled<<std::endl;


				return scale_max;
			}


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
			boost::array<float,4> posDir, posDirC, negDir, negDirC;

			/**
			 * Allocation matrix and maximum force, torque.
			 */
			Eigen::Matrix<float, 3,4> B;
			/**
			 * Inverse matrix.
			 */
			Eigen::MatrixXf BinvIdeal;
			/**
			 * The scale allocator.
			 */
			ScaleAllocation allocator;
			/**
			 * The battery voltage for thrust mapping.
			 */
			float Ub;
			/**
			 * Use the weighted allocation.
			 */
			bool useWeighted;
			/**
			 * The maximum thrust cap.
 			 */
			double maxCap;
		};
	}
}
/* PLADYPOSNODEV2_HPP_ */
#endif
