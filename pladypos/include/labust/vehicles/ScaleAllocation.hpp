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
*  Author: Dula Nad
*  Created: 06.03.2013.
*********************************************************************/
#ifndef SCALEALLOCATION_HPP_
#define SCALEALLOCATION_HPP_

#include <Eigen/Dense>

namespace labust
{
	namespace vehicles
	{
		/**
		 * Performs the allocation for the supplied matrix.
		 */
		class ScaleAllocation
		{
		public:
			ScaleAllocation(){};
			/**
			 * Main constructor. Takes the allocation matrix.
			 */
			template <class Matrix>
			ScaleAllocation(const Matrix& B, double tmax, double tmin):
				B(B),
				tmax(tmax),
				tmin(tmin)
			{
				Binv = B.transpose()*(B*B.transpose()).inverse();
			}

			template <class Matrix>
			void configure(const Matrix& B, double tmax, double tmin)
			{
				this->B = B;
				this->tmax = tmax;
				this->tmin = tmin;
				Binv = B.transpose()*(B*B.transpose()).inverse();
			}

			template <class Matrix>
			double scale(const Eigen::VectorXf& virtualInput, Matrix* scaled)
			{
				Eigen::VectorXf tdes = Binv*virtualInput;

				double scale_max = 1;
				for (size_t i=0; i<tdes.rows();++i)
				{
					double scale = fabs((tdes(i)>0)?tdes(i)/tmax:tdes(i)/tmin);
					if (scale>scale_max) scale_max=scale;
				}
				tdes = tdes/scale_max;
				(*scaled) = B*tdes;

				return scale_max;
			}

			template <class Matrix, class Vector>
			double scaleII(const Eigen::VectorXf& virtualInput,
					Matrix* scaled, Vector* taui)
			{
				Eigen::VectorXf tdes = Binv*virtualInput;

				double scale_max = 1;
				for (size_t i=0; i<tdes.rows();++i)
				{
					double scale = fabs((tdes(i)>0)?tdes(i)/tmax:tdes(i)/tmin);
					if (scale>scale_max) scale_max=scale;
				}
				tdes = tdes/scale_max;
				(*taui) = tdes;
				(*scaled) = B*tdes;

				return scale_max;
			}

		private:
			/**
			 * The allocation matrix and inverse.
			 */
			Eigen::MatrixXf B,Binv;
			/**
			 * Maximum and minimum thrust.
			 */
			double tmax, tmin;
		};
	}
}


#endif /* SCALEALLOCATION_HPP_ */
