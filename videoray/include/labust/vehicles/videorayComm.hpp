/*********************************************************************
 * videorayComm.hpp
 *
 *  Created on: Aug 20, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

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

#ifndef VIDEORAYCOMM_HPP_
#define VIDEORAYCOMM_HPP_

#include <boost/shared_ptr.hpp>
#include <labust/math/NumberManipulation.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The interface class for the VideoRay communication.
		 */
		class VRComms{

		public:
			/**
			 * The virtual destructor.
			 */
			virtual ~VRComms(){};
			/**
			 * Typedef for the thrust vector.
			 */
			typedef boost::array<int, 4> ThrustVec;
			/**
			 * Thruster enumerations.
			 */
			enum {port = 0, stbd, vert, light};
			/**
			 * The method encodes the given into the output buffer.
			 *
			 * \param thrust The given thrust values
			 * \param set The control bits.
			 */
			virtual bool encode(const ThrustVec& thrust) = 0;
			/**
			 * The method decodes the data in the buffer and returns the stateMap.
			 *
			 * \param state The state map to be filled.
			 */
			virtual bool decode(labust::vehicles::stateMapPtr state) = 0;

			/**
			 * The input buffer.
			 */
			std::vector<unsigned char> inputBuffer;
			/**
			 * The output buffer.
			 */
			std::vector<unsigned char> outputBuffer;
		};

		typedef boost::shared_ptr<VRComms> VRCommsPtr;


		/*************************************************************
		 * The class implements the serial communication with the VideoRay Pro3 vehicle.
		 * Physical media: RS232, baud rate 9600, 8 bit, 1 stop, no parity.
		 *
		 * We define the following protocol (decimal numbers):
		 *  Information in the 8 Bytes the PC Sends
		 *        1) 35 (for all VideoRay models)
		 *        2) 49 (for Pro III)
		 *        3) Current for the port thruster, minimum 0, maximum 220;
		 *        4) Current for the starboard thruster, minimum 0, maximum 220;
		 *        5) Current for the vertical thruster, minimum 0, maximum 220;
		 *        6) Current for the lights, minimum, maximum 200;
		 *        7) Bit level Controls for the manipulators and auto depth,
		 *        8) Bit level controls of camera tilt and focus, the direction of the thrusters,
		 *  Information of the 7 Bytes VideoRay Sends
		 *  The first 3 bytes of the 7 bytes contain an identifier then compass low byte,
		 *  compass high byte, pressure low byte and pressure high byte.
		 *        1) 40 (All VideoRay models)
		 *        2) 49 (All VideoRay Pro III)
		 *        3) 02 (data type for future use)
		 *        4) Low byte of Orientation
		 *        5) High byte of Orientation
		 *        6) Low byte of Depth
		 *        7) High byte of Depth
		 *
		 * The relation between low byte, high byte and the real value is:
		 * Real value = Low byte + 256 x High Byte, for instance:
		 * Orientation = Low byte of Orientation + 256 x High Byte of Orientation (0-359)
		 * Depth = Low byte of Depth + 256 x High Byte of Depth (0-1023)
		 *
		 * When Orientation is calculated, the following conversion is needed for the real orientation:
		 *
		 * Real Orientation = 360 - Orientation;  // mirror the image of the orientation
		 * if (Real Orientation < 90) Real Orientation = 270 + Real Orientation;
		 * // shift 90 degrees counterclockwise
		 * else Real Orientation = Real Orientation - 90;
		 *
		 ************************************************************/

		class VRSerialComms : public virtual VRComms{

			enum {header = 35, id = 49, inLen = 7, outLen = 8};

		public:
			/**
			 * Typedef for the bit control set.
			 */
			typedef std::bitset<16> ControlSet;
			/**
			 * Generic constructor.
			 */
			VRSerialComms();
			/**
			 * Generic destructor.
			 */
			~VRSerialComms();

			/**
			 * \override labust::vehicles::VRComms::encode
			 */
			bool encode(const ThrustVec& thrust);
			/**
			 * \override labust::vehicles::VRComms::decode
			 */
			bool decode(labust::vehicles::stateMapPtr state);

		private:
			/**
			 * The control bits set.
			 */
			ControlSet set;
		};


		VRSerialComms::VRSerialComms()
		{
			//Set the buffer sizes.
			outputBuffer.resize(outLen,0);
			inputBuffer.resize(inLen,0);

			//Set the constant values.
			outputBuffer[0] = header;
			outputBuffer[1] = id;
		}
		VRSerialComms::~VRSerialComms(){}

		bool VRSerialComms::encode(const ThrustVec& thrust)
		{
			enum {port_byte = 2, stbd_byte, vert_byte, light_byte, control1, control2};
			enum {manipulator = 0, manipulator_enable, camera_rear = 6};
			enum {tilt = 8, tilt_enable, focus, focus_enable, port_direction, stbd_direction, vert_direction};

		  outputBuffer[port_byte] = std::abs(thrust[port]);
		  outputBuffer[stbd_byte] = std::abs(thrust[stbd]);
		  outputBuffer[vert_byte] = std::abs(thrust[vert]);
		  outputBuffer[light_byte] = std::abs(thrust[light]);
		  set[port_direction] = !(thrust[port] > 0);
		  set[stbd_direction] = thrust[stbd] > 0;
		  set[vert_direction] = thrust[vert] > 0;

		  //std::cout<<"Thrust:"<<thrust[port]<<","<<thrust[stbd]<<std::endl;

		  unsigned short flags = static_cast<unsigned short>(set.to_ulong());
		  outputBuffer[control1] = flags%256;
		  outputBuffer[control2] = flags/256;

		  return true;
		}

		bool VRSerialComms::decode(labust::vehicles::stateMapPtr state)
		{
			using namespace labust::vehicles::state;
			//Check if the data header is ok.
			enum {yaw_byte = 3, depth_byte = 5};

			//Get heading and adjust
		  short int value = 360 - (inputBuffer[yaw_byte] + 256*inputBuffer[yaw_byte+1]);
		  if (value < 90) (*state)[heading]= value + 270; else (*state)[heading]= value - 90;
		  (*state)[yaw] = labust::math::wrapRad((*state)[heading]*M_PI/180);

		  //Get pressure and calculate depth.
		  (*state)[depthPressure] = inputBuffer[depth_byte] + 256*inputBuffer[depth_byte+1];

		  return true;
		}
	}
}


#endif /* VIDEORAYCOMM_HPP_ */
