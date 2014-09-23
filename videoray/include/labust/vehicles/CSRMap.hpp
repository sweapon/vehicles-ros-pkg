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
 *  Author: Gyula Nagy
 *  Created: 14.11.2013.
 *********************************************************************/
#ifndef CSRMAP_HPP_
#define CSRMAP_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/serialization/base_object.hpp>
#include <cstdint>

typedef uint8_t vec211c[211];
typedef float vec3f[3];

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(vehicles),
		BCMap,
		(float, rpm_target)
		(float, pwr_target)
		(float, rpm)
		(float, bus_v)
		(float, bus_i)
		(uint32_t, fault)
		(float, temp)
		(float, pwr_actual)
		(vec3f, rpm_PID)
		(vec211c, unmapped))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(vehicles),
		VRHeader,
		(uint16_t, sync)
		(uint8_t, network_id)
		(uint8_t, flags)
		(uint8_t, csraddr)
		(uint8_t, length))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(vehicles),
		TStdResponse,
		(uint8_t, deviceType)
		(float, rpm)
		(float, bus_v)
		(float, bus_i)
		(float, temp)
		(uint8_t, fault))

namespace labust
{
	namespace vehicles
	{
		struct Response
		{
			enum{tstdresponse=0x02};
		};

		struct Command
		{
			enum{custom = 0xF0};
			enum{setAll = 0xAA};
		};
	}
}


//CSRMAP_HPP_
#endif
