/*!
*	\file         sbg_simu.h
*	\author       SBG Systems
*	\date         September 30, 2020
*
*	\brief        SBG device simulation.
*
*	\section CodeCopyright Copyright Notice
*	MIT License
*
*	Copyright (c) 2020 SBG Systems
*
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/

#ifndef SBG_ROS_SIMU_H
#define SBG_ROS_SIMU_H

// Standard headers
#include <iostream>
#include <map>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// SbgECom headers
#include <sbgEComLib.h>

// Project headers

namespace sbg
{
class SbgSimu
{
private:

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  SbgEComHandle           m_com_handle_;
  SbgInterface            m_sbg_interface_;
  ros::NodeHandle&        m_ref_node_;
  ros::Time               m_last_imu_time_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  SbgErrorCode sendDeviceInfo(SbgEComHandle *pHandle);

  void connect(void);

  SbgErrorCode navDataToStream(SbgLogEkfNavData *pNavData, SbgStreamBuffer *pOutputStream);
  SbgErrorCode imuDataToStream(SbgLogImuData *pImuData, SbgStreamBuffer *pOutputStream);
  SbgErrorCode quatDataToStream(SbgLogEkfQuatData *pQuatData, SbgStreamBuffer *pOutputStream);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   *
   * \param[in] ref_node_handle   ROS NodeHandle.
   */
  SbgSimu(ros::NodeHandle& ref_node_handle);

  /*!
   * Default destructor.
   */
  ~SbgSimu(void);

  /*!
   * IMU callback.
   */
	void imuCallback(const sensor_msgs::Imu &msg);

  /*!
   * GPS callback.
   */
	void gpsCallback(const sensor_msgs::NavSatFix &msg);
};
}

#endif // SBG_ROS_SIMU_H
