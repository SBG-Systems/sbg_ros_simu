// File header
#include "sbg_simu.h"

// Standard headers
#include <iomanip>
#include <fstream>
#include <ctime>

// Boost headers
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

// SbgECom headers
#include <version/sbgVersion.h>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;
using sbg::SbgSimu;

//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

SbgSimu::SbgSimu(ros::NodeHandle& ref_node_handle):
m_ref_node_(ref_node_handle)
{
  connect();
}

SbgSimu::~SbgSimu(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComClose(&m_com_handle_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to close the SBG communication handle - %s.", sbgErrorCodeToString(error_code));
  }

  error_code = sbgInterfaceSerialDestroy(&m_sbg_interface_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("SBG_SIMU - Unable to close the communication interface.");
  }
}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

SbgErrorCode SbgSimu::sendDeviceInfo(SbgEComHandle *pHandle)
{
	SbgErrorCode      error_code;
	SbgStreamBuffer   streamBuf;
	uint8_t           buf[sizeof(SbgEComDeviceInfo)];

	error_code = sbgStreamBufferInitForWrite(&streamBuf, &buf, sizeof(SbgEComDeviceInfo));

	if (error_code == SBG_NO_ERROR)
	{
		SbgEComDeviceInfo deviceInfo;

		error_code = sbgStreamBufferWriteBuffer(&streamBuf, "sbg_simu", SBG_ECOM_INFO_PRODUCT_CODE_LENGTH);

		if (error_code == SBG_NO_ERROR)
		{
			deviceInfo.serialNumber		= 1007;
			deviceInfo.calibationRev	= 1;
			deviceInfo.calibrationYear	= 2021;
			deviceInfo.calibrationMonth	= 10;
			deviceInfo.calibrationDay	= 1;
			deviceInfo.hardwareRev		= 1;
			deviceInfo.firmwareRev		= 1;

			sbgStreamBufferWriteUint32LE(&streamBuf, deviceInfo.serialNumber);
			sbgStreamBufferWriteUint32LE(&streamBuf, deviceInfo.calibationRev);
			sbgStreamBufferWriteUint16LE(&streamBuf, deviceInfo.calibrationYear);
			sbgStreamBufferWriteUint8LE (&streamBuf, deviceInfo.calibrationMonth);
			sbgStreamBufferWriteUint8LE (&streamBuf, deviceInfo.calibrationDay);
			sbgStreamBufferWriteUint32LE(&streamBuf, deviceInfo.hardwareRev);
			sbgStreamBufferWriteUint32LE(&streamBuf, deviceInfo.firmwareRev);

			error_code = sbgStreamBufferGetLastError(&streamBuf);

			if (error_code == SBG_NO_ERROR)
			{
				size_t		receivedSize;
				uint8_t		receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
				uint32_t	time_out_ms = 10*1000;

				/*
				 * Wait for Driver command.
				 */
				error_code = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, receivedBuffer, &receivedSize, sizeof(receivedBuffer), time_out_ms);

				if (error_code == SBG_NO_ERROR)
				{
					error_code = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, buf, SBG_ARRAY_SIZE(buf));
				}
			}
		}
	}

	return error_code;
}

void SbgSimu::connect(void)
{
  SbgErrorCode error_code;

  sbgIpAddress ip = sbgNetworkIpFromString("127.0.0.1");
  uint32_t     portIn  = 6666;
  uint32_t     portOut = 6667;

  //
  // Initialize the communication interface.
  //
  error_code = sbgInterfaceUdpCreate(&m_sbg_interface_, ip, portIn, portOut);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_SIMU - [Init] Unable to initialize the interface - " + std::string(sbgErrorCodeToString(error_code)));
  }

  //
  // Initialize the sbgECom protocol.
  //
  error_code = sbgEComInit(&m_com_handle_, &m_sbg_interface_);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_SIMU - [Init] Unable to initialize the SbgECom protocol - " + std::string(sbgErrorCodeToString(error_code)));
  }

  error_code = sendDeviceInfo(&m_com_handle_);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_SIMU - [Init] Unable to write device info - " + std::string(sbgErrorCodeToString(error_code)));
  }

  ROS_WARN("SBG_SIMU - UDP: %d.%d.%d.%d in:%d out:%u", sbgIpAddrGetA(ip), sbgIpAddrGetB(ip), sbgIpAddrGetC(ip), sbgIpAddrGetD(ip), portIn, portOut);

}

SbgErrorCode SbgSimu::navDataToStream(SbgLogEkfNavData *pNavData, SbgStreamBuffer *pOutputStream)
{
	assert(pNavData);

	sbgStreamBufferWriteUint32LE(pOutputStream, pNavData->timeStamp);

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pNavData->velocity); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pNavData->velocity[i]);
	}

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pNavData->velocityStdDev); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pNavData->velocityStdDev[i]);
	}

	sbgStreamBufferWriteDoubleLE(pOutputStream, pNavData->position[0]);
	sbgStreamBufferWriteDoubleLE(pOutputStream, pNavData->position[1]);
	sbgStreamBufferWriteDoubleLE(pOutputStream, pNavData->position[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pNavData->undulation);

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pNavData->positionStdDev); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pNavData->positionStdDev[i]);
	}

	sbgStreamBufferWriteUint32LE(pOutputStream, pNavData->status);

	return sbgStreamBufferGetLastError(pOutputStream);
}

SbgErrorCode SbgSimu::imuDataToStream(SbgLogImuData *pImuData, SbgStreamBuffer *pOutputStream)
{
	assert(pImuData);

	sbgStreamBufferWriteUint32LE(pOutputStream, pImuData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pImuData->status);

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pImuData->accelerometers); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pImuData->accelerometers[i]);
	}

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pImuData->gyroscopes); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pImuData->gyroscopes[i]);
	}

	sbgStreamBufferWriteFloatLE(pOutputStream, pImuData->temperature);

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pImuData->deltaVelocity); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pImuData->deltaVelocity[i]);
	}

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pImuData->deltaAngle); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pImuData->deltaAngle[i]);
	}

	return sbgStreamBufferGetLastError(pOutputStream);
}

SbgErrorCode SbgSimu::quatDataToStream(SbgLogEkfQuatData *pQuatData, SbgStreamBuffer *pOutputStream)
{
	assert(pQuatData);

	sbgStreamBufferWriteUint32LE(pOutputStream, pQuatData->timeStamp);

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pQuatData->quaternion); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pQuatData->quaternion[i]);
	}

	for (unsigned i = 0; i < SBG_ARRAY_SIZE(pQuatData->eulerStdDev); i++)
	{
		sbgStreamBufferWriteFloatLE(pOutputStream, pQuatData->eulerStdDev[i]);
	}

	sbgStreamBufferWriteUint32LE(pOutputStream, pQuatData->status);

	return sbgStreamBufferGetLastError(pOutputStream);
}


//---------------------------------------------------------------------//
//- Public  methods                                                   -//
//---------------------------------------------------------------------//

// http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
void SbgSimu::imuCallback(const sensor_msgs::Imu &msg)
{
	SbgErrorCode		error_code;
	SbgLogImuData		imuData;
	SbgLogEkfQuatData	quatData;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	memset(&imuData, 0x0, sizeof(imuData));
	memset(&quatData, 0x0, sizeof(quatData));

	this->m_last_imu_time_ = ros::Time::now();
	imuData.timeStamp = this->m_last_imu_time_.toNSec() / 1000;

	imuData.status = SBG_ECOM_IMU_COM_OK | SBG_ECOM_IMU_STATUS_BIT | SBG_ECOM_IMU_ACCEL_X_BIT | SBG_ECOM_IMU_ACCEL_Y_BIT | SBG_ECOM_IMU_ACCEL_Z_BIT | SBG_ECOM_IMU_GYRO_X_BIT | SBG_ECOM_IMU_GYRO_Y_BIT | SBG_ECOM_IMU_GYRO_Z_BIT | SBG_ECOM_IMU_ACCELS_IN_RANGE | SBG_ECOM_IMU_GYROS_IN_RANGE;
	imuData.temperature = 20;

	/*
	 * Convert from ENU (/imu) to NED (SBG device).
	 */
    imuData.accelerometers[0] =  msg.linear_acceleration.x;
    imuData.accelerometers[1] = -msg.linear_acceleration.y;
    imuData.accelerometers[2] = -msg.linear_acceleration.z;

//    imuData.gyroscopes[0] =  msg.;
//    imuData.gyroscopes[1] = -msg.;
//    imuData.gyroscopes[2] = -msg.;
//
//    imuData.deltaVelocity[0] =  msg.;
//    imuData.deltaVelocity[1] = -msg.;
//    imuData.deltaVelocity[2] = -msg.;
//
//    imuData.deltaAngle[0] =  msg.;
//    imuData.deltaAngle[1] = -msg.;
//    imuData.deltaAngle[2] = -msg.;

	quatData.timeStamp = this->m_last_imu_time_.toNSec() / 1000;
	quatData.status = SBG_ECOM_SOL_MODE_NAV_POSITION | SBG_ECOM_SOL_ATTITUDE_VALID;

	quatData.quaternion[0] = msg.orientation.w;
	quatData.quaternion[1] = msg.orientation.x;
	quatData.quaternion[2] = msg.orientation.y;
	quatData.quaternion[3] = msg.orientation.z;
//	quatData.eulerStdDev[3];

	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
	error_code = imuDataToStream(&imuData, &outputStream);

	if (error_code == SBG_NO_ERROR)
	{
		error_code = sbgEComProtocolSend(&this->m_com_handle_.protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, &imuData, sizeof(imuData));

		if (error_code == SBG_NO_ERROR)
		{
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
			error_code = quatDataToStream(&quatData, &outputStream);

			if (error_code == SBG_NO_ERROR)
			{
				error_code = sbgEComProtocolSend(&this->m_com_handle_.protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &quatData, sizeof(quatData));

				if (error_code != SBG_NO_ERROR)
				{
					ROS_WARN("SBG_SIMU - EKF send error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
				}
			}
			else
			{
				ROS_WARN("SBG_SIMU - EKF stream error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
			}
		}
		else
		{
			ROS_WARN("SBG_SIMU - IMU send error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
		}
	}
	else
	{
		ROS_WARN("SBG_SIMU - IMU stream error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
	}
}

// http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
void SbgSimu::gpsCallback(const sensor_msgs::NavSatFix &msg)
{
	SbgErrorCode		error_code;
	SbgLogEkfNavData	navData;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	memset(&navData, 0x0, sizeof(navData));

	navData.timeStamp = this->m_last_imu_time_.toNSec() / 1000;

//	navData.velocity[3];			/*!< North, East, Down velocity in m.s^-1. */
//	navData.velocityStdDev[3];		/*!< North, East, Down velocity 1 sigma standard deviation in m.s^-1. */
	navData.position[0] = msg.latitude;
	navData.position[1] = msg.longitude;
	navData.position[2] = msg.altitude;
//	navData.undulation;				/*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
//	navData.positionStdDev[3];		/*!< Latitude, longitude and altitude 1 sigma standard deviation in meters. */
	navData.status = SBG_ECOM_SOL_MODE_NAV_POSITION | SBG_ECOM_SOL_POSITION_VALID;

	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
	error_code = navDataToStream(&navData, &outputStream);

	if (error_code == SBG_NO_ERROR)
	{
		error_code = sbgEComProtocolSend(&this->m_com_handle_.protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		if (error_code != SBG_NO_ERROR)
		{
			ROS_WARN("SBG_SIMU - GPS send error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
		}
	}
	else
	{
		ROS_WARN("SBG_SIMU - streambuffer error 0x%x (%s)", error_code, sbgErrorCodeToString(error_code));
	}
}
