/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for Car Pose estimation based on wheel speed and IMU
**********************************************************************
* $Author::  $ fink  $Date:: 2016-01-12 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#ifndef _CAR_POSE_H_
#define _CAR_POSE_H_

#define OID_ADTF_CAR_POSE "adtf.user.car_pose"

#include "TSignalValue.h"
#include "TBoolSignalValue.h"
#include "TPoseStruct.h"
#include "TWheelData.h"

//#define DEBUG_SPEED_TO_FILE TRUE // dump to /tmp/car_speed.dat (plot with gnuplot)

//*************************************************************************************************
class CarPose: public adtf::cFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CAR_POSE, "Car_Pose", OBJCAT_DataFilter, "Car Pose", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin carSpeed_input;
	cInputPin imuYaw_input;
	cInputPin imuAccY_input;
	cInputPin steeringAngle_input;
	cInputPin distance_overall_input;
	cInputPin wheel_left_input;
	cInputPin wheel_right_input;

	cOutputPin pose_output;
	cOutputPin car_speed_output; // without direction! TODO add direction

	TSignalValue carSpeed;
	TSignalValue imuYAW;
	TSignalValue imuAccY;
	TSignalValue steeringAngle;
	TSignalValue distanceOverall;
	TWheelData wheelLeftIn;
	TWheelData wheelRightIn;

	TPoseStruct carPose;
	TSignalValue carSpeedOut;

public:
	CarPose(const tChar* __info);
	virtual ~CarPose();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:

	// properties
	tFloat32 m_f32YawCorrection;
	tFloat32 m_f32CarWheelBase;
	tBool m_bUseIMUYaw;
	tInt32 m_i32DistanceMode;
	tFloat32 m_f32wheelCircumference;
    tFloat32 m_f32FilterConstantfirstOrder; // the filter constant of first order
    tBool m_bEnableFiltering; // enables filtering of car speed

	// tSignalValue to save last values of the IMU yaw angle and wheel speed
	TSignalValue::Data last_IMU_yaw;
	TSignalValue::Data last_steering_angle;
	TSignalValue::Data last_wheel_speed;
	TSignalValue::Data last_imu_acc;
	TSignalValue::Data last_imu_carspeed;
	TSignalValue::Data wheel_speed_dir;
	TSignalValue::Data last_overall_distance;
	tFloat32 imu_offset;
	tFloat32 imu_acc_offset;
	tFloat32 deltat_sum;
	tFloat32 last_calc_carspeed;
	tUInt32 counter_posesamples;
	tBool car_speed_forwards;
	TWheelData::Data last_tach_left;
	TWheelData::Data last_tach_right;
	TWheelData::Data last_tach_overall;
	tBool initialized_tach_left;
	tBool initialized_tach_right;
	tBool initialized_tach_overall;
	tUInt32 counter_wheel_left_input;

	// save last car pose
	TPoseStruct::Data last_carPose;

	// initialized pose
	tBool initialized;

	// tick counter
	tUInt64 tick_counter;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

	// functions for processing and transmitting
	tResult ProcessIMUInput (IMediaSample*);
	tResult ProcessSteeringAngle(IMediaSample*);
	tResult ProcessWheelSpeedInput (IMediaSample*);
	tResult ProcessImuAccInput (IMediaSample*);
	tResult SaveWheelSpeedInput (IMediaSample*);
	tResult ProcessLeftStructInput (IMediaSample*);
	tResult ProcessRightStructInput (IMediaSample*);
	tBool GetWheelSpeedDirection();
	tFloat32 GetLastCarSpeed();
	tResult ProcessOverallDistanceInput (IMediaSample*);
	tResult ProcessTachInfo(tUInt32);
	tResult TransmitPose(tTimeStamp inputTime);
	TPoseStruct::Data GetCarPose();
	tFloat32 GetLastIMUYaw();
	tFloat32 GetLastSteeringAngle();
	tResult SetIMUYawOffset(tFloat32);
	tResult PollWheelTach();


	/*! if debug console output is enabled */
	tBool m_bDebugModeEnabled;
	tBool firstReceivedIMU;  // received first IMU
	tBool firstReceivedSteering; // received first steering

	/**** Critical sections ****/

	/*! the critical section of accessing the car pose data */
	cCriticalSection m_oCriticalSectionPoseAccess;

	/*! the critical section of accessing the car pose data */
	cCriticalSection m_oCriticalSectionWheelSpeedAccess;

	/*! the critical section of accessing the IMU data */
	cCriticalSection m_oCriticalSectionIMUDataAccess;

	/*! the critical section of accessing the steering angle */
	cCriticalSection m_oCriticalSectionSteeringDataAccess;

    /*! the critical section of transmitting car pose */
    cCriticalSection m_oCriticalSectionTransmit;

    /*! the critical section of accessing the wheel struct data */
    cCriticalSection m_oCriticalSectionWheelTachAccess;
};

//*************************************************************************************************
#endif // _CAR_POSE_H_
