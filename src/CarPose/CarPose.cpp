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

#include <math.h>
#include "stdafx.h"
#include "CarPose.h"

#include <iostream>
#include <fstream>

#define CW_SLOT_COUNT 60.f // ticks

/// Create filter shell
ADTF_FILTER_PLUGIN("Car_Pose", OID_ADTF_CAR_POSE, CarPose);

CarPose::CarPose(const tChar* __info) :
		cFilter(__info) {

	// initialization of memory sizes of media samples to create
	m_f32YawCorrection = 1.0; // yaw correction
	m_f32CarWheelBase = 0.36; //wheelbase: move pose frame (0.6 is working better)
	m_bUseIMUYaw = tTrue;
	m_bDebugModeEnabled = tFalse;
	m_i32DistanceMode = 3; // use wheel tach inputs
	m_f32wheelCircumference = 0.34;
	m_f32FilterConstantfirstOrder = 0.07;
	m_bEnableFiltering = tTrue;

	SetPropertyFloat("IMU Yaw correction factor", m_f32YawCorrection);
	SetPropertyFloat("IMU Yaw correction factor" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("IMU Yaw correction factor" NSSUBPROP_DESCRIPTION,
				"Correction factor for IMU Yaw measurement");

	SetPropertyFloat("Car Wheelbase", m_f32CarWheelBase);
	SetPropertyFloat("Car Wheelbase" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Car Wheelbase" NSSUBPROP_DESCRIPTION,
				"Car Wheelbase used for pose calculation based on steering angle");

	SetPropertyBool("Use IMU Yaw angle", m_bUseIMUYaw);
	SetPropertyBool("Use IMU Yaw angle" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Use IMU Yaw angle" NSSUBPROP_DESCRIPTION,
				"True: Use IMU Yaw angle for theta calculation, false: use car steering angle");

	SetPropertyBool("Enable Debug", m_bDebugModeEnabled);
	SetPropertyBool("Enable Debug" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Debug" NSSUBPROP_DESCRIPTION,
				"Enables Debug messages");

    SetPropertyFloat("Wheel circumference",0.34);
    SetPropertyFloat("Wheel circumference" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Wheel circumference" NSSUBPROP_DESCRIPTION, "Set the wheel circumference in meter here");

    SetPropertyInt("Distance Source", 3);
    SetPropertyStr("Distance Source" NSSUBPROP_VALUELISTNOEDIT, "1@OVERALL_DIST|2@IMU_ACC|3@TACH");

	SetPropertyFloat("Filter constant first order",0.07);
	SetPropertyFloat("Filter constant first order" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Filter constant first order" NSSUBPROP_DESCRIPTION, "Set the filter constant for first order here");

	SetPropertyBool("Filtering enabled",tTrue);
	SetPropertyFloat("Filtering enabled" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Filtering enabled" NSSUBPROP_DESCRIPTION, "Enables or disables the low pass filtering of speed result");

	#ifdef DEBUG_SPEED_TO_FILE
		// log file for complete car pose
		LOG_WARNING("CarPose: Extended debug mode for car speed enabled, writing to /tmp/car_speed.dat");
		fstream file_speed;
		file_speed.open("/tmp/car_speed.dat", ios::out | ios::trunc);
		file_speed << "# time in s; speed wheel speed converter; speed IMU Acc Y\n";
		file_speed.close();
		counter_posesamples = 0;
	#endif
}

CarPose::~CarPose() {

}

tResult CarPose::CreateInputPins(__exception)
{

	RETURN_IF_FAILED(carSpeed_input.Create("set_car_speed", carSpeed.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&carSpeed_input));

	RETURN_IF_FAILED(imuYaw_input.Create("Imu_Yaw", imuYAW.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&imuYaw_input));

	RETURN_IF_FAILED(imuAccY_input.Create("Imu_Acc_Y", imuYAW.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&imuAccY_input));

	RETURN_IF_FAILED(steeringAngle_input.Create("steering_angle", steeringAngle.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&steeringAngle_input));

	RETURN_IF_FAILED(distance_overall_input.Create("distance_overall", distanceOverall.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&distance_overall_input));

	RETURN_IF_FAILED(wheel_left_input.Create("wheel_left", wheelLeftIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&wheel_left_input));

	RETURN_IF_FAILED(wheel_right_input.Create("wheel_right", wheelLeftIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&wheel_right_input));

	RETURN_NOERROR;
}

tResult CarPose::CreateOutputPins(__exception)
{
	// create output for car pose
	RETURN_IF_FAILED(pose_output.Create("car_pose", carPose.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&pose_output));

	RETURN_IF_FAILED(car_speed_output.Create("car_speed", carSpeedOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&car_speed_output));

	RETURN_NOERROR;
}

tResult CarPose::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{

		// inputs
		RETURN_IF_FAILED(carSpeed.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(imuYAW.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(imuAccY.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(steeringAngle.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(distanceOverall.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(wheelLeftIn.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(wheelRightIn.StageFirst(__exception_ptr));

		//output
		RETURN_IF_FAILED(carPose.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(carSpeedOut.StageFirst(__exception_ptr));

		// create and register the input and output pins
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface
		m_f32YawCorrection = tFloat32(GetPropertyFloat("IMU Yaw correction factor"));
		m_f32CarWheelBase = tFloat32(GetPropertyFloat("Car Wheelbase"));
		m_bUseIMUYaw = tBool(GetPropertyFloat("Use IMU Yaw angle"));
		m_bDebugModeEnabled = tBool(GetPropertyBool("Enable Debug"));
		m_i32DistanceMode = GetPropertyInt("Distance Source");
		m_f32wheelCircumference = static_cast<tFloat32>(GetPropertyFloat("wheel circumference"));
		m_bEnableFiltering = static_cast<tBool>(GetPropertyBool("filtering enabled"));
		m_f32FilterConstantfirstOrder = static_cast<tFloat32>(GetPropertyFloat("filter constant first order"));

	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.

		// inputs
		RETURN_IF_FAILED(carSpeed.StageGraphReady());
		RETURN_IF_FAILED(imuYAW.StageGraphReady());
		RETURN_IF_FAILED(imuAccY.StageGraphReady());
		RETURN_IF_FAILED(steeringAngle.StageGraphReady());
		RETURN_IF_FAILED(distanceOverall.StageGraphReady());
		RETURN_IF_FAILED(wheelLeftIn.StageGraphReady());
		RETURN_IF_FAILED(wheelRightIn.StageGraphReady());

		// outputs
		RETURN_IF_FAILED(carPose.StageGraphReady());
		RETURN_IF_FAILED(carSpeedOut.StageGraphReady());

		// car pose init
		imu_offset = 0;		// imu offset
		imu_acc_offset = 0; // imu acc offset
		deltat_sum = 0; // time running in s
		initialized = tFalse; // initialized
		firstReceivedIMU = tFalse; // start when first IMU yaw sample received
		firstReceivedSteering = tFalse; // start when first steering sample received
		car_speed_forwards = tTrue; // get car direction
		initialized_tach_left = tFalse;
		initialized_tach_right = tFalse;
		initialized_tach_overall = tFalse;
		tick_counter = 0; // global tick counter
		counter_wheel_left_input = 0; // counts media sample input
		last_calc_carspeed = 0; // save last calculated car speed

		#ifdef DEBUG_SPEED_TO_FILE
			counter_posesamples = 0;
		#endif

	}

	RETURN_NOERROR;
}

tResult CarPose::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult CarPose::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult CarPose::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult CarPose::ProcessIMUInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionIMUDataAccess);

	// save last IMU yaw angle
	imuYAW.Read(pMediaSample, &last_IMU_yaw);

	if(!firstReceivedIMU) {
		firstReceivedIMU = tTrue;	
	}

	RETURN_NOERROR;
}

tResult CarPose::ProcessSteeringAngle(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionSteeringDataAccess);

	// save last steering angle
	steeringAngle.Read(pMediaSample, &last_steering_angle);

	if(!firstReceivedSteering) {
		firstReceivedSteering = tTrue;	
	}

	RETURN_NOERROR;
}

tFloat32 CarPose::GetLastSteeringAngle(){
	__synchronized_obj(m_oCriticalSectionSteeringDataAccess);
	return -(last_steering_angle.f32_value-90.0);
}

tFloat32 CarPose::GetLastIMUYaw(){
	__synchronized_obj(m_oCriticalSectionIMUDataAccess);
	return last_IMU_yaw.f32_value - imu_offset;
}

tResult CarPose::SetIMUYawOffset(tFloat32 offset){
	__synchronized_obj(m_oCriticalSectionIMUDataAccess);
	imu_offset = offset;
	RETURN_NOERROR;
}

tResult CarPose::ProcessWheelSpeedInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionPoseAccess);

	// save last car speed 
	TSignalValue::Data wheel_speed;
	carSpeed.Read(pMediaSample, &wheel_speed);

	// first run: save values
	if(!initialized) {
		if(firstReceivedIMU) {
			last_wheel_speed = wheel_speed;
			initialized = tTrue;
			SetIMUYawOffset(GetLastIMUYaw());
			if (m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("CarPose: Initialized with IMU yaw: %f, wheel speed: %f", GetLastIMUYaw(), last_wheel_speed.f32_value ));
			}
		} else {
			if (m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("CarPose: Waiting for first IMU sample to initialize"));
			}
		}
		RETURN_NOERROR;
	}

	// calculate pose based on IMU yaw angle and wheel speed

	tFloat32 car_speed_y = wheel_speed.f32_value;
	tFloat32 car_speed_x = 0.0;
	tFloat32 last_yaw = ( (GetLastIMUYaw()) / 180.0) * ( 3.1415926 ) * m_f32YawCorrection;
	tFloat32 dt = ( wheel_speed.ui32_arduinoTimestamp - last_wheel_speed.ui32_arduinoTimestamp ) / 1E6;

	// calculate relative distances (TODO remove speed_y)
    tFloat32 delta_x = ( car_speed_x * cos(last_yaw) - car_speed_y * sin(last_yaw) ) * dt;
    tFloat32 delta_y = ( car_speed_x * sin(last_yaw) + car_speed_y * cos(last_yaw) ) * dt;

    last_carPose.f32_x += delta_x;
    last_carPose.f32_y += delta_y;
    last_carPose.f32_yaw = last_yaw;
    last_carPose.ui32_arduinoTimestamp = wheel_speed.ui32_arduinoTimestamp;

    // LOG_WARNING(cString::Format("inputAngle: data value of car pose x, y: %f, %f",delta_x, delta_y));

    // save last wheel speed with arduino timestamp
	last_wheel_speed = wheel_speed;

	RETURN_NOERROR;
}

tResult CarPose::ProcessImuAccInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionPoseAccess);

	// save last car speed
	TSignalValue::Data imu_acc_y;
	imuAccY.Read(pMediaSample, &imu_acc_y);

	// first run: save values
	if(!initialized) {
		if(firstReceivedIMU) {
			last_imu_acc = imu_acc_y;
			initialized = tTrue;
			SetIMUYawOffset(GetLastIMUYaw());
			last_carPose.f32_x = 0;
			last_carPose.f32_y = 0;
			last_carPose.f32_yaw = 0;
			imu_acc_offset = imu_acc_y.f32_value;
			deltat_sum = 0;
			if (m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("CarPose: Initialized [IMU Acc mode] IMU Yaw: %f, IMU Acc: %f", GetLastIMUYaw(), imu_acc_y.f32_value ));
			}
		} else {
			if (m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("CarPose [IMU Acc mode]: Waiting for first IMU Yaw sample to initialize"));
			}
		}
		RETURN_NOERROR;
	}

	// calculate pose based on IMU yaw angle and IMU Acc

	tFloat32 dt = ( imu_acc_y.ui32_arduinoTimestamp - last_imu_acc.ui32_arduinoTimestamp ) / 1E6;
	deltat_sum += dt;
	tFloat32 imu_acc = imu_acc_y.f32_value-imu_acc_offset;
	last_imu_carspeed.f32_value += (imu_acc * dt) + deltat_sum * 0.00000; // 0.00002

	tFloat32 last_yaw = ( (GetLastIMUYaw()) / 180.0) * ( 3.1415926 ) * m_f32YawCorrection;

	// calculate relative distances
    tFloat32 delta_x = last_imu_carspeed.f32_value * cos(last_yaw) * dt;
    tFloat32 delta_y = last_imu_carspeed.f32_value * sin(last_yaw) * dt;

    last_carPose.f32_x += delta_x;
    last_carPose.f32_y += delta_y;
    last_carPose.f32_yaw = last_yaw;
    last_carPose.ui32_arduinoTimestamp = imu_acc_y.ui32_arduinoTimestamp;

	// log car speed to file
	#ifdef DEBUG_SPEED_TO_FILE
		if ( counter_posesamples % 2 == 0) { // 1 out of 2
			fstream file_speed;
			file_speed.open("/tmp/car_speed.dat", ios::out | ios::app);
			file_speed << deltat_sum << " " << GetLastCarSpeed() << " " << last_imu_carspeed.f32_value << "\n";
			file_speed.close();
			//counter_posesamples = 0;
		}
		counter_posesamples++;
	#endif

    if (m_bDebugModeEnabled) {
    	LOG_WARNING(cString::Format("CarPose [IMU Acc mode]: pose x %f, y %f, yaw imu: %f, dx: %f, dy: %f, accY: %f, imu speed: %f, dt: %f, car speed: %f",last_carPose.f32_x, last_carPose.f32_y, last_carPose.f32_yaw, delta_x, delta_y, imu_acc, last_imu_carspeed.f32_value, dt, GetLastCarSpeed()));
    }

    // save last wheel speed with arduino timestamp
	last_imu_acc = imu_acc_y;

	RETURN_NOERROR;
}

tResult CarPose::SaveWheelSpeedInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionWheelSpeedAccess);
	carSpeed.Read(pMediaSample, &wheel_speed_dir);
	if ( wheel_speed_dir.f32_value < 0.0 ) {
		car_speed_forwards = tFalse;
	} else if ( wheel_speed_dir.f32_value > 0.0 ) {
		car_speed_forwards = tTrue;
	}
	RETURN_NOERROR;
}

tResult CarPose::ProcessLeftStructInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionWheelTachAccess);

	counter_wheel_left_input++;

	TWheelData::Data rec_tach_left;
	wheelLeftIn.Read(pMediaSample, &rec_tach_left);

	if(!initialized_tach_left) {
		initialized_tach_left = tTrue;
		last_tach_left = rec_tach_left;
		ProcessTachInfo(0);
		RETURN_NOERROR;
	}

	if ( rec_tach_left.ui32_wheelTach > last_tach_left.ui32_wheelTach ) {
		ProcessTachInfo(rec_tach_left.ui32_wheelTach - last_tach_left.ui32_wheelTach);
		tick_counter += rec_tach_left.ui32_wheelTach - last_tach_left.ui32_wheelTach;
	}

	// poll overall tick counter with 40 Hz
	if(counter_wheel_left_input % 1 == 0) {
		PollWheelTach();
		counter_wheel_left_input = 0;
	}

	last_tach_left = rec_tach_left;
	RETURN_NOERROR;
}


tResult CarPose::ProcessRightStructInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionWheelTachAccess);

	TWheelData::Data rec_tach_right;
	wheelRightIn.Read(pMediaSample, &rec_tach_right);

	if(!initialized_tach_right) {
		initialized_tach_right = tTrue;
		last_tach_right = rec_tach_right;
		ProcessTachInfo(0);
		RETURN_NOERROR;
	}

	if ( rec_tach_right.ui32_wheelTach > last_tach_right.ui32_wheelTach ) {
		ProcessTachInfo(rec_tach_right.ui32_wheelTach - last_tach_right.ui32_wheelTach);
		tick_counter += rec_tach_right.ui32_wheelTach - last_tach_right.ui32_wheelTach;
	}

	last_tach_right = rec_tach_right;
	RETURN_NOERROR;
}

tResult CarPose::PollWheelTach() {

	__synchronized_obj(m_oCriticalSectionWheelTachAccess);

	// first run: save values
	if(!initialized_tach_overall) {
		initialized_tach_overall = tTrue;
		last_tach_overall.ui32_arduinoTimestamp = _clock->GetStreamTime();
		last_tach_overall.ui32_wheelTach = tick_counter;
		RETURN_NOERROR;
	}

	tUInt32 tick_diff = tick_counter - last_tach_overall.ui32_wheelTach;
	tFloat32 time_diff = ( _clock->GetStreamTime() - last_tach_overall.ui32_arduinoTimestamp ) / 1E6;

	tFloat32 distance_since_last_sample = fabsf( (m_f32wheelCircumference/CW_SLOT_COUNT) * tick_diff ) * 0.5; // * 0.5: two inputs, left and right

	//LOG_WARNING(cString::Format("CarPose [carSpeed]: Tickdiff: %d, time_diff: %f, distance since last poll: %f", tick_diff, time_diff, distance_since_last_sample));

	TSignalValue::Data tmp_carspeed;
	tmp_carspeed.ui32_arduinoTimestamp = 0; // TODO insert right time
	tmp_carspeed.f32_value = distance_since_last_sample / time_diff;

	if (m_bEnableFiltering) {
		tmp_carspeed.f32_value = last_calc_carspeed + m_f32FilterConstantfirstOrder*(tmp_carspeed.f32_value - last_calc_carspeed);
		last_calc_carspeed = tmp_carspeed.f32_value;
	}

	carSpeedOut.Transmit(&car_speed_output, tmp_carspeed, _clock->GetStreamTime());

	// save last overall tick counter and time
	last_tach_overall.ui32_arduinoTimestamp = _clock->GetStreamTime();
	last_tach_overall.ui32_wheelTach = tick_counter;

	RETURN_NOERROR;
}


tFloat32 CarPose::GetLastCarSpeed(){
	__synchronized_obj(m_oCriticalSectionWheelSpeedAccess);
	return wheel_speed_dir.f32_value;
}

tBool CarPose::GetWheelSpeedDirection(){
	__synchronized_obj(m_oCriticalSectionWheelSpeedAccess);
	return car_speed_forwards;
}

tResult CarPose::ProcessTachInfo(tUInt32 tachdiff){
	__synchronized_obj(m_oCriticalSectionPoseAccess);

	// get IMU an d steering angle
	tFloat32 current_imu_yaw = GetLastIMUYaw();
	tFloat32 current_steering_angle = GetLastSteeringAngle();

	// first run: save values
	if(!initialized) {
		initialized = tTrue;
		last_carPose.f32_x = 0;
		last_carPose.f32_y = 0;
		last_carPose.f32_yaw = 0;
		SetIMUYawOffset(current_imu_yaw);
		if (m_bDebugModeEnabled) {
			LOG_WARNING(cString::Format("CarPose: Initialized [Tach Mode] with IMU yaw: %f, tach diff: %d", current_imu_yaw, tachdiff ));
		}
		RETURN_NOERROR;
	}

	// distance since last overall sample
	tFloat32 distance_since_last_sample = fabsf( (m_f32wheelCircumference/CW_SLOT_COUNT) * tachdiff ) * 0.5; // * 0.5: two inputs, left and right

	// TODO check this workaround
	//if (fabsf(GetLastCarSpeed()) < 0.02) distance_since_last_sample = 0;

	if (m_bUseIMUYaw) {
		// calculate pose based on IMU yaw angle
		last_carPose.f32_yaw = ( (current_imu_yaw) / 180.0) * ( 3.1415926 ) * m_f32YawCorrection;  // absolute
	} else {
		// calculate pose based on steering angle
		last_carPose.f32_yaw += ( distance_since_last_sample / m_f32CarWheelBase ) * tan( ( (current_steering_angle) / 180.0 ) * 3.1415 );  // sum wheelbase
	}

	// check car direction based on wheel speed
	if (!GetWheelSpeedDirection()) {
		 distance_since_last_sample *= -1; // invert distance
	}

	// calculate relative distances
    tFloat32 delta_x =  distance_since_last_sample * cos(last_carPose.f32_yaw);
    tFloat32 delta_y =  distance_since_last_sample * sin(last_carPose.f32_yaw);

    last_carPose.f32_x += delta_x;
    last_carPose.f32_y += delta_y;
    last_carPose.ui32_arduinoTimestamp = last_tach_right.ui32_arduinoTimestamp;

    if (m_bDebugModeEnabled) {
    	LOG_WARNING(cString::Format("CarPose [Tach Mode]: pose x %f, y %f, yaw imu: %f, delta x %f, delta y %f, steering angle: %f",last_carPose.f32_x, last_carPose.f32_y, last_carPose.f32_yaw, delta_x, delta_y, GetLastSteeringAngle()));
    }

	RETURN_NOERROR;
}

tResult CarPose::ProcessOverallDistanceInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionPoseAccess);

	// read overall distance and get IMU
	TSignalValue::Data overall_distance;
	distanceOverall.Read(pMediaSample, &overall_distance);
	tFloat32 current_imu_yaw = GetLastIMUYaw();
	tFloat32 current_steering_angle = GetLastSteeringAngle();

	// first run: save values
	if(!initialized) {
		if( (firstReceivedIMU && m_bUseIMUYaw) || (firstReceivedSteering && !m_bUseIMUYaw)) {
			last_overall_distance = overall_distance;
			initialized = tTrue;
			last_carPose.f32_x = 0;
			last_carPose.f32_y = 0;
			last_carPose.f32_yaw = 0;
			SetIMUYawOffset(current_imu_yaw);
			if (m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("CarPose: Initialized [Overall Mode] IMU yaw: %f, overall distance: %f", current_imu_yaw, last_overall_distance.f32_value ));
			}
		} else {
			if (m_bDebugModeEnabled && m_bUseIMUYaw) {
				LOG_WARNING(cString::Format("CarPose [Overall Mode]: Waiting for first IMU sample to initialize"));
			} else if (m_bDebugModeEnabled && !m_bUseIMUYaw) {
				LOG_WARNING(cString::Format("CarPose [Overall Mode]: Waiting for first steering angle sample to initialize"));
			}
		}
		RETURN_NOERROR;
	}

	// distance since last overall sample
	tFloat32 distance_since_last_sample = fabsf(overall_distance.f32_value - last_overall_distance.f32_value);
	// TODO check this workaround
	if (fabsf(GetLastCarSpeed()) < 0.02) distance_since_last_sample = 0;

	if (m_bUseIMUYaw) {
		// calculate pose based on IMU yaw angle
		last_carPose.f32_yaw = ( (current_imu_yaw) / 180.0) * ( 3.1415926 ) * m_f32YawCorrection;  // absolute
	} else {
		// calculate pose based on steering angle
		last_carPose.f32_yaw += ( distance_since_last_sample / m_f32CarWheelBase ) * tan( ( (current_steering_angle) / 180.0 ) * 3.1415 );  // sum wheelbase
	}

	// check car direction based on wheel speed
	if (!GetWheelSpeedDirection()) {
		 distance_since_last_sample *= -1; // invert distance
	}

	// calculate relative distances
    tFloat32 delta_x =  distance_since_last_sample * cos(last_carPose.f32_yaw);
    tFloat32 delta_y =  distance_since_last_sample * sin(last_carPose.f32_yaw);

    last_carPose.f32_x += delta_x;
    last_carPose.f32_y += delta_y;
    last_carPose.ui32_arduinoTimestamp = overall_distance.ui32_arduinoTimestamp;


    if (m_bDebugModeEnabled) {
    	LOG_WARNING(cString::Format("CarPose [Overall Mode]: pose x %f, y %f, yaw imu: %f, delta x %f, delta y %f, steering angle: %f",last_carPose.f32_x, last_carPose.f32_y, last_carPose.f32_yaw, delta_x, delta_y, GetLastSteeringAngle()));
    }

    // save last overall distance with arduino timestamp
    last_overall_distance = overall_distance;

	RETURN_NOERROR;
}

TPoseStruct::Data CarPose::GetCarPose() {
	__synchronized_obj(m_oCriticalSectionPoseAccess);
	return last_carPose;
}

tResult CarPose::TransmitPose(tTimeStamp inputTime){
	__synchronized_obj(m_oCriticalSectionTransmit);

	TPoseStruct::Data tmp_pose;
	tmp_pose = GetCarPose();

	carPose.Transmit(&pose_output, tmp_pose, inputTime);

	RETURN_NOERROR;
}

tResult CarPose::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {

	// so we received a media sample, so this pointer better be valid.
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is (and if output exists?)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {

		// received IMU yaw, save sample
		if (pSource == &imuYaw_input) {
			ProcessIMUInput(pMediaSample);

		// received steering angle sample
		} else if (pSource == &steeringAngle_input) {
			ProcessSteeringAngle(pMediaSample);

		// received wheel speed input sample
		} else if (pSource == &carSpeed_input) {
			// save sample for car direction
			SaveWheelSpeedInput(pMediaSample);

		// received overall distance sample
		} else if (pSource == &distance_overall_input) {
			if(m_i32DistanceMode == 1) {
				// process received data
				ProcessOverallDistanceInput(pMediaSample);
				/** transmit car pose **/
				if(initialized) {
					TransmitPose(pMediaSample->GetTime());
				}
			}

		// received IMU acc input
		} else if (pSource == &imuAccY_input) {
			if(m_i32DistanceMode == 2) {
				// process received data
				ProcessImuAccInput(pMediaSample);
				/** transmit car pose **/
				if(initialized) {
					TransmitPose(pMediaSample->GetTime());
				}
			}

		// received wheel left struct sample
		} else if (pSource == &wheel_left_input) {
			if(m_i32DistanceMode == 3) {
				// save sample for car direction
				ProcessLeftStructInput(pMediaSample);
			}

		// received wheel right struct sample
		} else if (pSource == &wheel_right_input) {
			if(m_i32DistanceMode == 3) {
				// save sample for car direction
				ProcessRightStructInput(pMediaSample);
				if(initialized) {
					TransmitPose(pMediaSample->GetTime());
				}
			}
		}
	}

	RETURN_NOERROR;
}
