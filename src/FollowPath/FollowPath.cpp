/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ADTF Filter for calculation speed and steering angle to follow fix paths
 XML file parsing based on cCalibrationXml filter by Audi
**********************************************************************
* $Author::  $ fink  $Date:: 2016-01-25 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#include <stdio.h>
#include <math.h>
#include "stdafx.h"
#include "FollowPath.h"
#include "ScmCommunication.h"

#include <iostream>
#include <fstream>

/// Create filter shell
ADTF_FILTER_PLUGIN("Follow_Path", OID_ADTF_FOLLOW_PATH, FollowPath);

FollowPath::FollowPath(const tChar* __info) : cAsyncDataTriggeredFilter(__info) {

	m_f32SteeringAngleCorrection = 1.0; // factor for steering angle output
	m_f32DistanceToNextPathPoint = 0.07; // set path point to next point
	m_f32USDistanceToNextPathPoint = 0.15; // set path point to next point when in this range and US mode active
	m_f32DistanceToLastPathPoint = 0.1; // stop when [param] meters away from last point
	m_bSendGoalCoordinates = tFalse; // activates pose output pin
	m_bDebugModeEnabled = tFalse; // debug messsaged
	m_bDebugModeAllPoints = tFalse;  // all debug messages
	m_bPrintInitialXMLTable = tFalse; // print XML table to console
	m_f32ReduceSpeedLastPointDistance = 0.8; // reduce car speed when [param] meters away from last point
	debug_path_select = 2020; // default: turn left
	missed_points_counter = 5; // points missed counter (rescue mode)
	m_f32CarSpeedXYMode = 0.6; // car speed when in external goal mode
	m_f32US_error_distance = 0.03; // Ultrasonic condition: Error Distance (US value < [param] -> Error)
	m_f32maxGoalYawDeviation = 10.0; // max. permitted yaw deviation between actual goal yaw and set goal yaw
	m_bEnableRescue = tTrue; // Enable Rescue Mode

	// car control factors
	kalpha = 2.3; // old: 1.8, 1.6
	kbeta = -1.0;	// old: -0.7
	krho_forw = 1.5; // Car control factor krho forwards
	krho_back = 2.0; // Car control factor krho backwards
	reduction_offset = 0.05; // Speed Reduction Offset in m/s

	// Change properties BELOW, too!

	#ifdef DEBUG_TO_FILE
		// log file for path following when started
		LOG_WARNING("FollowPath: Extended debug mode enabled, writing to /tmp/logfile_followpath.txt");
		LogFile = fopen("/tmp/logfile_followpath.txt", "w");
		fprintf(LogFile, "index; pose absolute x; y; yaw; pose relative x; y; yaw; goal absolute x; y; yaw; speed; steering_angle; US sensor; US value; alpha; beta; beta_; rho; delta_x; delta_y; gamma\n");
	#endif

	#ifdef DEBUG_POSE_TO_FILE
		// log file for complete car pose
		LOG_WARNING("FollowPath: Extended debug mode for car pose enabled, writing to /tmp/car_pose.dat");
		fstream file_pose;
		file_pose.open("/tmp/car_pose.dat", ios::out | ios::trunc);
		file_pose << "# pose absolute x; y; yaw\n";
		file_pose.close();
		counter_posesamples = 0;
	#endif


    SetPropertyStr("FollowPath::Configuration file for path points", "../../../../utilities/FollowPathPoints.xml");
    SetPropertyBool("FollowPath::Configuration file for path points" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("FollowPath::Configuration file for path points" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("FollowPath::Configuration file for path points" NSSUBPROP_DESCRIPTION, "The XML file containing path points has to be set here");

    SetPropertyBool("Debugging::Print initial table to Console", m_bPrintInitialXMLTable);
    SetPropertyStr("Debugging::Print initial table to Console" NSSUBPROP_DESCRIPTION, "If enabled the loaded points of the paths (from XML) are printed to console");

	SetPropertyFloat("Legacy::Steering angle correction factor", 1.0);
	SetPropertyFloat("Legacy::Steering angle correction factor" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Legacy::Steering angle correction factor" NSSUBPROP_DESCRIPTION,
				"Correction factor for steering angle on output pin (not used)");

	SetPropertyFloat("FollowPath::Distance to next path point", 0.07);
	SetPropertyFloat("FollowPath::Distance to next path point" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Distance to next path point" NSSUBPROP_DESCRIPTION,
				"Distance to next path point in m");

	SetPropertyFloat("FollowPath::US Distance to next path point", 0.15);
	SetPropertyFloat("FollowPath::US Distance to next path point" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::US Distance to next path point" NSSUBPROP_DESCRIPTION,
				"Ultrasonic condition: Distance to next path point in m (when US mode is active & no US error)");

	SetPropertyFloat("FollowPath::US Error Distance", 0.03);
	SetPropertyFloat("FollowPath::US Error Distance" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::US Error Distance" NSSUBPROP_DESCRIPTION,
				"Ultrasonic condition: Error Distance (US value < [param] -> Error)");

	SetPropertyBool("FollowPath::Enable Rescue Mode", tTrue);
	SetPropertyBool("FollowPath::Enable Rescue Mode" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Enable Rescue Mode" NSSUBPROP_DESCRIPTION,
				"Enables Rescue Mode when Point missed");

	SetPropertyFloat("FollowPath::Car Speed in GOTOXY mode", 0.6);
	SetPropertyFloat("FollowPath::Car Speed in GOTOXY mode" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Car Speed in GOTOXY mode" NSSUBPROP_DESCRIPTION,
				"Car speed in GOTO XY (external goal) mode");

	SetPropertyFloat("Legacy::Distance to last path point", 0.1);
	SetPropertyFloat("Legacy::Distance to last path point" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Legacy::Distance to last path point" NSSUBPROP_DESCRIPTION,
				"Distance to last path point in m (not used)");

	SetPropertyFloat("FollowPath::kalpha", 2.3);
	SetPropertyFloat("FollowPath::kalpha" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::kalpha" NSSUBPROP_DESCRIPTION,
				"Car control factor kalpha (car to goal) (kalpha - krho > 0)");

	SetPropertyFloat("FollowPath::kbeta", -1.0);
	SetPropertyFloat("FollowPath::kbeta" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::kbeta" NSSUBPROP_DESCRIPTION,
				"Car control factor kbeta (car to goal angle) (kbeta < 0)");

	SetPropertyFloat("FollowPath::krho forwards", 1.5);
	SetPropertyFloat("FollowPath::krho forwards" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::krho forwards" NSSUBPROP_DESCRIPTION,
				"Car control factor krho forwards (speed reduction) (krho > 0)");

	SetPropertyFloat("FollowPath::krho backwards", 2.0);
	SetPropertyFloat("FollowPath::krho backwards" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::krho backwards" NSSUBPROP_DESCRIPTION,
				"Car control factor krho backwards (speed reduction) (krho > 0)");

	SetPropertyFloat("FollowPath::Speed Reduction Offset", 0.05);
	SetPropertyFloat("FollowPath::Speed Reduction Offset" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Speed Reduction Offset" NSSUBPROP_DESCRIPTION,
				"Speed Reduction Offset in m/s (increases absolute value of car speed when near next path point)");

	SetPropertyFloat("FollowPath::Max Goal Yaw deviation", 10.0);
	SetPropertyFloat("FollowPath::Max Goal Yaw deviation" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Max Goal Yaw deviation" NSSUBPROP_DESCRIPTION,
				"Max. permitted Yaw deviation between actual Goal Yaw and set Goal Yaw (in degrees)");

	SetPropertyInt("FollowPath::Rescue Mode Counter", 5);
	SetPropertyInt("FollowPath::Rescue Mode Counter" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("FollowPath::Rescue Mode Counter" NSSUBPROP_DESCRIPTION,
				"Rescue Mode Counter: points with increasing distance before next point is set (rescue mode)");

	SetPropertyBool("Legacy::Send relative goal coordinates on output pins", m_bSendGoalCoordinates);
	SetPropertyBool("Legacy::Send relative goal coordinates on output pins" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Legacy::Send relative goal coordinates on output pins" NSSUBPROP_DESCRIPTION,
				"Activate output (goal_out) of goal point (relative to car)(not used)");

	SetPropertyFloat("Legacy::Distance to reduce car speed", 0.8);
	SetPropertyFloat("Legacy::Distance to reduce car speed" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Legacy::Distance to reduce car speed" NSSUBPROP_DESCRIPTION,
				"Reduce car speed when car is [param] meters away from last path point (not used)");

	SetPropertyBool("Debugging::Enable Debug", m_bDebugModeEnabled);
	SetPropertyBool("Debugging::Enable Debug" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Debugging::Enable Debug" NSSUBPROP_DESCRIPTION,
				"Enables Debug messages");

	SetPropertyBool("Debugging::Enable Extended Debug", m_bDebugModeAllPoints);
	SetPropertyBool("Debugging::Enable Extended Debug" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Debugging::Enable Extended Debug" NSSUBPROP_DESCRIPTION,
				"Enables extended Debug messages (all path points)");

	SetPropertyInt("Debugging::Debug Path Select", debug_path_select);
	SetPropertyInt("Debugging::Debug Path Select" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Debugging::Debug Path Select" NSSUBPROP_DESCRIPTION,
				"Debug Path Select (Path ID form XML Path Points)");
}

FollowPath::~FollowPath() {
	#ifdef DEBUG_TO_FILE
		fclose(LogFile);
	#endif
}

tResult FollowPath::CreateInputPins(__exception)
{
	// create input pin for enabling path filter (debugging) (tBoolSignalValue)
	RETURN_IF_FAILED(enable_input.Create("enable", enableFilter.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&enable_input));

	// create input pin for car pose (tPoseStruct)
	RETURN_IF_FAILED(pose_input.Create("car_pose", carPose.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&pose_input));

	// Creation of the input pin of type tActionStruct (command input)
	RETURN_IF_FAILED(action_input.Create("action", PathAction.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&action_input));

	// Creation of the input pin of type tPoseStruct (external goal pose input)
	RETURN_IF_FAILED(goal_input.Create("goal_pose", ExternalGoal.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&goal_input));

	// Creation of the input pin of type tUlteasonciStruct
	RETURN_IF_FAILED(US_input.Create("US_struct", USSensors.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&US_input));

	RETURN_NOERROR;
}

tResult FollowPath::CreateOutputPins(__exception)
{

	/**** goal output (next path pose, relative to car) (tPoseStruct) *****/
	RETURN_IF_FAILED(next_goal_output.Create("goal_out", NextGoalPose.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&next_goal_output));

	/**** car speed (tSignalValue) *****/
	RETURN_IF_FAILED(car_speed_output.Create("car_speed", speedOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&car_speed_output));

	/**** Steering Angle (tSignalValue) *****/
	RETURN_IF_FAILED(steering_angle_output.Create("steering_angle", steeringOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&steering_angle_output));

	// FEEDBACK OUTPUT (tFeedbackStruct)
	RETURN_IF_FAILED(feedack_output.Create("feedback", feedbackOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedack_output));

	RETURN_NOERROR;
}


tResult FollowPath::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		// inputs
		RETURN_IF_FAILED(carPose.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(enableFilter.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(PathAction.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(ExternalGoal.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(USSensors.StageFirst(__exception_ptr));

		// outputs
		RETURN_IF_FAILED(NextGoalPose.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(speedOut.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(steeringOut.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(feedbackOut.StageFirst(__exception_ptr));

		// create and register the input and output pins
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface
		m_f32SteeringAngleCorrection = tFloat32(GetPropertyFloat("Legacy::Steering angle correction factor"));
		m_f32DistanceToNextPathPoint = tFloat32(GetPropertyFloat("FollowPath::Distance to next path point"));
		m_f32USDistanceToNextPathPoint = tFloat32(GetPropertyFloat("FollowPath::US Distance to next path point"));
		m_f32DistanceToLastPathPoint = tFloat32(GetPropertyFloat("Legacy::Distance to last path point"));
		m_bSendGoalCoordinates = tBool(GetPropertyBool("Legacy::Send relative goal coordinates on output pins"));
		m_bDebugModeEnabled = tBool(GetPropertyBool("Debugging::Enable Debug"));
		m_bDebugModeAllPoints = tBool(GetPropertyBool("Debugging::Enable Extended Debug"));
		m_f32ReduceSpeedLastPointDistance = tFloat32(GetPropertyFloat("Legacy::Distance to reduce car speed"));
		kalpha = tFloat32(GetPropertyFloat("FollowPath::kalpha"));
		kbeta = tFloat32(GetPropertyFloat("FollowPath::kbeta"));
		krho_forw = tFloat32(GetPropertyFloat("FollowPath::krho forwards"));
		krho_back = tFloat32(GetPropertyFloat("FollowPath::krho backwards"));
		reduction_offset = tFloat32(GetPropertyFloat("FollowPath::Speed Reduction Offset"));
		debug_path_select = tUInt32(GetPropertyInt("Debugging::Debug Path Select"));
		missed_points_counter = tUInt32(GetPropertyInt("FollowPath::Rescue Mode Counter"));
		m_f32CarSpeedXYMode = tFloat32(GetPropertyFloat("FollowPath::Car Speed in GOTOXY mode"));
		m_f32US_error_distance = tFloat32(GetPropertyFloat("FollowPath::US Error Distance"));
		m_f32maxGoalYawDeviation = (tFloat32(GetPropertyFloat("FollowPath::Max Goal Yaw deviation"))/180.f)*M_PI;
		m_bEnableRescue = tBool(GetPropertyBool("FollowPath::Enable Rescue Mode"));

	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.

		// inputs
		RETURN_IF_FAILED(carPose.StageGraphReady());
		RETURN_IF_FAILED(enableFilter.StageGraphReady());
		RETURN_IF_FAILED(PathAction.StageGraphReady());
		RETURN_IF_FAILED(ExternalGoal.StageGraphReady());
		RETURN_IF_FAILED(USSensors.StageGraphReady());

		// outputs
		RETURN_IF_FAILED(NextGoalPose.StageGraphReady());
		RETURN_IF_FAILED(speedOut.StageGraphReady());
		RETURN_IF_FAILED(steeringOut.StageGraphReady());
		RETURN_IF_FAILED(feedbackOut.StageGraphReady());

		running = tFalse; // set running state
		wait_samples = tFalse; // change in car direction, wait flag
		reset_pose = tTrue; // reset pose on startup
		last_is_rescue_point = tFalse; // last point is rescue point
		activate_steering_output = tTrue; // False: no steering output, drive path [xValue] meters overall
		point_index = 0; // path point index
		received_command = 0; // received state machine action struct command
		boolean_start = tFalse; // for boolean start input (debugging)
		direction = 0; // initial run, direction 0
		sample_counter = 0;	// counts received overall distance samples
		last_completed_path = 0; // save last completed path for state machine request
		points_in_path = 0; // number of points in path
		last_alpha = 0; // save last alpha
		last_rho = 0; // first last rho: 0
		point_missed_counter = 0; // counts rho values greater than last rho

        //load XML files with path points
        THROW_IF_FAILED(LoadPathPointData());
	}

	RETURN_NOERROR;
}

tResult FollowPath::Start(__exception)
{
	return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult FollowPath::Stop(__exception)
{
	return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}

tResult FollowPath::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	// call the base class implementation
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult FollowPath::ProcessPoseInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionPoseAccess);

	// save last pose sample
	carPose.Read(pMediaSample, &car_pose_last);

	// log pose to file
	#ifdef DEBUG_POSE_TO_FILE
		if ( counter_posesamples % 5 == 0) { // 1 out of 5
			fstream file_pose;
			file_pose.open("/tmp/car_pose.dat", ios::out | ios::app);
			file_pose << car_pose_last.f32_x << " " << car_pose_last.f32_y << " " << car_pose_last.f32_yaw << "\n";
			file_pose.close();
			counter_posesamples = 0;
		}
		counter_posesamples++;
	#endif

	RETURN_NOERROR;
}

tResult FollowPath::ProcessGoalPointInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionGoalAccess);

	// save external goal pose sample
	ExternalGoal.Read(pMediaSample, &external_goal);

	RETURN_NOERROR;
}


tResult FollowPath::ProcessEnableInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionTriggerOffsetAccess);

	// read enable input
	TBoolSignalValue::Data TriggerInput;
	enableFilter.Read(pMediaSample, &TriggerInput);

	// for debugging: set running state when enable received
	if(!GetRunningState() && TriggerInput.bValue) {
		SetRunningState(tTrue);

		// get pose offset
		car_pose_offset = GetCarPose();
		SetCommand(debug_path_select); // drive right turn
 
		if (m_bDebugModeEnabled) {
			LOG_WARNING(cString::Format("FollowPath Started (enable input): initial car_pose_x: %f, car_pose_y: %f, car_pose_yaw: %f", car_pose_offset.f32_x, car_pose_offset.f32_y, car_pose_offset.f32_yaw));
		}

		point_index = 0; // reset path point counter
		wait_samples = tFalse; // reset wait flag
		boolean_start = tTrue; // boolean start input received
		reset_pose = tTrue; // reset pose on startup
		last_is_rescue_point = tFalse; // last point is rescue point
		activate_steering_output = tTrue; // False: no steering output, drive path [xValue] meters overall
		direction = 0; // reset direction
		last_alpha = 0; // first last alpha: 0
		last_rho = 0; // first last rho: 0
		point_missed_counter = 0; // counts rho values greater than last rho

		SetPath(); // st path
		RETURN_NOERROR;
	}

	// disable filter
	if (!TriggerInput.bValue) {
		SetRunningState(tFalse);
	}

	RETURN_NOERROR;
}

tResult FollowPath::ProcessActionInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionActionAccess);

	TActionStruct::ActionSub actionSub;
	actionSub = PathAction.Read_Action(pMediaSample, F_FOLLOW_PATH);

	// set running state
	if(!GetRunningState() && actionSub.enabled && actionSub.started) {

		// reset parking status
		if (actionSub.command == AC_FP_RESET_PARKING_STATUS) {
			last_completed_path = 0;
		}

		// parking status request
		else if (actionSub.command == AC_FP_GET_PARKING_STATUS) {
			TFeedbackStruct::Data feedback;
			feedback.ui32_filterID = F_FOLLOW_PATH;
			if(last_completed_path == AC_FP_PARKING_LONG) {
				feedback.ui32_status = FB_FP_PARKING_STATUS_PARALLEL;
			} else if(last_completed_path == AC_FP_PARKING_TRANS) {
				feedback.ui32_status = FB_FP_PARKING_STATUS_TRANSVERSE;
			} else {
				feedback.ui32_status = FB_FP_PARKING_STATUS_NO_PARKING;
			}
			// send parking status feedback
			feedbackOut.Transmit(&feedack_output, feedback, _clock->GetStreamTime());

		// enable path following
		} else {
			point_index = 0; // reset counter
			wait_samples = tFalse; // reset wait flag
			boolean_start = tFalse; // no boolean start input received
			reset_pose = tTrue; // reset pose on startup
			last_is_rescue_point = tFalse; // last point is rescue point
			activate_steering_output = tTrue; // False: no steering output, drive path [xValue] meters overall
			direction = 0; // reset direction
			last_alpha = 0; // first last alpha: 0
			last_rho = 0; // first last rho: 0
			point_missed_counter = 0; // counts rho values greater than last rho

			// set command
			SetCommand(actionSub.command);

			// set path and check return
			tResult tmpReturn = SetPath();
			if(tmpReturn < 0){
				LOG_ERROR(cString::Format("FollowPath: ERROR occurred in SetPath(), not started"));
			} else {
				// get pose offset
				if(reset_pose) car_pose_offset = GetCarPose();
				// set running state
				SetRunningState(tTrue);
				if (m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("FollowPath Started: initial car_pose_x: %f, car_pose_y: %f, car_pose_yaw: %f, received command: %d, resetted pose: %d", car_pose_offset.f32_x, car_pose_offset.f32_y, car_pose_offset.f32_yaw, actionSub.command, reset_pose));
				}
			}
		}

		RETURN_NOERROR;
	}

	// send disable information when in running mode
	// TODO stop feedback when filter is stopped, not disabled
	if (!actionSub.enabled && GetRunningState()) {
		SetRunningState(tFalse);

		// transmit feedback struct with filter ID and FB_FP_STOPPED
		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_FOLLOW_PATH;
		feedback.ui32_status = FB_FP_STOPPED; // TODO check feedback
		feedbackOut.Transmit(&feedack_output, feedback, _clock->GetStreamTime());
	}

	RETURN_NOERROR;
}

tResult FollowPath::ProcessUltrasonicInput(IMediaSample* mediaSample) {
	__synchronized_obj(m_oCriticalSectionUSAccess);

	// get ultrasonic media sample
	USSensors.Read(mediaSample, &ultrasonic_last);
	/*LOG_WARNING(cString::Format("Ultrasonic Check input sensor values: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
			ultrasonic_last.FrontCenter.f32_value,
			ultrasonic_last.FrontCenterLeft.f32_value,
			ultrasonic_last.FrontCenterRight.f32_value,
			ultrasonic_last.FrontLeft.f32_value,
			ultrasonic_last.FrontRight.f32_value,
			ultrasonic_last.RearCenter.f32_value,
			ultrasonic_last.RearLeft.f32_value,
			ultrasonic_last.RearRight.f32_value,
			ultrasonic_last.SideLeft.f32_value,
			ultrasonic_last.SideRight.f32_value));
	*/
	RETURN_NOERROR;
}

tFloat32 FollowPath::GetUSSensorValue(tInt32 sensor_choice) {
	__synchronized_obj(m_oCriticalSectionUSAccess);

	tFloat32 return_value;

	// get relevant US sensor value
	switch (sensor_choice) {
		case 1:
			return_value = ultrasonic_last.FrontLeft.f32_value;
			break;
		case 2:
			return_value = ultrasonic_last.FrontCenterLeft.f32_value;
			break;
		case 3:
			return_value = ultrasonic_last.FrontCenter.f32_value;
			break;
		case 4:
			return_value = ultrasonic_last.FrontCenterRight.f32_value;
			break;
		case 5:
			return_value = ultrasonic_last.FrontRight.f32_value;
			break;
		case 6:
			return_value = ultrasonic_last.SideRight.f32_value;
			break;
		case 7:
			return_value = ultrasonic_last.RearRight.f32_value;
			break;
		case 8:
			return_value = ultrasonic_last.RearCenter.f32_value;
			break;
		case 9:
			return_value = ultrasonic_last.RearLeft.f32_value;
			break;
		case 10:
			return_value = ultrasonic_last.SideLeft.f32_value;
			break;
		default:
			return_value = 10.0; // high value
	}

	return return_value;

}

tUInt32 FollowPath::GetCommand(){
	__synchronized_obj(m_oCriticalSectionCommandAccess);
	return received_command;
}

tResult FollowPath::SetCommand(tUInt32 command){
	__synchronized_obj(m_oCriticalSectionCommandAccess);
	received_command = command;
	RETURN_NOERROR;
}

tInt32 FollowPath::GetUSSensor(){
	__synchronized_obj(m_oCriticalSectionUSSelectAccess);
	return US_sensor_select;
}

tResult FollowPath::SetUSSensor(tInt32 sensor){
	__synchronized_obj(m_oCriticalSectionUSSelectAccess);
	US_sensor_select = sensor;
	RETURN_NOERROR;
}

TPoseStruct::Data FollowPath::GetExternalGoal(){
	__synchronized_obj(m_oCriticalSectionGoalAccess);
	return external_goal;
}

tResult FollowPath::SetPath() {
	__synchronized_obj(m_oCriticalSectionPathAccess);

	// get action commands
	tUInt32 tmp_command = GetCommand();
	path_to_goal.clear();

	// check for external goal input (stop/no stop)
	if(tmp_command == AC_FP_GOTO_XY || tmp_command == AC_FP_GOTO_XY_NOSTOP || tmp_command == AC_FP_GOTO_XY_NOSTEERING || tmp_command == AC_FP_GOTO_XY_NOSTEERING_NOSTOP) {
		path_to_goal.resize(1); // only one point
		TPoseStruct::Data tmp_external_goal = GetExternalGoal();
		if(tmp_command == AC_FP_GOTO_XY_NOSTEERING || tmp_command == AC_FP_GOTO_XY_NOSTEERING_NOSTOP ) {
			activate_steering_output = tFalse;
			path_to_goal[0].x = sqrt(tmp_external_goal.f32_x * tmp_external_goal.f32_x + tmp_external_goal.f32_y * tmp_external_goal.f32_y);
			path_to_goal[0].y = 0;
			path_to_goal[0].yaw = 0;
		} else {
			path_to_goal[0].x = tmp_external_goal.f32_x;
			path_to_goal[0].y = tmp_external_goal.f32_y;
			path_to_goal[0].yaw = tmp_external_goal.f32_yaw;
		}
		// set negative car speed if point is behind car (x < 0)
		if(tmp_external_goal.f32_x < 0) {
			path_to_goal[0].car_speed = -m_f32CarSpeedXYMode; // set property car speed negative
		} else {
			path_to_goal[0].car_speed = m_f32CarSpeedXYMode; // set property car speed
		}
		points_in_path = 1;

		if(m_bDebugModeEnabled) {
			LOG_WARNING(cString::Format("Follow Path: Enabled in GOTO XY mode with command id %d, Steering Output: %d, Goal x: %f, y: %f, yaw: %f, speed: %f", tmp_command, activate_steering_output, path_to_goal[0].x, path_to_goal[0].y, path_to_goal[0].yaw, path_to_goal[0].car_speed));
		}

	} else {
		// search in xml_paths for command id
		std::vector<fp_paths>::iterator it = xml_paths.begin();
		while(it != xml_paths.end()) {
			if(it->path_id == tmp_command) {
				break;
			}
			it++;
		}

		// check if path for command id exists
		if (it != xml_paths.end() && it->points.size() > 0) {
			path_to_goal.resize(it->points.size());
			path_to_goal = it->points; // get points
			reset_pose = it->reset_pose; // get reset pose flag
			last_is_rescue_point = it->rescue_point;
			activate_steering_output = it->steering;
			// Set US mode info
			SetUSSensor(path_to_goal[0].us_sensor);
			// set number of points
			if(last_is_rescue_point) {
				points_in_path = it->points.size()-1;
			} else {
				points_in_path = it->points.size();
			}
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Follow Path: Enabled in FollowPath mode with command id %d, points in path: %d, last is rescue: %d, steering enabled: %d", tmp_command, points_in_path, last_is_rescue_point, GetSteeringState()));
			}
		} else {
			LOG_ERROR(cString::Format("FollowPath: No points in path for command %d", tmp_command));
			RETURN_ERROR(ERR_INVALID_STATE);
		}

	}

	RETURN_NOERROR;
}

tBool FollowPath::GetRunningState(){
	__synchronized_obj(m_oCriticalSectionRunningStateAccess);
	return running;
}

tResult FollowPath::SetRunningState(tBool state){
	__synchronized_obj(m_oCriticalSectionRunningStateAccess);
	running = state;
	RETURN_NOERROR;
}

tBool FollowPath::GetSteeringState(){
	__synchronized_obj(m_oCriticalSteeringStateAccess);
	return activate_steering_output;
}

TPoseStruct::Data FollowPath::GetCarPoseOffset() {
	__synchronized_obj(m_oCriticalSectionTriggerOffsetAccess);
	return car_pose_offset;
}

TPoseStruct::Data FollowPath::GetCarPose() {
	__synchronized_obj(m_oCriticalSectionPoseAccess);
	return car_pose_last;
}

tResult FollowPath::CalculateTransmitGoalNoSteering(){
	__synchronized_obj(m_oCriticalSectionGoalTransmit);

	// get car pose and car pose offset
	TPoseStruct::Data car_pose_tmp = GetCarPose();
	TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();

	// calculate car offset
	car_pose_tmp.f32_x -= car_pose_offset_tmp.f32_x;
	car_pose_tmp.f32_y -= car_pose_offset_tmp.f32_y;
	car_pose_tmp.f32_yaw -= car_pose_offset_tmp.f32_yaw;

	// driven distance since start
	tFloat32 overall_distance = sqrt (car_pose_tmp.f32_x * car_pose_tmp.f32_x + car_pose_tmp.f32_y * car_pose_tmp.f32_y);

	// generate output data
	TSignalValue::Data car_speed; // car speed (set in XML)
	// set output data timestamps
	car_speed.ui32_arduinoTimestamp = car_pose_tmp.ui32_arduinoTimestamp;

	// get car speed from XML
	car_speed.f32_value = path_to_goal[point_index].car_speed;

	// distance to goal
	tFloat32 rho = fabsf(path_to_goal[point_index].x) - overall_distance;

	// if near next goal
	if ( rho < m_f32DistanceToNextPathPoint || ( rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) ) {

		// set next goal pose
		if (point_index < ( points_in_path - 1 )) {

			// next point speed 0.0: set flag and wait a few samples before transmitting new car speed
			if (path_to_goal[point_index+1].car_speed == 0.0  ) {
				wait_samples = tTrue;
				sample_counter = 0;
				// debug message
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Follow Path [No Steering]: car speed 0 at next path point, waiting few samples"));
				}
			}

			// increase path point counter
			point_index++;

			// debug message for US mode
			if(m_bDebugModeEnabled) {
				if(rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) {
					LOG_WARNING(cString::Format("Follow Path [No Steering]: point %d reached in US mode, new point index: %d, US select: %d, US distance: %f, rho: %f", point_index-1, point_index, GetUSSensor(), GetUSSensorValue(GetUSSensor()), rho));
				}
			}

			// Set US mode info
			SetUSSensor(path_to_goal[point_index].us_sensor);

			// debug message
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Follow Path [No Steering]: next point, index: %d, driven distance: %f, goal distance: %f, rho: %f", point_index, overall_distance, path_to_goal[point_index].x, rho));
			}

			// => new distance to goal
			rho = fabsf(path_to_goal[point_index].x) - overall_distance;

			// => new car speed from xml
			car_speed.f32_value = path_to_goal[point_index].car_speed;

		// end of list reached
		} else {

			// save actual path as last completed path
			last_completed_path = GetCommand();

			// set car speed to zero
			if(path_to_goal[point_index].car_speed == 0.0) {
				car_speed.f32_value = 0.0;
			}

			// send feedback with received command + 1
			if (!boolean_start) {
				TFeedbackStruct::Data feedback;
				feedback.ui32_filterID = F_FOLLOW_PATH;
				feedback.ui32_status = GetCommand()+1; // TODO check feedback
				feedbackOut.Transmit(&feedack_output, feedback, _clock->GetStreamTime());
			}

			// stop car
			if(GetCommand() == AC_FP_GOTO_XY || GetCommand() == AC_FP_GOTO_XY_NOSTEERING || boolean_start) {
				car_speed.f32_value = 0.0; // goal reached, set wheel speed to zero if in GOTO XY mode
				boolean_start = tFalse; // or if triggered through boolean input
			}

			SetRunningState (tFalse); // set running state false
			wait_samples = tFalse; // dont wait when last point is reached, wait with state machine!

			// debug message for US mode
			if(m_bDebugModeEnabled) {
				if(rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) {
					LOG_WARNING(cString::Format("Follow Path [No Steering]: end point reached in US mode, point index: %d, US select: %d, US distance: %f, rho: %f", point_index, GetUSSensor(), GetUSSensorValue(GetUSSensor()), rho));
				}
			}

			// debug warning
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Follow Path [No Steering]: path end point index %d reached, driven distance: %f, goal distance: %f, rho: %f", point_index, overall_distance, path_to_goal[point_index].x, rho));
			}
		}
	}

	tFloat32 reduced_speed = car_speed.f32_value;
	// conditions to reduce speed (in external pose mode, enabled with enable input and last point, direction change in next path pose, stop in next path point (add same pose with 0.0 speed to stop car))
	if ( GetCommand() == AC_FP_GOTO_XY || GetCommand() == AC_FP_GOTO_XY_NOSTEERING || ( boolean_start && point_index == ( points_in_path - 1 ) ) ) {
		if(reduced_speed < 0) {
			reduced_speed = krho_back * rho * car_speed.f32_value - reduction_offset;
		} else if (reduced_speed > 0) {
			reduced_speed = krho_forw * rho * car_speed.f32_value + reduction_offset;
		}
	} else if (( point_index < ( points_in_path - 1 ) )) {
		if (path_to_goal[point_index+1].car_speed == 0.0 ) {
			if(reduced_speed < 0) {
				reduced_speed = krho_back * rho * car_speed.f32_value - reduction_offset;
			} else if (reduced_speed > 0) {
				reduced_speed = krho_forw * rho * car_speed.f32_value + reduction_offset;
			}
		}
	}

	// only set reduced speed if less than XML speed
	if( fabsf(reduced_speed) < fabsf(car_speed.f32_value)) {
		car_speed.f32_value = reduced_speed;
	}

	// limit car speed
	if (car_speed.f32_value > 1.0) {
		car_speed.f32_value = 1.0;
		//LOG_WARNING("FollowPath: car speed greater than 1.0");
	} else if (car_speed.f32_value < -1.0) {
		car_speed.f32_value = -1.0;
		//LOG_WARNING("FollowPath: car speed less than -1.0");
	}

	if(GetRunningState()) {
		// log all values to file
		#ifdef DEBUG_TO_FILE
			fprintf(LogFile, "%d;%f;%f;%f;%f\n", point_index, overall_distance, path_to_goal[point_index].x, car_speed.f32_value, rho);
		#endif

		// Debug all values
		if(m_bDebugModeAllPoints) {
			LOG_WARNING(cString::Format("FollowPath [No Steering]: index: %d, distance overall: %f, goal distance: %f, speed: %f, rho: %f", point_index, overall_distance, path_to_goal[point_index].x, car_speed.f32_value, rho));
		}
	}

	// create new media sample for car speed output
	// if direction change in car speed, send 60 samples (1.5 s) with car speed 0.0
	if(wait_samples && sample_counter < 60) {
		car_speed.f32_value = 0.0;
		sample_counter++;
	} else {
		wait_samples = tFalse;
	}
	speedOut.Transmit(&car_speed_output, car_speed, _clock->GetStreamTime());

	RETURN_NOERROR;
}

tResult FollowPath::CalculateTransmitGoal(){
	__synchronized_obj(m_oCriticalSectionGoalTransmit);

	// get car pose and car pose offset
	TPoseStruct::Data car_pose_tmp = GetCarPose();
	TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();

	// calculate car offset
	car_pose_tmp.f32_x -= car_pose_offset_tmp.f32_x;
	car_pose_tmp.f32_y -= car_pose_offset_tmp.f32_y;
	car_pose_tmp.f32_yaw -= car_pose_offset_tmp.f32_yaw;

	// car pose yaw: -4.71 -> 1.57 (convert yaw angle to values between -pi and pi) 
	if(car_pose_tmp.f32_yaw < -M_PI) {
		car_pose_tmp.f32_yaw += 2*M_PI;	
	} else if(car_pose_tmp.f32_yaw > M_PI) {
		car_pose_tmp.f32_yaw -= 2*M_PI;
	}

	// rotate coordinates for local pose
	tFloat32 y_sin = ( car_pose_tmp.f32_y ) * sin(car_pose_offset_tmp.f32_yaw);
	tFloat32 x_cos =  ( car_pose_tmp.f32_x ) * cos(car_pose_offset_tmp.f32_yaw);
	tFloat32 x_sin = - ( car_pose_tmp.f32_x ) * sin(car_pose_offset_tmp.f32_yaw);
	tFloat32 y_cos = ( car_pose_tmp.f32_y ) * cos(car_pose_offset_tmp.f32_yaw);

	tFloat32 temp_1 = y_sin + x_cos;
	tFloat32 temp_2 = x_sin + y_cos;

	// workaround for rotation. TODO why??
	// car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
	car_pose_tmp.f32_x = temp_1;
	car_pose_tmp.f32_y = temp_2;

	// generate output data
	TPoseStruct::Data output_goal; // goal pose output for external controller
	TSignalValue::Data steering_angle; // steering angle
	TSignalValue::Data car_speed; // car speed (set in XML)

	// set output data timestamps
	output_goal.ui32_arduinoTimestamp = car_pose_tmp.ui32_arduinoTimestamp;
	steering_angle.ui32_arduinoTimestamp = car_pose_tmp.ui32_arduinoTimestamp;
	car_speed.ui32_arduinoTimestamp = car_pose_tmp.ui32_arduinoTimestamp;

	// calculate delta x and delta y (car pose - goal point)
	tFloat32 delta_x;
	delta_x = car_pose_tmp.f32_x - path_to_goal[point_index].x;
	tFloat32 delta_y;
	delta_y =  car_pose_tmp.f32_y - path_to_goal[point_index].y;

	// distance to goal
	tFloat32 rho;
	rho = sqrt ( ( delta_x ) * ( delta_x )  + ( delta_y ) * ( delta_y ) );

	// get car speed from XML
	car_speed.f32_value = path_to_goal[point_index].car_speed;

	// if near next goal (range, US range or point with speed 0.0)
	if ( rho < m_f32DistanceToNextPathPoint || car_speed.f32_value == 0.0 || ( rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) ) {

		if(car_speed.f32_value == 0.0 && rho > m_f32DistanceToNextPathPoint) {
			LOG_WARNING("Follow Path: skipped point with speed 0.0 (and out of range)");
		}

		// set next goal pose
		if (point_index < ( points_in_path - 1 )) {

			// next point with new direction: set flag and wait a few samples before transmitting new car speed
			if (path_to_goal[point_index+1].car_speed == 0.0 || ( car_speed.f32_value > 0 && path_to_goal[point_index+1].car_speed < 0 ) || ( car_speed.f32_value < 0 && path_to_goal[point_index+1].car_speed > 0 ) ) {
				wait_samples = tTrue;
				sample_counter = 0;
				// debug message
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Follow Path: change in car direction or car speed 0 at next path point, waiting few samples"));
				}
			}

			// increase path point counter
			point_index++;
			direction = 0; // reset direction

			// debug message for US mode
			if(m_bDebugModeEnabled) {
				if(rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) {
					LOG_WARNING(cString::Format("Follow Path: point %d reached in US mode, new point index: %d, US select: %d, US distance: %f, rho: %f", point_index-1, point_index, GetUSSensor(), GetUSSensorValue(GetUSSensor()), rho));
				}
			}

			// Set US mode info
			SetUSSensor(path_to_goal[point_index].us_sensor);

			// debug message
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Follow Path: next point, index: %d, pose x: %f, y: %f, yaw: %f, goal x: %f, y: %f, yaw: %f, rho: %f", point_index, car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw, path_to_goal[point_index].x, path_to_goal[point_index].y,path_to_goal[point_index].yaw, rho));
			}

			// => new delta x and delta y (car pose - goal point)
			delta_x = car_pose_tmp.f32_x - path_to_goal[point_index].x;
			delta_y =  car_pose_tmp.f32_y - path_to_goal[point_index].y;

			// => new distance to goal
			rho = sqrt ( ( delta_x ) * ( delta_x )  + ( delta_y ) * ( delta_y ) );

			// => new car speed from xml
			car_speed.f32_value = path_to_goal[point_index].car_speed;

		// end of list reached
		} else {

			// save actual path as last completed path
			last_completed_path = GetCommand();

			// set car speed to zero
			if(path_to_goal[point_index].car_speed == 0.0) {
				car_speed.f32_value = 0.0;
			}

			// send feedback with received command + 1 ( + 2 -> BADYAW)
			if (!boolean_start) {
				TFeedbackStruct::Data feedback;
				feedback.ui32_filterID = F_FOLLOW_PATH;
				if(fabsf(car_pose_tmp.f32_yaw - path_to_goal[point_index].yaw) > m_f32maxGoalYawDeviation) {
					feedback.ui32_status = GetCommand()+2; // Feedback: BADYAW, TODO check feedback
					if(m_bDebugModeEnabled) {
						LOG_WARNING(cString::Format("Follow Path: end point reached, but large deviation between Yaw agles; set Yaw: %f, actual Yaw: %f, deviation: %f", path_to_goal[point_index].yaw, car_pose_tmp.f32_yaw, fabsf(car_pose_tmp.f32_yaw - path_to_goal[point_index].yaw)));
					}
				} else {
					feedback.ui32_status = GetCommand()+1; // Normal Feedback, TODO check feedback
				}
				feedbackOut.Transmit(&feedack_output, feedback, _clock->GetStreamTime());
			}

			// stop car
			if(GetCommand() == AC_FP_GOTO_XY || GetCommand() == AC_FP_GOTO_XY_NOSTEERING || boolean_start) {
				car_speed.f32_value = 0.0; // goal reached, set wheel speed to zero if in GOTO XY mode
				boolean_start = tFalse; // or if triggered through boolean input
			}

			SetRunningState (tFalse); // set running state false
			direction = 0; // reset direction
			wait_samples = tFalse; // dont wait when last point is reached, wait with state machine!
			steering_angle.f32_value = 90.0; // last steering angle: straight

			// debug message for US mode
			if(m_bDebugModeEnabled) {
				if(rho < m_f32USDistanceToNextPathPoint && GetUSSensorValue(GetUSSensor()) < path_to_goal[point_index].min_distance && GetUSSensorValue(GetUSSensor()) > m_f32US_error_distance && GetUSSensor() > 0) {
					LOG_WARNING(cString::Format("Follow Path: end point reached in US mode, point index: %d, US select: %d, US distance: %f, rho: %f", point_index, GetUSSensor(), GetUSSensorValue(GetUSSensor()), rho));
				}
			}

			// debug warning
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Follow Path: path end point index %d reached, pose x: %f, y: %f, yaw: %f, goal x: %f, y: %f, yaw: %f, rho: %f", point_index, car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw, path_to_goal[point_index].x, path_to_goal[point_index].y,path_to_goal[point_index].yaw, rho));
			}
		}
	}

	tFloat32 reduced_speed = car_speed.f32_value;
	// conditions to reduce speed (in external pose mode, enabled with enable input and last point, direction change in next path pose, stop in next path point (add same pose with 0.0 speed to stop car))
	if ( GetCommand() == AC_FP_GOTO_XY || GetCommand() == AC_FP_GOTO_XY_NOSTEERING || ( boolean_start && point_index == ( points_in_path - 1 ) ) ) {
		if(reduced_speed < 0) {
			reduced_speed = krho_back * rho * car_speed.f32_value - reduction_offset;
		} else if (reduced_speed > 0) {
			reduced_speed = krho_forw * rho * car_speed.f32_value + reduction_offset;
		}
	} else if (( point_index < ( points_in_path - 1 ) )) {
		if (path_to_goal[point_index+1].car_speed == 0.0 || ( car_speed.f32_value > 0 && path_to_goal[point_index+1].car_speed < 0 ) || ( car_speed.f32_value < 0 && path_to_goal[point_index+1].car_speed > 0 ) ) {
			if(reduced_speed < 0) {
				reduced_speed = krho_back * rho * car_speed.f32_value - reduction_offset;
			} else if (reduced_speed > 0) {
				reduced_speed = krho_forw * rho * car_speed.f32_value + reduction_offset;
			}
		}
	}

	// only set reduced speed if less than XML speed
	if( fabsf(reduced_speed) < fabsf(car_speed.f32_value)) {
		car_speed.f32_value = reduced_speed;
	}

	// limit car speed
	if (car_speed.f32_value > 1.0) {
		car_speed.f32_value = 1.0;
		//LOG_WARNING("FollowPath: car speed greater than 1.0");
	} else if (car_speed.f32_value < -1.0) {
		car_speed.f32_value = -1.0;
		//LOG_WARNING("FollowPath: car speed less than -1.0");
	}

	tBool send_steering_angle = tTrue;

	// calculate steering angle
	if(GetRunningState()) {

		tFloat32 beta;
		tFloat32 alpha;
		tFloat32 gamma;

		// calculation based on 'polar_sfunc.m' (p. corke robotics toolbox)
		if (direction == 0) { // first run
			beta = -atan2(-delta_y, -delta_x);
			alpha = -car_pose_tmp.f32_yaw - beta;

			// first run: direction of travel
			if (alpha > M_PI_2 || alpha < -M_PI_2) {
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("FollowPath: driving backwards to x %f, y %f, yaw %f, set speed: %f, alpha %f, beta %f", path_to_goal[point_index].x, path_to_goal[point_index].y, path_to_goal[point_index].yaw, path_to_goal[point_index].car_speed, alpha, beta));
				}
				direction = -1;
			} else {
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("FollowPath: driving forwards to x %f, y %f, yaw %f, set speed: %f, alpha %f, beta %f", path_to_goal[point_index].x, path_to_goal[point_index].y,path_to_goal[point_index].yaw, path_to_goal[point_index].car_speed, alpha, beta));
				}
				direction = 1;
			}
			last_alpha = 0;
			RETURN_NOERROR;
		} else if (direction == -1) { // backwards
			beta = -atan2(delta_y, delta_x);
			alpha = -car_pose_tmp.f32_yaw - beta;
		} else { // forwards
			beta = -atan2(-delta_y, -delta_x);
			alpha = -car_pose_tmp.f32_yaw - beta;
		}

		tFloat32 beta_ = beta + path_to_goal[point_index].yaw;  // add goal pose

		// limit alpha
		if (alpha > M_PI_2) {
			alpha = M_PI_2;
		} else if (alpha < -M_PI_2) {
			alpha = -M_PI_2;
		}

		if(m_bEnableRescue) {
			//if( GetCommand() != AC_FP_GOTO_XY &&  GetCommand() != AC_FP_GOTO_XY_NOSTEERING && GetCommand() != AC_FP_GOTO_XY_NOSTOP && rho > last_rho) {
			if( rho > last_rho && !wait_samples) { // alsways enable rescue; TODO check
				point_missed_counter++;
				if(rho < m_f32USDistanceToNextPathPoint) {
					 send_steering_angle = tFalse;
				}
			} else {
				point_missed_counter = 0;
			}
			//if (((alpha > 0 && last_alpha < 0) || (alpha < 0 && last_alpha > 0) ) && fabsf(last_alpha-alpha) > 0.1 && GetCommand() != AC_FP_GOTO_XY && GetCommand() != AC_FP_GOTO_XY_NOSTOP) {
			// rescue check: path point missed (and no new point)
			if( point_missed_counter > missed_points_counter) { // point missed
				tUInt32 missed_counter_tmp = point_missed_counter;
				point_missed_counter = 0; // reset counter
				// point missed, not last point -> next point
				if (point_index < ( points_in_path - 1 )) {
					// next point with new direction: set flag and wait a few samples before transmitting new car speed
					if (path_to_goal[point_index+1].car_speed == 0.0 || ( car_speed.f32_value > 0 && path_to_goal[point_index+1].car_speed < 0 ) || ( car_speed.f32_value < 0 && path_to_goal[point_index+1].car_speed > 0 ) ) {
						wait_samples = tTrue;
						sample_counter = 0;
						// debug message
						if(m_bDebugModeEnabled) {
							LOG_WARNING(cString::Format("Follow Path [Rescue mode]: change in car direction or car speed 0 at next path point, waiting few samples"));
						}
					}
					point_index++; // set next point
					direction = 0; // reset direction
					SetUSSensor(path_to_goal[point_index].us_sensor); 	// Set US mode info
					//LOG_WARNING(cString::Format("Follow Path [Rescue mode]: Change in alpha value, point missed, old alpha: %f, new: %f, set next point with index: %d", last_alpha, alpha, point_index));
					LOG_WARNING(cString::Format("Follow Path [Rescue mode]: Missed point %d, rho increased %d times, rho: %f, set next point: %d", point_index-1, missed_counter_tmp, rho, point_index));
					last_alpha = 0;
					RETURN_NOERROR; // stop this run, new calculations next time
				// last point missed
				} else if (point_index == ( points_in_path - 1 )) {
					// -> set rescue point if available
					if (last_is_rescue_point) {
						point_index++; // set rescue point
						points_in_path++;
						direction = 0; // reset direction
						last_is_rescue_point = tFalse;
						SetUSSensor(path_to_goal[point_index].us_sensor); 	// Set US mode info
						//LOG_WARNING(cString::Format("Follow Path [Rescue mode]: Change in alpha value, last point missed, old alpha: %f, new: %f, set last rescue point with index: %d", last_alpha, alpha, point_index));
						LOG_WARNING(cString::Format("Follow Path [Rescue mode]: Missed last point, id: %d, rho increased %d times, rho: %f, set rescue point: %d", point_index-1, missed_counter_tmp, rho, point_index));
						last_alpha = 0;
						RETURN_NOERROR; // stop this run, new calculations next time
					// -> stop if no more points available
					} else {
						//LOG_ERROR(cString::Format("Follow Path [Rescue mode]: Change in alpha value, last point missed and no more points, stopped, old alpha: %f, new: %f", last_alpha, alpha));
						//car_speed.f32_value = 0.0; // stop car
						TFeedbackStruct::Data feedback;
						feedback.ui32_filterID = F_FOLLOW_PATH;
						if(fabsf(car_pose_tmp.f32_yaw - path_to_goal[point_index].yaw) > m_f32maxGoalYawDeviation) {
							feedback.ui32_status = GetCommand()+2; // Feedback: BADYAW, TODO check feedback
							if(m_bDebugModeEnabled) {
								LOG_WARNING(cString::Format("Follow Path [Rescue mode]: end point reached, but large deviation between Yaw agles; set Yaw: %f, actual Yaw: %f, deviation: %f", path_to_goal[point_index].yaw, car_pose_tmp.f32_yaw, fabsf(car_pose_tmp.f32_yaw - path_to_goal[point_index].yaw)));
							}
						} else {
							feedback.ui32_status = GetCommand()+1; // Normal Feedback, TODO check feedback
						}
						feedbackOut.Transmit(&feedack_output, feedback, _clock->GetStreamTime());
						LOG_ERROR(cString::Format("Follow Path [Rescue mode]: Missed last (rescue) point, id: %d -> STOP, rho increased %d times, rho: %f", point_index, missed_counter_tmp, rho));
						SetRunningState(tFalse);
					}
				}
			}
		}

		last_alpha = alpha; // save last distance alpha for rescue check
		last_rho = rho; // save last distance to goal point for rescue check

		// calculate outputs
		gamma = direction * (kbeta * beta_ + kalpha * alpha); // gamma
		//car_speed.f32_value = direction * ( krho * rho); // car speed set from XML

		// calculate steering angle in deg (from 60 to 120)
		steering_angle.f32_value = -( gamma / 3.1415 ) * 180;  // invert steering angle, convert to deg

		// steering angle limits
		if (steering_angle.f32_value > 30) {
			steering_angle.f32_value = 30;
		} else if (steering_angle.f32_value < -30) {
			steering_angle.f32_value = -30;
		}
		steering_angle.f32_value += 90;

		// log all values to file
		#ifdef DEBUG_TO_FILE
			fprintf(LogFile, "%d;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d;%f;%f;%f;%f;%f;%f;%f;%f\n", point_index,  GetCarPose().f32_x,  GetCarPose().f32_y, GetCarPose().f32_yaw, car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw, path_to_goal[point_index].x, path_to_goal[point_index].y, path_to_goal[point_index].yaw, car_speed.f32_value, steering_angle.f32_value, GetUSSensor(), GetUSSensorValue(GetUSSensor()), alpha, beta, beta_, rho, delta_x, delta_y, gamma);
		#endif

		// Debug all values
		if(m_bDebugModeAllPoints) {
			LOG_WARNING(cString::Format("FollowPath: car pose absolute x: %f, y: %f, yaw: %f, car pose rel x: %f, y: %f, yaw: %f, goal: x_abs: %f, y_abs: %f, yaw_abs: %f, angle: %f, speed: %f, rho: %f, alpha: %f, gamma %f, delta_x %f, delta_y %f", GetCarPose().f32_x,  GetCarPose().f32_y, GetCarPose().f32_yaw, car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw, path_to_goal[point_index].x, path_to_goal[point_index].y, path_to_goal[point_index].yaw, steering_angle.f32_value, car_speed.f32_value, rho, alpha, gamma, delta_x, delta_y));
		}
	}

    // check car speed direction // TODO remove?
    if ( (direction > 0 && car_speed.f32_value < 0) || (direction < 0 && car_speed.f32_value > 0)) {
    	LOG_ERROR(cString::Format("FollowPath: error in car speed direction, XML speed %f, calculated speed sign: %f", car_speed.f32_value, direction));
    	car_speed.f32_value = 0.0;
    	SetRunningState (tFalse); // reset running state
    }

	// create new media sample for car speed output
	// if direction change in car speed, send 60 samples (1.5 s) with car speed 0.0
	if(wait_samples && sample_counter < 60) {
		car_speed.f32_value = 0.0;
		sample_counter++;
	} else {
		wait_samples = tFalse;
	}
	speedOut.Transmit(&car_speed_output, car_speed, _clock->GetStreamTime());

	if(send_steering_angle) {
		// transmit steering angle (if not in running state: transmit 90)
		steeringOut.Transmit(&steering_angle_output, steering_angle, _clock->GetStreamTime());
	}

	RETURN_NOERROR;
}


tResult FollowPath::LoadPathPointData()
{
	__synchronized_obj(m_oCriticalSectionPathAccess);

	//Get path of configuration file
	m_filePathPoints = GetPropertyStr("FollowPath::Configuration file for path points");

	// check if file exits
	if (m_filePathPoints.IsEmpty()) {
		LOG_ERROR("FollowPath: XML file not found!");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(m_filePathPoints);
	m_filePathPoints = m_filePathPoints.CreateAbsolutePath(".");

	// Load file, parse configuration, print the data

	if (cFileSystem::Exists(m_filePathPoints))
	{
		cDOM oDOM;
		oDOM.Load(m_filePathPoints);

		// load points
		cDOMElementRefList paths;
		cDOMElementRefList points;

		// clear path
		xml_paths.clear();

		if(IS_OK(oDOM.FindNodes("calibration/Path", paths))) {
			for (cDOMElementRefList::iterator path_element = paths.begin(); path_element != paths.end(); ++path_element)
			{
				// print path ID
				// LOG_WARNING(cString::Format("id: %d", (*path_element)->GetAttributeUInt32("id")));
				fp_paths new_path;
				new_path.path_id = (*path_element)->GetAttributeUInt32("id");
				new_path.reset_pose = (*path_element)->GetAttributeBool("reset_pose");
				new_path.rescue_point = (*path_element)->GetAttributeBool("rescue_point");
				new_path.steering = (*path_element)->GetAttributeBool("steering");

				// path elements
				if(IS_OK((*path_element)->FindNodes("point", points)))	{

					// points in path
					tFloat32 last_car_speed = 0;
					for (cDOMElementRefList::iterator path_element = points.begin(); path_element != points.end(); ++path_element)
					{
						// points
						fp_points new_points;
						cDOMElement* pConfigElement;

						if (IS_OK((*path_element)->FindNode("xValue", pConfigElement))) {
							new_points.x = tFloat32(cString(pConfigElement->GetData()).AsFloat64());
						}
						if (IS_OK((*path_element)->FindNode("yValue", pConfigElement))) {
							new_points.y = tFloat32(cString(pConfigElement->GetData()).AsFloat64());
						}
						if (IS_OK((*path_element)->FindNode("yawValue", pConfigElement))) {
							new_points.yaw = tFloat32(cString(pConfigElement->GetData()).AsFloat64());
						}
						if (IS_OK((*path_element)->FindNode("speedValue", pConfigElement))) {
							last_car_speed = tFloat32(cString(pConfigElement->GetData()).AsFloat64());
							new_points.car_speed = last_car_speed;
						} else {
							new_points.car_speed = last_car_speed;
							//LOG_WARNING("Follow path: no content");
						}
						if (IS_OK((*path_element)->FindNode("USSensor", pConfigElement))) {
							new_points.us_sensor = tInt32(cString(pConfigElement->GetData()).AsInt32());
						}
						if (IS_OK((*path_element)->FindNode("minimalDistance", pConfigElement))) {
							new_points.min_distance = tFloat32(cString(pConfigElement->GetData()).AsFloat64());
						}
						// push points
						new_path.points.push_back(new_points);
					}
				}
				// push path
				xml_paths.push_back(new_path);
			}
		}
		//LOG_WARNING(cString::Format("Size of paths: %d ", xml_paths.size()));
		//LOG_WARNING(cString::Format("Size of path points: %d ", xml_paths[0].points.size() ) );

		if (paths.size() > 0) {
			if (GetPropertyBool("Debugging::Print initial table to Console")) {
				for (tUInt i = 0; i < xml_paths.size(); i++) {
					LOG_WARNING(cString::Format("FollowPath: New Path, index: %d, ID: %d, Reset pose: %d, Last point is rescue point: %d, steering: %d", i, xml_paths[i].path_id, xml_paths[i].reset_pose, xml_paths[i].rescue_point, xml_paths[i].steering));
					for (tUInt j = 0; j < xml_paths[i].points.size(); j++) {
						LOG_WARNING(cString::Format("  FollowPath: Point index: %d, x: %f, y: %f, yaw: %f, speed: %f, USSensor: %d, minDistance: %f", j, xml_paths[i].points[j].x, xml_paths[i].points[j].y, xml_paths[i].points[j].yaw, xml_paths[i].points[j].car_speed, xml_paths[i].points[j].us_sensor, xml_paths[i].points[j].min_distance));
					}
				}
			}
		} else {
			LOG_ERROR("FollowPath: no path points in given file found!");
			RETURN_ERROR(ERR_INVALID_FILE);
		}
		//checks if data are valid // TODO check?
		//RETURN_IF_FAILED(CheckConfigurationData());

	} else {
		LOG_ERROR("FollowPath: Path point file not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	RETURN_NOERROR;
}

tResult FollowPath::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {

	// so we received a media sample, so this pointer better be valid.
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is (and if output exists?)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {

		// received pose input, save sample
		if (pSource == &pose_input) {
			ProcessPoseInput(pMediaSample);

			// calculate and transmit goal point
			if ( GetRunningState() ) {
				if(GetSteeringState()) {
					CalculateTransmitGoal();
				} else {
					CalculateTransmitGoalNoSteering();
				}
			}

		// received external goal point input, save sample
		} else if (pSource == &goal_input) {
			ProcessGoalPointInput(pMediaSample);

		// process external (debug) enable/start input
		} else if (pSource == &enable_input) {
			ProcessEnableInput(pMediaSample);

		// process state machine action input
		} else if (pSource == &action_input) {
			ProcessActionInput(pMediaSample);

		// process US input when sensor value is needed (US mode active)
		} else if (pSource == &US_input) {
			if(GetUSSensor() > 0) {
				ProcessUltrasonicInput(pMediaSample);
			}
		}

	}

	RETURN_NOERROR;
}
