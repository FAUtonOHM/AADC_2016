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
 XML file parsing based on cCalibrationXml filter from Audi
**********************************************************************
* $Author::  $ fink  $Date:: 2016-01-25 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/


#ifndef _FOLLOW_PATH_H_
#define _FOLLOW_PATH_H_

// dump all path values
//#define DEBUG_TO_FILE TRUE // dump to /tmp/logfile_followpath.txt
//#define DEBUG_POSE_TO_FILE TRUE // dump to /tmp/car_pose.dat (plot with gnuplot)

#define OID_ADTF_FOLLOW_PATH "adtf.user.follow_path"

#include "TSignalValue.h"
#include "TBoolSignalValue.h"
#include "TPoseStruct.h"
#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "TUltrasonicStruct.h"

//*************************************************************************************************
class FollowPath: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_FOLLOW_PATH, "Follow_Path", OBJCAT_DataFilter, "Follow Path", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin pose_input;
	cInputPin enable_input;
	cInputPin action_input;
	cInputPin goal_input;
	cInputPin US_input;

	cOutputPin next_goal_output;
	cOutputPin car_speed_output;
	cOutputPin steering_angle_output;
	cOutputPin feedack_output;

	// inputs
	TPoseStruct carPose;
	TBoolSignalValue enableFilter;
	TActionStruct PathAction;
	TPoseStruct ExternalGoal;
	TUltrasonicStruct USSensors;

	// outputs
	TPoseStruct NextGoalPose;
	TSignalValue speedOut;
	TSignalValue steeringOut;
	TFeedbackStruct feedbackOut;

public:
	FollowPath(const tChar* __info);
	virtual ~FollowPath();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:

	// debug path following to file
	#ifdef DEBUG_TO_FILE
		FILE * LogFile;
	#endif

	// debug complete car pose to file
	#ifdef DEBUG_POSE_TO_FILE
		tUInt32 counter_posesamples;
	#endif

	// properties
	tFloat32 m_f32SteeringAngleCorrection; // Correction factor for steering angle on output pin (not used)
	tFloat32 m_f32DistanceToNextPathPoint; // Distance to next path point in m
	tFloat32 m_f32USDistanceToNextPathPoint; // Ultrasonic condition: Distance to next path point in m (when US mode is active)
	tFloat32 m_f32DistanceToLastPathPoint; // Distance to last path point in m (not used)
	tBool m_bSendGoalCoordinates; // Activate output (goal_out) of goal point (relative to car)(not used)
	tFloat32 m_f32ReduceSpeedLastPointDistance; // Reduce car speed when car is [param] meters away from last path point
	tBool m_bDebugModeEnabled; // enable debug mode
	tBool m_bDebugModeAllPoints; // enable extended debug mode (print all points)
	tBool m_bPrintInitialXMLTable; // Print initial table to Console
	tFloat32 kalpha;
	tFloat32 kbeta;
	tFloat32 krho_forw; // Car control factor krho forwards
	tFloat32 krho_back; // Car control factor krho backwards
	tFloat32 reduction_offset; // Speed Reduction Offset in m/s
	tUInt32 debug_path_select; // debug mode path select (with property)
	tUInt32 missed_points_counter; // points missed counter (rescue mode)
	tFloat32 m_f32CarSpeedXYMode; // car speed when in external goal mode
	tFloat32 m_f32US_error_distance; // Ultrasonic condition: Error Distance (US value < [param] -> Error)
	tFloat32 m_f32maxGoalYawDeviation; // max. permitted yaw deviation between actual goal yaw and set goal yaw
	tBool m_bEnableRescue; // Enable Rescue Mode

	typedef struct fp_points_tmp {
		tFloat32 x;
		tFloat32 y;
		tFloat32 yaw;
		tFloat32 car_speed;
		tInt32 us_sensor;
		tFloat32 min_distance;

		fp_points_tmp() :
			x(0), y(0.0), yaw(0.0), car_speed(0.0), us_sensor(0), min_distance(0.0) {}
	} fp_points;

	typedef struct fp_paths_tmp {
		tUInt32 path_id;
		tBool reset_pose;
		tBool rescue_point;
		tBool steering;
	    std::vector<fp_points> points;

	    fp_paths_tmp() :
	    	path_id(0), reset_pose(tTrue), rescue_point(tFalse), steering(tTrue) {}
	} fp_paths;

	std::vector<fp_paths> xml_paths; // read points from xml file
	std::vector<fp_points> path_to_goal; // set path points

	// Pose x, y and yaw (received from input pin)
	TPoseStruct::Data car_pose_last;

	// pose offset
	TPoseStruct::Data car_pose_offset;

	// external goal point
	TPoseStruct::Data external_goal;

	// boolean start input received (debugging)
	tBool boolean_start;

	// Ultrasonic Input sample
	TUltrasonicStruct::Data ultrasonic_last;

	// flags and values
	tBool running; // running state
	tBool wait_samples; // change in car direction, wait flag
	tBool reset_pose; // reset pose on startup
	tBool last_is_rescue_point; // last point is rescue point
	tBool activate_steering_output; // False: no steering output, drive path [xValue] meters overall
	tUInt32 sample_counter; // count received samples
	tUInt32 point_index;	// path point index
	tUInt32 received_command; // received action command
	tFloat32 direction;	// car direction: 1 forwards, -1 backwards
	tUInt32 last_completed_path; // save last completed path for state machine request
	tUInt32 points_in_path; // number of points in path
	tFloat32 last_alpha; // save last alpha
	tFloat32 last_rho; // save last rho
	tInt32 US_sensor_select; // selected US sensor for parking mode
	tUInt32 point_missed_counter; // counts rho values greater than last rho

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

	// functions for processing and transmitting
	tResult ProcessPoseInput (IMediaSample*);
	tResult ProcessGoalPointInput (IMediaSample*);
	tResult ProcessEnableInput (IMediaSample*);
	tResult ProcessActionInput (IMediaSample*);
	tResult ProcessUltrasonicInput (IMediaSample*);
	TPoseStruct::Data GetCarPoseOffset();
	TPoseStruct::Data GetCarPose();
	TPoseStruct::Data GetExternalGoal();
	tFloat32 GetUSSensorValue(tInt32);
	tBool GetRunningState();
	tBool GetSteeringState();
	tResult SetRunningState(tBool);
	tUInt32 GetCommand();
	tResult SetCommand(tUInt32);
	tInt32 GetUSSensor();
	tResult SetUSSensor(tInt32);
	tResult CalculateTransmitGoalNoSteering();
	tResult CalculateTransmitGoal();
	tResult SetPath();

	// ***************************
	// Read XML path points

    /*! reads the xml file which is set in the filter properties */
    tResult LoadPathPointData();
    /*! holds the XML file for the supporting points*/
    cFilename m_filePathPoints;

	/*** critical sections ****/

	/*! the critical section of accessing the car pose data */
	cCriticalSection m_oCriticalSectionPoseAccess;

	/*! the critical section of accessing the external goal pose data */
	cCriticalSection m_oCriticalSectionGoalAccess;

	/* the critical section of accessing the trigger input */
	cCriticalSection m_oCriticalSectionTriggerOffsetAccess;

	/* the critical section of accessing the action input */
	cCriticalSection m_oCriticalSectionActionAccess;

	/* the critical section of accessing the running state */
	cCriticalSection m_oCriticalSectionRunningStateAccess;

    /*! the critical section of transmitting goal points */
    cCriticalSection m_oCriticalSectionGoalTransmit;

    /*! the critical section of accessing received command */
    cCriticalSection m_oCriticalSectionCommandAccess;

    /*! the critical section of accessing path points */
    cCriticalSection m_oCriticalSectionPathAccess;

    /*! the critical section of accessing US sensor choice */
    cCriticalSection m_oCriticalSectionUSSelectAccess;

    /*! the critical section of accessing US media sample data */
    cCriticalSection m_oCriticalSectionUSAccess;

    /*! the critical section of accessing US media sample data */
    cCriticalSection m_oCriticalSteeringStateAccess;

};

//*************************************************************************************************
#endif // _FOLLOW_PATH_H_
