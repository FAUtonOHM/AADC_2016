/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-03-11 #$ $Rev:: 0.2.0   $
 **********************************************************************/

#include "PoseCache.h"
#include "LineSpecifier.h"
#include "cache/DetectionCache.h"
#include "speed/SpeedRecommender.h"
#include "stop/StopLineDetector.h"
#include "parking/ParkingSpotDetector.h"
#include "crossing/IntersectionDetector.h"

#include "LineSpecifierAdapter.h"

#include "ImageProcessingUtils.h"

#include "ScmCommunication.h"
#include "roadSign_enums.h"

#include <iostream>

#define LS_STOPLINE_OFFSET_X "stopline::offset_x"
#define LS_STOPLINE_OFFSET_Y "stopline::offset_y"
#define LS_STOPLINE_MIN_WIDTH "stopline::min width"
#define LS_STOPLINE_MAX_WIDTH "stopline::max width"
#define LS_STOPLINE_MAX_Y_X_RATIO "stopline::max_y_x ratio"
#define LS_STOPLINE_YAW_SCALE "stopline::yaw scale"
#define LS_STOPLINE_TRANSMISSION "stopline::transmission_distance"

#define LS_HALT_EMERGENCY_DISTANCE "halt::emergency_distance"
#define LS_STEERING_OFFSET_X "steering::offset x"
#define LS_STEERING_OFFSET_Y "steering::offset_y"
#define LS_STEERING_ANGLE_SCALE_LEFT "steering::angle scale left"
#define LS_STEERING_ANGLE_SCALE_RIGHT "steering::angle scale right"

#define LS_PARKING_TRANS_OFFSET_X "parkingTrans::offset_x"
#define LS_PARKING_TRANS_OFFSET_Y "parkingTrans::offset_y"

#define LS_PULL_OUT_PARKING_TRANS_OFFSET_X "pullOutParkingTrans::offset_x"

#define LS_PARKING_LONG_OFFSET_X "parkingLong::offset_x"
#define LS_PARKING_LONG_OFFSET_Y "parkingLong::offset_y"

#define LS_BACKWARDS_OFFSET_X "backwards::offset_x"
#define LS_BACKWARDS_OFFSET_Y "backwards::offset_y"

#define LS_SPEED_STRAIGHT_SLOW_FAR "speed::straight far slow"
#define LS_SPEED_STRAIGHT_SLOW_NEAR "speed::straight near_slow"
#define LS_SPEED_STRAIGHT_NORMAL_FAR "speed::straight far normal"
#define LS_SPEED_STRAIGHT_NORMAL_NEAR "speed::straight near_normal"
#define LS_SPEED_MIN "speed::min"
#define LS_SPEED_CURVE "speed::curve"
#define LS_SPEED_DETECTION "speed::detection"

#define LS_START_COMMAND "Debug::start action command"
#define LS_SAVE_TO_DISK "Debug::save to disk"

#define LOCAL_IMAGE_COPY 0
#define FEATURE_TESTING 1

ADTF_FILTER_PLUGIN("LineSpecifier", OID_ADTF_LINE_SPECIFIER, LineSpecifierAdapter);

LineSpecifierAdapter::LineSpecifierAdapter(const tChar* __info) :
		cAsyncDataTriggeredFilter(__info) {

	SetPropertyFloat(LS_STOPLINE_OFFSET_X, -0.60);
	SetPropertyFloat(LS_STOPLINE_OFFSET_X NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_STOPLINE_OFFSET_Y, 0);
	SetPropertyFloat(LS_STOPLINE_OFFSET_Y NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STOPLINE_MAX_Y_X_RATIO, 1);
	SetPropertyFloat(LS_STOPLINE_MAX_Y_X_RATIO NSSUBPROP_MAX, 1);
	SetPropertyFloat(LS_STOPLINE_MAX_Y_X_RATIO NSSUBPROP_MIN, 0);
	SetPropertyFloat(LS_STOPLINE_MAX_Y_X_RATIO NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STOPLINE_YAW_SCALE, 0.7);
	SetPropertyFloat(LS_STOPLINE_YAW_SCALE NSSUBPROP_MAX, 2);
	SetPropertyFloat(LS_STOPLINE_YAW_SCALE NSSUBPROP_MIN, 0);
	SetPropertyFloat(LS_STOPLINE_YAW_SCALE NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STOPLINE_MIN_WIDTH, 0.025);
	SetPropertyFloat(LS_STOPLINE_MIN_WIDTH NSSUBPROP_MAX, 0.04);
	SetPropertyFloat(LS_STOPLINE_MIN_WIDTH NSSUBPROP_MIN, 0.02);
	SetPropertyFloat(LS_STOPLINE_MIN_WIDTH NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STOPLINE_MAX_WIDTH, 0.076);
	SetPropertyFloat(LS_STOPLINE_MAX_WIDTH NSSUBPROP_MAX, 0.086);
	SetPropertyFloat(LS_STOPLINE_MAX_WIDTH NSSUBPROP_MIN, 0.04);
	SetPropertyFloat(LS_STOPLINE_MAX_WIDTH NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STOPLINE_TRANSMISSION, 1);
	SetPropertyFloat(LS_STOPLINE_TRANSMISSION NSSUBPROP_MAX, 2);
	SetPropertyFloat(LS_STOPLINE_TRANSMISSION NSSUBPROP_MIN, 0);
	SetPropertyFloat(LS_STOPLINE_TRANSMISSION NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LS_STOPLINE_TRANSMISSION NSSUBPROP_DESCRIPTION,"if the detected stop position is nearer than the given distance, the position is transmitted on localGoal");

	SetPropertyFloat(LS_HALT_EMERGENCY_DISTANCE, 1.33);
	SetPropertyFloat(LS_HALT_EMERGENCY_DISTANCE NSSUBPROP_MAX, 3);
	SetPropertyFloat(LS_HALT_EMERGENCY_DISTANCE NSSUBPROP_MIN, 0);
	SetPropertyFloat(LS_HALT_EMERGENCY_DISTANCE NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_PARKING_TRANS_OFFSET_X, -0.1);
	SetPropertyFloat(LS_PARKING_TRANS_OFFSET_X NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_PARKING_TRANS_OFFSET_Y, 0.25);
	SetPropertyFloat(LS_PARKING_TRANS_OFFSET_Y NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_PULL_OUT_PARKING_TRANS_OFFSET_X, -1.1);
	SetPropertyFloat(LS_PULL_OUT_PARKING_TRANS_OFFSET_X NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_PARKING_LONG_OFFSET_X, -0.1);
	SetPropertyFloat(LS_PARKING_LONG_OFFSET_X NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_PARKING_LONG_OFFSET_Y, 0.25);
	SetPropertyFloat(LS_PARKING_LONG_OFFSET_Y NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_BACKWARDS_OFFSET_X, -1.0);
	SetPropertyFloat(LS_BACKWARDS_OFFSET_X NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_BACKWARDS_OFFSET_Y, 0);
	SetPropertyFloat(LS_BACKWARDS_OFFSET_Y NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STEERING_OFFSET_X, 0.35);
	SetPropertyFloat(LS_STEERING_OFFSET_X NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_STEERING_OFFSET_Y, 0);
	SetPropertyFloat(LS_STEERING_OFFSET_Y NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STEERING_ANGLE_SCALE_RIGHT, 1);
	SetPropertyFloat(LS_STEERING_ANGLE_SCALE_RIGHT NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_STEERING_ANGLE_SCALE_LEFT, 1);
	SetPropertyFloat(LS_STEERING_ANGLE_SCALE_LEFT NSSUBPROP_REQUIRED, tTrue);

	SetPropertyFloat(LS_SPEED_DETECTION, 0.6);
	SetPropertyFloat(LS_SPEED_DETECTION NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_DETECTION NSSUBPROP_MAX, 2);
	SetPropertyFloat(LS_SPEED_DETECTION NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_NEAR, 0.8);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_NEAR NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_NEAR NSSUBPROP_MAX, 4);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_NEAR NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_FAR, 1.1);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_FAR NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_FAR NSSUBPROP_MAX, 4);
	SetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_FAR NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_NEAR, 0.8);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_NEAR NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_NEAR NSSUBPROP_MAX, 4);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_NEAR NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_FAR, 1.1);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_FAR NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_FAR NSSUBPROP_MAX, 4);
	SetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_FAR NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_MIN, 0.5);
	SetPropertyFloat(LS_SPEED_MIN NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_MIN NSSUBPROP_MAX, 1);
	SetPropertyFloat(LS_SPEED_MIN NSSUBPROP_MIN, 0);

	SetPropertyFloat(LS_SPEED_CURVE, 0.7);
	SetPropertyFloat(LS_SPEED_CURVE NSSUBPROP_REQUIRED, tTrue);
	SetPropertyFloat(LS_SPEED_CURVE NSSUBPROP_MAX, 4);
	SetPropertyFloat(LS_SPEED_CURVE NSSUBPROP_MIN, 0);


	SetPropertyInt(LS_START_COMMAND, 0);
	SetPropertyFloat(LS_START_COMMAND NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LS_START_COMMAND NSSUBPROP_DESCRIPTION,"action command which is loaded on startup");

	SetPropertyBool(LS_SAVE_TO_DISK, tFalse);
	SetPropertyFloat(LS_SAVE_TO_DISK NSSUBPROP_REQUIRED, tTrue);
}

LineSpecifierAdapter::~LineSpecifierAdapter() {

}

tResult LineSpecifierAdapter::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst) {
		imageNumber = 0;

		debugEnabled = tFalse;

		RETURN_IF_FAILED(tActionStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSpeed.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSteering.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueNull.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tPoseStructGoal.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tPoseStructIn.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tTrafficSign.StageFirst(__exception_ptr));

		RETURN_IF_FAILED(actionInput.Create("action", tActionStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&actionInput));

		RETURN_IF_FAILED(trafficSignInput.Create("sign", tTrafficSign.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&trafficSignInput));

		RETURN_IF_FAILED(poseInput.Create("car_pose", tPoseStructIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&poseInput));

		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

		// Video Input
		RETURN_IF_FAILED(inputVideoPin.Create("Binary_Image", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&inputVideoPin));

		RETURN_IF_FAILED(videoDebugOutputPin.Create("RGB_DebugVideo", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoDebugOutputPin));
	} else if (eStage == StageNormal) {
		imageNumber = 0;
		sync.bufferCount = 0;
		sync.inputFormat.nSize = 0,
		lastVideoTime = 0;
		onLeftLane = tFalse;
		firstParkingDetection = tTrue;

		offset.steering = Point2f(GetPropertyFloat(LS_STEERING_OFFSET_X),GetPropertyFloat(LS_STEERING_OFFSET_Y));
		offset.stopline = Point2f(GetPropertyFloat(LS_STOPLINE_OFFSET_X),GetPropertyFloat(LS_STOPLINE_OFFSET_Y));
		offset.yawScale = GetPropertyFloat(LS_STOPLINE_YAW_SCALE);
		offset.maxXYRation_Stopline = GetPropertyFloat(LS_STOPLINE_MAX_Y_X_RATIO);
		offset.parkingTrans = Point2f(GetPropertyFloat(LS_PARKING_TRANS_OFFSET_X),GetPropertyFloat(LS_PARKING_TRANS_OFFSET_Y));
		offset.parkingLong = Point2f(GetPropertyFloat(LS_PARKING_LONG_OFFSET_X),GetPropertyFloat(LS_PARKING_LONG_OFFSET_Y));
		offset.pullOutXOffset = GetPropertyFloat(LS_PULL_OUT_PARKING_TRANS_OFFSET_X);
		offset.backwards = Point2f(GetPropertyFloat(LS_BACKWARDS_OFFSET_X),GetPropertyFloat(LS_BACKWARDS_OFFSET_Y));
		offset.stopTransmissionDistance = GetPropertyFloat(LS_STOPLINE_TRANSMISSION);
		steeringAngleScaleLeft = GetPropertyFloat(LS_STEERING_ANGLE_SCALE_LEFT);
		steeringAngleScaleRight = GetPropertyFloat(LS_STEERING_ANGLE_SCALE_RIGHT);

		speedController.speedConfig.straightSlowNear = GetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_NEAR);
		speedController.speedConfig.straightNormalNear = GetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_NEAR);
		speedController.speedConfig.straightSlowFar = GetPropertyFloat(LS_SPEED_STRAIGHT_SLOW_FAR);
		speedController.speedConfig.straightNormalFar = GetPropertyFloat(LS_SPEED_STRAIGHT_NORMAL_FAR);
		speedController.speedConfig.min = GetPropertyFloat(LS_SPEED_MIN);
		speedController.speedConfig.curve = GetPropertyFloat(LS_SPEED_CURVE);
		speedController.speedConfig.detection = GetPropertyFloat(LS_SPEED_DETECTION);
		saveToHardDisk = GetPropertyBool(LS_SAVE_TO_DISK);
		debugEnabled = saveToHardDisk;

		lineSpecifier = new LineSpecifier();
		stopLineDetector = new StopLineDetector(METER_TO_PIXEL(GetPropertyFloat(LS_STOPLINE_MIN_WIDTH)), METER_TO_PIXEL(GetPropertyFloat(LS_STOPLINE_MAX_WIDTH)));
		parkingDetector = new ParkingSpotDetector();
		intersectionDetector = new IntersectionDetector(METER_TO_PIXEL(GetPropertyFloat(LS_HALT_EMERGENCY_DISTANCE)));
		carPoses = new PoseCache();

		try {
			lineSpecifier->Init();
		} catch (cv::Exception &e) {
			LOG_ERROR(e.msg.c_str());
		}

		//writer = new VideoWriter ("/tmp/test.avi", VideoWriter::fourcc('H','2','6','4'), 30.0, Size(BINARY_IMAGE_WIDTH, BINARY_DEBUG_IMAGE_HEIGHT), true);
		//writer->set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	}
	else if (eStage == StageGraphReady)
	{
		RETURN_IF_FAILED(tActionStruct.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSpeed.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSteering.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueNull.StageGraphReady());
		RETURN_IF_FAILED(tPoseStructGoal.StageGraphReady());
		RETURN_IF_FAILED(tPoseStructIn.StageGraphReady());
		RETURN_IF_FAILED(tTrafficSign.StageGraphReady());

        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(inputVideoPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        const tBitmapFormat* format = inputVideoPin.GetFormat();

		if (format != NULL) {
			sync.inputFormat = (*format);
			LOG_WARNING(cString::Format(
							"LineSpecifier: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
							sync.inputFormat.nWidth, sync.inputFormat.nHeight, sync.inputFormat.nBytesPerLine, sync.inputFormat.nSize,
							sync.inputFormat.nPixelFormat));
		} else {
			RETURN_AND_LOG_ERROR_STR(ERR_FAILED, "LineSpecfier: StageGraphReady: input format == 0")
		}
		if (sync.inputFormat.nWidth <= 0 || sync.inputFormat.nWidth > 1000 || sync.inputFormat.nHeight <= 0
				|| sync.inputFormat.nHeight > 1000) {
			RETURN_AND_LOG_ERROR_STR(ERR_FAILED, "LineSpecfier: StageGraphReady: input format not supported")
		}

#if LOCAL_IMAGE_COPY
		image.create(sync.inputFormat.nHeight, sync.inputFormat.nWidth, CV_8UC1);
#endif

		if(!debugEnabled) {
			debugEnabled = videoDebugOutputPin.IsConnected();
			if(debugEnabled) {
				LOG_WARNING("LineSpecifier: debug mode enabled because of connected RGB_DebugVideo pin");
			}
		}

		outputFormat.nWidth = BINARY_IMAGE_WIDTH;
		outputFormat.nHeight = BINARY_DEBUG_IMAGE_HEIGHT;
		outputFormat.nBitsPerPixel = 24;
		outputFormat.nBytesPerLine = BINARY_IMAGE_WIDTH * 3;
		outputFormat.nSize = outputFormat.nBytesPerLine * BINARY_DEBUG_IMAGE_HEIGHT;
		outputFormat.nPaletteSize = 0;
		outputFormat.nPixelFormat = cImage::PF_RGB_888;
		videoDebugOutputPin.SetFormat(&outputFormat, NULL);

		lineSpecifier->switchDebugMode(debugEnabled);
		stopLineDetector->switchDebugMode(debugEnabled);
		parkingDetector->switchDebugMode(debugEnabled);
		intersectionDetector->switchDebugMode(debugEnabled);
		speedController.switchDebugMode(debugEnabled);

		tInt32 startCommand = GetPropertyInt(LS_START_COMMAND);
		if ( startCommand >= 1000 && startCommand < 2000) {
			actionSub.enabled = tTrue;
			actionSub.started = tTrue;
			actionSub.command = startCommand;
		}
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageGraphReady) {
	} else if (eStage == StageNormal) {
		delete lineSpecifier;
		delete stopLineDetector;
		delete parkingDetector;
		delete intersectionDetector;
		delete carPoses;

	    //writer->release();

	    //delete writer;
	} else if (eStage == StageFirst) {
		debugEnabled = tFalse;
	}

	// call the base class implementation
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult LineSpecifierAdapter::CreateOutputPins(__exception) {
	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	// create output pin for stop line position
	RETURN_IF_FAILED(localGoalOutput.Create("localGoal", tPoseStructGoal.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&localGoalOutput));

	// create output pin for stop line position
	RETURN_IF_FAILED(speedOutput.Create("speed", tSignalValueSpeed.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&speedOutput));

	//Steering Angle Output Pin
	RETURN_IF_FAILED(steeringAngleOutput.Create("Steering_Angle", tSignalValueSteering.GetMediaType(), 0));
	RETURN_IF_FAILED(RegisterPin(&steeringAngleOutput));

	//Null Output Pin
	RETURN_IF_FAILED(nullOutput.Create("NULL", tSignalValueNull.GetMediaType(), 0));
	RETURN_IF_FAILED(RegisterPin(&nullOutput));

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &inputVideoPin) {
			{
				adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

				if(sync.bufferCount > 1) {
					sync.bufferCount = 0;
					ClearAsyncQueue();
					sync.PrintCleared();
				}

				sync.IncreaseBufferCount();
			}

			return cAsyncDataTriggeredFilter::OnPinEvent(source, nEventCode, nParam1, nParam2, mediaSample);
		} else if (source == &actionInput){
			adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

			sync.lastActionMediaSample = mediaSample;
		} else if (source == &poseInput){
			TPoseStruct::Data data;
			tPoseStructIn.Read(mediaSample, &data);

			CarPose p = CarPose(Point2f(data.f32_x, data.f32_y), data.f32_yaw, mediaSample->GetTime(), data.ui32_arduinoTimestamp);
			carPoses->PushFront(p);
		} else if (source == &trafficSignInput){
			adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

			tTrafficSign.Read(mediaSample, &sync.trafficSign);
			sync.checkCrossing = true;
		}
	} else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
		if (source == &inputVideoPin) {
			adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

			//the input format was changed, so the imageformat has to changed in this filter also
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(inputVideoPin.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			const tBitmapFormat* format = inputVideoPin.GetFormat();

			if (format != NULL) {
				{
					adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));
					sync.inputFormat = (*format);
				}
				LOG_WARNING(cString::Format(
								"LineSpecifier: PE_MediaTypeChanged: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
								sync.inputFormat.nWidth, sync.inputFormat.nHeight, sync.inputFormat.nBytesPerLine, sync.inputFormat.nSize,
								sync.inputFormat.nPixelFormat));
			}


#if LOCAL_IMAGE_COPY
			image.create(sync.inputFormat.nHeight, sync.inputFormat.nWidth, CV_8UC1);
#endif

		}
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	sync.criticalSection_OnPinEvent.Enter();

	cObjectPtr<IMediaSample> actionMediaSample = sync.lastActionMediaSample;
	if(actionMediaSample != 0) {
		sync.lastActionMediaSample = 0;

		sync.criticalSection_OnPinEvent.Leave();
		ProcessAction(actionMediaSample);

		sync.criticalSection_OnPinEvent.Enter();
	}

	sync.criticalSection_OnPinEvent.Leave();
	if(lastVideoTime > 0) {
		tFloat32 lastDistance = carPoses->GetLastDistance();
		if(lastDistance > 0) {
			prioritySign.distance -= lastDistance;
		} else {
			prioritySign.distance -= (mediaSample->GetTime() - lastVideoTime) * 0.000001 * speedController.lastSpeed;
		}
		if(prioritySign.distance < 1) {
			//TODO: delete it
			prioritySign.distance = -1;
		}
	}
	sync.criticalSection_OnPinEvent.Enter();

	if(sync.checkCrossing) {
		prioritySign.priority = false;
		switch(sync.trafficSign.ui32_signID) {
		case MARKER_ID_HAVEWAY:
			prioritySign.priority = true;
		case MARKER_ID_UNMARKEDINTERSECTION:
		case MARKER_ID_STOPANDGIVEWAY:
		case MARKER_ID_GIVEWAY:
			if(prioritySign.distance < 0) {
				prioritySign.distance = sync.trafficSign.f32_distance;

				if(prioritySign.priority) {
					LOG_WARNING(cString::Format("LineSpec: New PrioritySign distance %f", prioritySign.distance));
				} else {
					LOG_WARNING(cString::Format("LineSpec: New GiveWaySign distance %f", prioritySign.distance));
				}
			} else {
				prioritySign.distance = 0.5*(prioritySign.distance + sync.trafficSign.f32_distance);

				if(prioritySign.priority) {
					LOG_WARNING(cString::Format("LineSpec: PrioritySign distance %f", prioritySign.distance));
				} else {
					LOG_WARNING(cString::Format("LineSpec: GiveWaySign distance %f", prioritySign.distance));
				}
			}


			break;
		default:
			prioritySign.distance = -1;
		}

		sync.checkCrossing = false;
	}

	sync.DecreaseBufferCount();
	sync.criticalSection_OnPinEvent.Leave();

	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		tSignalValueNull.Transmit(&nullOutput, 0, 0, mediaSample->GetTime());

		if (source == &inputVideoPin) {
			if(actionSub.enabled && actionSub.started) {
				RETURN_IF_FAILED(ProcessVideo(mediaSample));
			}
		}
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessAction(IMediaSample* mediaSample) {
	TActionStruct::ActionSub currentAction = tActionStruct.Read_Action(mediaSample, F_LINE_SPECIFIER);

	TFeedbackStruct::Data feedback;
	feedback.ui32_filterID = F_LINE_SPECIFIER;

	firstParkingDetection = tTrue;

	prioritySign.distance = -1;
	if(currentAction.enabled == tFalse || currentAction.started == tFalse) {
		actionSub.command = AC_LS_STOP;
		lineSpecifier->SetFirstPoint(tTrue);
		stopLineDetector->ClearDetectedStopPositions();
		stopLineDetector->SetTrackingMode(tFalse);
		intersectionDetector->ClearDetectedStopPositions();
		intersectionDetector->SetTrackingMode(tFalse);
	} else if(currentAction.command == AC_LS_STOP) {
		onLeftLane = tFalse;
		feedback.ui32_status = FB_LS_STOPPED;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if (currentAction.command == AC_LS_NOSPEED) {
		onLeftLane = tFalse;
		lastIntersection.yawIntegral = 0;
		feedback.ui32_status = FB_LS_NOSPEED;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if (currentAction.command == AC_LS_SLOW_RIGHTLANE) {
		onLeftLane = tFalse;
		lastIntersection.yawIntegral = 0;
		feedback.ui32_status = FB_LS_SLOW_RIGHTLANE;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if(currentAction.command == AC_LS_NORMAL_RIGHTLANE) {
		onLeftLane = tFalse;
		lastIntersection.yawIntegral = 0;
		feedback.ui32_status = FB_LS_NORMAL_RIGHTLANE;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if (currentAction.command == AC_LS_SLOW_LEFTLANE) {
		onLeftLane = tTrue;
		lastIntersection.yawIntegral = 0;
		feedback.ui32_status = FB_LS_SLOW_LEFTLANE;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if(currentAction.command == AC_LS_NORMAL_LEFTLANE) {
		onLeftLane = tTrue;
		lastIntersection.yawIntegral = 0;
		feedback.ui32_status = FB_LS_NORMAL_LEFTLANE;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	} else if(currentAction.command == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW) {
		if(actionSub.command == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW) {
			firstParkingDetection = tFalse;
		}
		lastIntersection.yawIntegral = 0;
	} else if(currentAction.command == AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW) {
		if(actionSub.command == AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW) {
			firstParkingDetection = tFalse;
		}
		lastIntersection.yawIntegral = 0;
	}

	actionSub = currentAction;

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessVideo(IMediaSample* mediaSample) {
	RETURN_IF_FAILED(ConvertInputImage(mediaSample));

	Point2f frameCoord;
	tFloat32 nearPlane;
	ReadAsFloatAndClear(image, 4, &frameCoord.x);
	ReadAsFloatAndClear(image, 8, &frameCoord.y);
	ReadAsFloatAndClear(image, 12, &nearPlane);

	if(onLeftLane) {
		cv::flip(image, image, 1);
	}

	Mat debugImage(1, 1, CV_8UC3);

	if (debugEnabled) {
		resize(debugImage, debugImage, Size(BINARY_IMAGE_WIDTH, BINARY_DEBUG_IMAGE_HEIGHT), CV_8UC3);
		Mat temp;
		cvtColor(image, temp, CV_GRAY2RGB);
		copyMakeBorder(temp, debugImage, 0, BINARY_DEBUG_IMAGE_HEIGHT - BINARY_IMAGE_HEIGHT, 0, 0, BORDER_CONSTANT);
	}

	LineSpecifier::HINT hint = LineSpecifier::NO_HINT;
	if (actionSub.command == AC_LS_STOP) {
		tFloat32 yawDiff = carPoses->GetYawDiff();
		lastIntersection.yawIntegral += yawDiff;

		Point2f localDestination = ImageUtils::ConvertToWorldCoordinates(lineSpecifier->GetLastDestination(), frameCoord);

		if(lastIntersection.yawIntegral < -0.10) {
			hint = LineSpecifier::RIGHT_TURN;
			//right turn;

			Point2f rightDir = intersectionDetector->OnRightTurn(image, &debugImage);
			if(rightDir.x != 0 && rightDir.y != 0) {
				lineSpecifier->SetLastDestination(rightDir, 0.5);
			} else if (localDestination.y > 0) {
				lineSpecifier->SetFirstPoint(true);
			}
		} else if(lastIntersection.yawIntegral > 0.10) {
			hint = LineSpecifier::LEFT_TURN;
			//left turn;

			Point2f leftDir = intersectionDetector->OnLeftTurn(image, &debugImage);
			if(leftDir.x != 0 && leftDir.y != 0) {
				lineSpecifier->SetLastDestination(leftDir, 0.9);
			} else if(localDestination.y < 0) {
				lineSpecifier->SetFirstPoint(true);
			}
		}

		if(yawDiff != 0 && cv::abs(yawDiff) < 0.08) {
			Point2f movement = Point2f(-yawDiff * (METER_TO_PIXEL(frameCoord.x) - BINARY_IMAGE_HEIGHT), 0);
			if(0 < movement.x && movement.x < 1) {
				movement.x = 1;
			}

			if(-1 < movement.x && movement.x < 0) {
				movement.x = -1;
			}

			lineSpecifier->MoveDestPoint(movement);
		}
	}

	Point2f image_destination;
	tFloat32 probability;
	lineSpecifier->DetermineDestinationPoint(image, debugImage, &image_destination, &probability, hint);
	Point2f destination = ImageUtils::ConvertToWorldCoordinates(image_destination, frameCoord);

	currentSteeringAngle = speedController.DetermineSteeringAngle(image, &debugImage, destination, offset.steering,steeringAngleScaleRight, steeringAngleScaleLeft, onLeftLane, mediaSample->GetTime());

	switch(actionSub.command) {
	case AC_LS_SLOW_LEFTLANE:
	case AC_LS_NORMAL_LEFTLANE:
	case AC_LS_SLOW_RIGHTLANE:
	case AC_LS_NORMAL_RIGHTLANE:
	case AC_LS_DETECT_LONG_PARKING_SPOT_SLOW:
	case AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW:
	{
		if(onLeftLane) {
			tSignalValueSteering.Transmit(&steeringAngleOutput, - currentSteeringAngle + 90, 0, mediaSample->GetTime());
		} else {
			tSignalValueSteering.Transmit(&steeringAngleOutput, currentSteeringAngle + 90, 0, mediaSample->GetTime());
		}

		speedController.DetectMaxSpeed(image, &debugImage, currentSteeringAngle, mediaSample->GetTime());

		tFloat32 speed = speedController.SpeedController(probability, prioritySign.distance, actionSub.command, currentSteeringAngle);
		tSignalValueSpeed.Transmit(&speedOutput, speed, 0, mediaSample->GetTime());
		break;
	}
	case AC_LS_NOSPEED:
		// TRANSMITTING Steering Angle
		if(onLeftLane) {
			tSignalValueSteering.Transmit(&steeringAngleOutput, - currentSteeringAngle + 90, 0, mediaSample->GetTime());
		} else {
			tSignalValueSteering.Transmit(&steeringAngleOutput, currentSteeringAngle + 90, 0, mediaSample->GetTime());
		}

		speedController.DetectMaxSpeed(image, &debugImage, currentSteeringAngle, mediaSample->GetTime());

		break;
	case AC_LS_NOSPEED_BACKWARDS:
		speedController.DetectMaxSpeed(image, &debugImage, currentSteeringAngle, mediaSample->GetTime());
		RETURN_IF_FAILED(ProcessBackwards(debugImage, frameCoord, image_destination, mediaSample->GetTime()));
		break;
	}


	switch(actionSub.command) {
	case AC_LS_SLOW_RIGHTLANE:
	case AC_LS_NORMAL_RIGHTLANE: {
		if(prioritySign.distance > 0 && prioritySign.priority) {
			RETURN_IF_FAILED(ProcessIntersection(debugImage, frameCoord, nearPlane, mediaSample->GetTime()));
		} else {
			RETURN_IF_FAILED(ProcessStoplineDetection(debugImage, frameCoord, image_destination, nearPlane, mediaSample->GetTime()));
		}
	} break;
	case AC_LS_DETECT_LONG_PARKING_SPOT_SLOW:
	case AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW:
	case AC_LS_DETECT_LONG_PARKING_SPOT_NEXT:
	case AC_LS_DETECT_TRANS_PARKING_SPOT_NEXT: {
		RETURN_IF_FAILED(ProcessParkingDetection(debugImage, frameCoord, mediaSample->GetTime()));
	} break;
	case AC_LS_GET_PARKSPOT_TYPE: {
		RETURN_IF_FAILED(ProcessPullOut(debugImage, frameCoord, mediaSample->GetTime()));
	} break;
	case AC_LS_CHECK_CROSSING_LINE: {
		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_LINE_SPECIFIER;
		if(intersectionDetector->DetectCrossingLine(image, &debugImage, mediaSample->GetTime())) {
			feedback.ui32_status = FB_LS_CROSSING_LINE_HORIZONTAL;
		} else if(intersectionDetector->DetectStraightLine(image, &debugImage)){
			feedback.ui32_status = FB_LS_CROSSING_LINE_VERTICAL;
		} else {
			feedback.ui32_status = FB_LS_CROSSING_LINE_NONE;
		}

		tFeedbackStruct.Transmit(&feedbackOutput, feedback, mediaSample->GetTime());
	}
	}

	if(saveToHardDisk) {
		Record(image, debugImage);
	}
	//writer->write(debugImage);

	lastVideoTime = mediaSample->GetTime();

	if (debugEnabled) {
		cObjectPtr<IMediaSample> sample;
		if (IS_OK(AllocMediaSample(&sample))) {
			sample->Update(mediaSample->GetTime(), debugImage.data, tInt32(outputFormat.nSize), 0);

			//videoDebugOutputPin.SetFormat(&outputFormat, NULL);
			videoDebugOutputPin.Transmit(sample);
		}
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessStoplineDetection(cv::Mat &debugImage, cv::Point2f frameCoord, cv::Point2f image_destination, tFloat32 nearPlane, tTimeStamp outputTime) {
	Point2f car_image_position = ImageUtils::ConvertToImageCoordinates(offset.steering, frameCoord);
	Point2f image_yawDirection;

	Point2f image_stopPosition = stopLineDetector->Detect(image, debugImage, car_image_position, image_destination, &image_yawDirection, outputTime);

	//Point2f image_stopPosition(0,0);
	if ((image_stopPosition.x > 0 && image_stopPosition.y > 0)) {
		stopLineDetector->SetTrackingMode(true);
		speedController.detectionModeActive = 5; // 5 frames
	}

	if(stopLineDetector->IsInTrackingMode()) {
		Point2f localStopPosition = carPoses->InterpolatePoses(stopLineDetector->GetDetectedStopPositions(), frameCoord);

		if(localStopPosition.x > offset.stopTransmissionDistance) {
			RETURN_NOERROR;
		}

		if(localStopPosition.x == 0 && localStopPosition.y == 0) {
			if(carPoses->IsEmpty()) {
				//fallback, if no car poses in the cache
				localStopPosition = ImageUtils::ConvertToWorldCoordinates(image_stopPosition, frameCoord);
			} else if (stopLineDetector->GetDetectedStopPositions().size() > 1){
				//TODO: fix bug
				LOG_WARNING("LineSpec: tracking failed");
				localStopPosition = stopLineDetector->GetDetectedStopPositions().front().pos;

				if(localStopPosition.x == 0 && localStopPosition.y == 0) {
					localStopPosition.x = offset.stopTransmissionDistance - 0.1;
				}
				image_yawDirection = Point2f(0, -1);
				RETURN_NOERROR;
			} else {
				RETURN_NOERROR;
			}
		}

		TPoseStruct::Data stop = DetermineWorldStopPoint(localStopPosition, image_yawDirection);

		if(stop.f32_x == 0 && stop.f32_y == 0) {
			RETURN_NOERROR;
		}

		stopLineDetector->SetTrackingMode(false);
		stopLineDetector->ClearDetectedStopPositions();

		if (debugEnabled) {
			char info[100];
			sprintf(info, "stop x %f y %f yaw %f", stop.f32_x, stop.f32_y, stop.f32_yaw);

			cv::putText(debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
		}
		LOG_WARNING(cString::Format("LineSpec: stop x %f y %f yaw %f", stop.f32_x, stop.f32_y, stop.f32_yaw));

		if(actionSub.command == AC_LS_SLOW_RIGHTLANE || actionSub.command == AC_LS_NORMAL_RIGHTLANE) {
			// TRANSMITTING stop Line
			prioritySign.distance = -1;
			tPoseStructGoal.Transmit(&localGoalOutput, stop, outputTime);

			// brake before loosing control
			if(stop.f32_x < 0.3) {
				tSignalValueSpeed.Transmit(&speedOutput, 0.2, 0, outputTime);
			} else {
				tSignalValueSpeed.Transmit(&speedOutput, speedController.speedConfig.min, 0, outputTime);
			}

			if(!carPoses->IsEmpty()) {
				//save last intersection
				lastIntersection.Save(carPoses->GetGlobalPose(Point2f(stop.f32_x, stop.f32_y), stop.f32_yaw), outputTime);
			}

			TFeedbackStruct::Data feedback;
			feedback.ui32_filterID = F_LINE_SPECIFIER;
			feedback.ui32_status = FB_LS_STOPLINE;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
		}
	} else if (prioritySign.distance > 0) {
		RETURN_IF_FAILED(ProcessIntersection(debugImage, frameCoord, nearPlane, outputTime));
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessParkingDetection(cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime){
	ParkingSpotDetector::ParkingSpot parking = parkingDetector->Detect(image, &debugImage, actionSub.command, firstParkingDetection);

	if (parking.valid) {
		Point2f parkingPosition = ImageUtils::ConvertToWorldCoordinates(parking.locationOnVerticalLine, frameCoord);
		Point2f direction = cv::Point2f(-parking.verticalLine.direction.y, -parking.verticalLine.direction.x);
		//cout << "dir " << direction.x << " " << direction.y << endl;

		if(parking.typeLong) {
			parkingPosition += offset.parkingLong.x * direction;
			parkingPosition += offset.parkingLong.y * CVMath::RotateCW90(- direction);
		} else {
			parkingPosition += offset.parkingTrans.x * direction;
			parkingPosition += offset.parkingTrans.y * CVMath::RotateCW90(- direction);
		}

		TPoseStruct::Data data;
		data.f32_x = parkingPosition.x;
		data.f32_y = parkingPosition.y;
		if(direction.x == 0) {
			RETURN_NOERROR;
		} else if(direction.x < 0) {
			data.f32_yaw = atan(direction.y / (-direction.x));
		} else {
			data.f32_yaw = atan(direction.y / direction.x);
		}

		if (debugEnabled) {
			char info[100];
			sprintf(info, "parking x %f y %f yaw %f", parkingPosition.x, parkingPosition.y, data.f32_yaw);

			cv::putText(debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
		}

		LOG_WARNING(cString::Format("LineSpec: parking x %f y %f yaw %f vertline dir x %f y %f", parkingPosition.x, parkingPosition.y, 	data.f32_yaw, direction.x ,direction.y));

		// TRANSMITTING stop Line
		tPoseStructGoal.Transmit(&localGoalOutput, data, outputTime);

		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_LINE_SPECIFIER;

		if(parking.typeLong) {
			feedback.ui32_status = FB_LS_PARKING_LONG;
		} else {
			feedback.ui32_status = FB_LS_PARKING_TRANS;
		}

		tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
	} else {
		TPoseStruct::Data data;

		data.f32_x = 1;
		tPoseStructGoal.Transmit(&localGoalOutput, data, outputTime);
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessPullOut(cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime) {
	TFeedbackStruct::Data feedback;
	feedback.ui32_filterID = F_LINE_SPECIFIER;
	Point2f goal;
	Point2f  directionImage;
	if(parkingDetector->IsLongitudinalParkingSpot(image, &debugImage)) {
		feedback.ui32_status = FB_LS_IN_PARKSPOT_LONG;

		tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
	} else if (parkingDetector->IsTransversalParkingSpot(image, &debugImage, &goal, &directionImage)) {

		Point2f goalWorld = ImageUtils::ConvertToWorldCoordinates(goal, frameCoord);

		Point2f direction = cv::Point2f(-directionImage.y, -directionImage.x); //Transform to world
		direction = CVMath::RotateCW90(-direction);

		tFloat32 pullOutDistance = direction.dot(goalWorld) + offset.pullOutXOffset;

		TPoseStruct::Data stop;
		stop.f32_x = direction.x * pullOutDistance;
		stop.f32_y = direction.y * pullOutDistance;
		if(direction.x == 0) {
			stop.f32_yaw = 0;
		} else if(direction.x < 0) {
			stop.f32_yaw = atan(direction.y / (-direction.x));
		} else {
			stop.f32_yaw = atan(direction.y / direction.x);
		}

		LOG_WARNING(cString::Format("LineSpec Pull Out: %s", stop.ToString().GetPtr()));

		tPoseStructGoal.Transmit(&localGoalOutput, stop, outputTime);

		feedback.ui32_status = FB_LS_IN_PARKSPOT_TRANS;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
	} else {
		feedback.ui32_status = FB_LS_IN_PARKSPOT_LONG;

		tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
	}

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ProcessIntersection(cv::Mat &debugImage, cv::Point2f frameCoord, tFloat32 nearPlane, tTimeStamp outputTime) {
	Point2f image_yawDirection;
	Point2f image_stopPosition = intersectionDetector->DetectIntersection(image, &debugImage, METER_TO_PIXEL(prioritySign.distance), METER_TO_PIXEL(nearPlane), &image_yawDirection, &lastIntersection.badCrossingType, outputTime);

	if ((image_stopPosition.x > 0 && image_stopPosition.y > 0)) {
		intersectionDetector->SetTrackingMode(true);
		speedController.detectionModeActive = 5; // 5 frames
	}

	if(intersectionDetector->IsInTrackingMode()) {
		Point2f localStopPosition = carPoses->InterpolatePoses(intersectionDetector->GetDetectedStopPositions(), frameCoord);

		if(localStopPosition.x > offset.stopTransmissionDistance) {
			RETURN_NOERROR;
		}

		if(localStopPosition.x == 0 && localStopPosition.y == 0) {
			if(carPoses->IsEmpty()) {
				//fallback, if no car poses in the cache
				localStopPosition = ImageUtils::ConvertToWorldCoordinates(image_stopPosition, frameCoord);
			} else if (intersectionDetector->GetDetectedStopPositions().size() > 1){
				//fix bug
				LOG_WARNING("LineSpec: tracking failed");
				localStopPosition = intersectionDetector->GetDetectedStopPositions().front().pos;

				if(localStopPosition.x == 0 && localStopPosition.y == 0) {
					localStopPosition.x = offset.stopTransmissionDistance - 0.1;
				}
				image_yawDirection = Point2f(0, -1);
				RETURN_NOERROR;
			} else {
				RETURN_NOERROR;
			}
		}

		TPoseStruct::Data stop = DetermineWorldStopPoint(localStopPosition, image_yawDirection);

		if(stop.f32_x == 0 && stop.f32_y == 0) {
			RETURN_NOERROR;
		}

		intersectionDetector->SetTrackingMode(false);
		intersectionDetector->ClearDetectedStopPositions();

		if (debugEnabled) {
			char info[100];
			sprintf(info, "crossing x %f y %f yaw %f", stop.f32_x, stop.f32_y, stop.f32_yaw);

			cv::putText(debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
		}

		if(lastIntersection.badCrossingType) {
			LOG_WARNING(cString::Format("LineSpec: stop with sign x %f y %f yaw %f", stop.f32_x, stop.f32_y, stop.f32_yaw));
		} else {
			if(stop.f32_x < 0) {
				stop.f32_x = 0;
				stop.f32_y = 0;
			}
			LOG_WARNING(cString::Format("LineSpec: crossing x %f y %f yaw %f", stop.f32_x, stop.f32_y, stop.f32_yaw));
		}

		if(actionSub.command == AC_LS_SLOW_RIGHTLANE || actionSub.command == AC_LS_NORMAL_RIGHTLANE) {
			// TRANSMITTING stop Line
			prioritySign.distance = -1;

			// brake before loosing control
			if(stop.f32_x < 0) {
				if(lastIntersection.badCrossingType) {
					stop.f32_x *= 1.2;
				} else {
					stop.f32_x = 0;
					stop.f32_y = 0;
				}
				tSignalValueSpeed.Transmit(&speedOutput, 0, 0, outputTime);
			} else if(stop.f32_x < 0.3) {
				tSignalValueSpeed.Transmit(&speedOutput, 0.2, 0, outputTime);
			} else {
				tSignalValueSpeed.Transmit(&speedOutput, speedController.speedConfig.min, 0, outputTime);
			}

			tPoseStructGoal.Transmit(&localGoalOutput, stop, outputTime);

			if(!carPoses->IsEmpty()) {
				//save last intersection
				lastIntersection.Save(carPoses->GetGlobalPose(Point2f(stop.f32_x, stop.f32_y), stop.f32_yaw), outputTime);
			}

			speedController.isInCurve = 0;

			TFeedbackStruct::Data feedback;
			feedback.ui32_filterID = F_LINE_SPECIFIER;
			if(lastIntersection.badCrossingType) {
				feedback.ui32_status = FB_LS_STOP_WITH_SIGN;
			} else {
				feedback.ui32_status = FB_LS_CROSSING_WAIT_POSITION;
			}
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, outputTime);
		}
	}

	RETURN_NOERROR;
}

TPoseStruct::Data LineSpecifierAdapter::DetermineWorldStopPoint(Point2f stopPosition, Point2f image_yawDirection) {
	Point2f yawDirection = Point2f(-image_yawDirection.y, -image_yawDirection.x);

	TPoseStruct::Data stop;
	if(yawDirection.x == 0) {
		return stop;
	} else if(yawDirection.x < 0) {
		stop.f32_yaw = atan(yawDirection.y / (-yawDirection.x));
	} else {
		stop.f32_yaw = atan(yawDirection.y / yawDirection.x);
	}

	stopPosition += offset.stopline.x * yawDirection + offset.stopline.y * CVMath::RotateCW90(-yawDirection);

	if(stopPosition.y * stopPosition.x < -offset.maxXYRation_Stopline) {
		stopPosition.y = -offset.maxXYRation_Stopline * stopPosition.x;
	} else if(stopPosition.y * stopPosition.x > offset.maxXYRation_Stopline ) {
		stopPosition.y = offset.maxXYRation_Stopline * stopPosition.x;
	}

	if(stopPosition.x == 0) {
		stopPosition.x = 0.005;
	} else if(stopPosition.x < 0) {
		stopPosition.y = 0;
	}

	stop.f32_x = stopPosition.x;
	stop.f32_y = stopPosition.y;
	stop.f32_yaw *= offset.yawScale;

	return stop;
}

tResult LineSpecifierAdapter::ProcessBackwards(cv::Mat &debugImage, Point2f frameCoord, cv::Point2f image_destination, tTimeStamp outputTime) {
	Point2f image_yawDirection;
	Point2f image_DestPosition = speedController.Backwards(image, &debugImage, image_destination, outputTime, &image_yawDirection, ImageUtils::MeterToPixel(frameCoord.x) - BINARY_IMAGE_HEIGHT);

	Point2f destPosition = ImageUtils::ConvertToWorldCoordinates(image_DestPosition, frameCoord);
	Point2f yawDirection = Point2f(-image_yawDirection.y, -image_yawDirection.x);

	TPoseStruct::Data stop;
	if(yawDirection.x == 0) {
		stop.f32_yaw = 0;
	} else if(yawDirection.x < 0) {
		stop.f32_yaw = atan(yawDirection.y / (-yawDirection.x));
	} else {
		stop.f32_yaw = atan(yawDirection.y / yawDirection.x);
	}

	destPosition += offset.backwards.x * yawDirection + offset.backwards.y * CVMath::RotateCW90(-yawDirection);

	stop.f32_x = destPosition.x;
	stop.f32_y = destPosition.y;

	tPoseStructGoal.Transmit(&localGoalOutput, stop, outputTime);

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ConvertInputImage(IMediaSample* mediaSample) {
	tBitmapFormat  format;
	{
		adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));
		format = sync.inputFormat;
	}

	if (format.nPixelFormat != cImage::PF_GREYSCALE_8) {
		RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED, "LineSpecifier: Only Greyscale images are implemented");
	}

    const tVoid* srcBuffer;

    Mat inputImage;
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(mediaSample->Lock(&srcBuffer)))
    {
    	//std::cout << "inputFormat.nHeight" << inputFormat.nHeight <<"inputFormat.nWidth"  <<inputFormat.nWidth <<std::endl;
    	inputImage = Mat(format.nHeight, format.nWidth,CV_8UC1,(tVoid*)srcBuffer, format.nBytesPerLine);

#if LOCAL_IMAGE_COPY
    	inputImage.copyTo(image);
#endif
    	image = inputImage;
    }
    mediaSample->Unlock(srcBuffer);

	RETURN_NOERROR;
}

tResult LineSpecifierAdapter::ReadAsFloatAndClear(cv::Mat& image, tUInt32 bytePosition, tFloat32 *value) {
	if(bytePosition > image.dataend - image.datastart) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, "CameraEstimatorAdapter::ReadAsFloatAndClear bytePosition out of bounds");
	}

	union {
	     tUInt8 bytes[4];
	     tFloat32 f;
	} byteToFloat;

	byteToFloat.bytes[0] = image.data[bytePosition];
	byteToFloat.bytes[1] = image.data[bytePosition+1];
	byteToFloat.bytes[2] = image.data[bytePosition+2];
	byteToFloat.bytes[3] = image.data[bytePosition+3];

	*value = byteToFloat.f;

	image.data[bytePosition] = 0;
	image.data[bytePosition+1] = 0;
	image.data[bytePosition+2] = 0;
	image.data[bytePosition+3] = 0;

	RETURN_NOERROR;
}

tVoid LineSpecifierAdapter::Record(Mat &in, Mat &debug) {
	cString filename;
	const int START = 0;
	const int END = 2000;
	if (imageNumber >= START && imageNumber <= END) {
		int i = imageNumber;

		if(!cFileSystem::Exists("/tmp/record/")) {
			cFileSystem::CreatePath ("/tmp/record/");
		}

		filename = cString::Format("/tmp/record/in_%04d.jpg", i);
		imwrite(filename.GetPtr(), in);

		filename = cString::Format("/tmp/record/de_%04d.jpg", i);
		imwrite(filename.GetPtr(), debug);
	}
	imageNumber++;

	if(imageNumber > END) {
		imageNumber = START;
	}
}
