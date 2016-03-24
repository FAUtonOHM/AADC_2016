/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements an Light (Morse) Message Receiver using an RGB camera
**********************************************************************
* $Author::  $  fink $Date:: 2016-03-01 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#ifndef _LIGHTMESSAGE_RECEIVER_FILTER_HEADER_
#define _LIGHTMESSAGE_RECEIVER_FILTER_HEADER_

#define OID_ADTF_LIGHTMESSAGE_RECEIVER  "adtf.aadc.lightmessagereceiver"

//#define DEBUG_MODE_OUTPUT_VIDEO
//#define DEBUG_MODE_WRITE_IMAGES

#include "stdafx.h"

class LightMessageReceiver: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LIGHTMESSAGE_RECEIVER, "LightMessageReceiver", OBJCAT_DataFilter, "LightMessageReceiver", 1, 0, 0, "FAUtonOHM");

protected:
	//Input of the RGB image
	cVideoPin inputVideoImagePin;

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	// output debug depth video
	cVideoPin outputVideoPin;
	#endif

	cInputPin		actionInput;
	cOutputPin		feedbackOutput;

	TFeedbackStruct	tFeedbackStruct_object;
	TActionStruct	tActionStruct_object;

private:
	// synchronisation of OnPinEvent
	cCriticalSection criticalSection_OnPinEvent;
	cCriticalSection criticalSectionLastActionAccess;
	cCriticalSection criticalSectionStringAccess;
	cCriticalSection criticalSectionRunningStateAccess;
	cCriticalSection cCritialSectionReceiveStringAccess;

	/* flag indicating whether input video format has to be read */
	tBool firstFrame;
	tBool saved_diffimage;

	tBitmapFormat inputFormat;

	// running statte
	tBool runningState;

	// last received action command
	tUInt32 last_received_action;

	// properties
	tBool debugModeEnabled; // debug mode
	tUInt32 pixel_threshold; // pixel threshold for pixel to be counted as changed (positive or negative)
	tUInt32 min_changed_pixel; // min pixel changes for detecting light toggle
	tUInt32 max_changed_pixel; // max pixel changes for detecting light toggle
	tFloat32 dit_time;	// dit time limit in ms
	tFloat32 dah_time;	// dah time limit in ms
	tFloat32 wordspace_time;	// wordspace time limit in ms

	// last received image
	Mat last_received_gray;

	// car image
	cFilename carImageFilename;
	Mat matchResult;
	cv::Point carPosition;
	Rect carROI;
	Mat car;

	// tmp image counter
	tUInt32 counter_images;

	tBool light_status;
	tBool timer_running;
	tTimeStamp start_time;

	// save received strings
	cString received_char;
	cString received_string;
	cString receive_string;

	struct morse_code{
		const char *morse[37];
		char alph[37];
	};

	morse_code codes;

	/* structure for region of interest, to be checked for obstacles */
	struct roi {
		cv::Point2f bottomleft_corner;
		tFloat32 roi_y_width;
		tFloat32 roi_x_height;

		roi(){
			bottomleft_corner.x = 0.0f;
			bottomleft_corner.y = 0.0f;
			roi_y_width = 0.0f;
			roi_x_height = 0.0f;
		}
	};

	/* specific regions of interest that need to be checked */
	/* for checking at intersections */
	// TODO integrate!
	roi roi_intersection_oncoming;

public:
	LightMessageReceiver(const tChar*);
	virtual ~LightMessageReceiver();

	tResult CreateInputPins(__exception);
	tResult CreateOutputPins(__exception);
	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:

	tResult ProcessVideo(IMediaSample* sample);

	tResult GetInputVideoImage(cv::Mat &image, IMediaSample* mediaSample);

	tResult ProcessAction(TActionStruct::ActionSub actionSub);

	tResult ProcessReceivedChar(cString);

	/* access to last received action command */
	tResult SetLastAction(tUInt32);
	tUInt32 GetLastAction();

	/* access to runningState */
	tResult SetRunningState(tBool);
	tBool GetRunningState();

	/* access to string/char to receive */
	tResult SetReceiveString(cString);
	cString GetReceiveString();

	cString GetReceivedString();

};

#endif // _LIGHTMESSAGE_RECEIVER_FILTER_HEADER_
