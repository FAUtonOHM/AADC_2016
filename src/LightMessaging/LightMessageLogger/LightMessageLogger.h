/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements an JSON file logger for LightMessageCommunication
**********************************************************************
* $Author::  $ fink  $Date:: 2016-03-20 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#ifndef _LIGHTMESSAGE_LOGGER_FILTER_HEADER_
#define _LIGHTMESSAGE_LOGGER_FILTER_HEADER_

#define OID_ADTF_LIGHTMESSAGE_LOGGER  "adtf.aadc.lightmessageLogger"

#define DEBUG_MODE_OUTPUT_VIDEO

#include "stdafx.h"

class LightMessageLogger: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LIGHTMESSAGE_LOGGER, "LightMessageLogger", OBJCAT_DataFilter, "LightMessageLogger", 1, 0, 0, "FAUtonOHM");

protected:

	cInputPin		actionInput;

	cOutputPin		feedbackOutput;

	TFeedbackStruct	tFeedbackStruct_object;
	TActionStruct	tActionStruct_object;

private:
	// synchronisation of OnPinEvent
	cCriticalSection criticalSection_OnPinEvent;
	cCriticalSection criticalSection_FileLogAccess;

	// properties
	tBool debugModeEnabled; // debug mode
	tBool carIsCarOne;

	struct logfile_message{
		cString car_name;
		cString time;
		cString message;

		logfile_message(): car_name(""), time(""), message(""){}
	};

	struct logfile{
		cString header;
		cString header_meta;
		std::vector<logfile_message> content;
		cString footer;
	};

	logfile logfile_content; // logfile content

	cString own_car_name;
	cString respondent_car_name;

	tUInt32 new_content_counter;

public:
	LightMessageLogger(const tChar*);
	virtual ~LightMessageLogger();

	tResult CreateInputPins(__exception);
	tResult CreateOutputPins(__exception);

	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);

	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

	/*! overrides cFilter */
	virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

private:

	tResult ProcessAction(TActionStruct::ActionSub);

	/* access to content to log */
	tResult AppendToLog(logfile_message);
	logfile GetLogContent();

};

#endif // _LIGHTMESSAGE_LOGGER_FILTER_HEADER_
