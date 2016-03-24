/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements an Light (morse) Message Transmitter using Car Head Lights
**********************************************************************
* $Author::  $ fink  $Date:: 2016-03-12 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#ifndef _LIGHTMESSAGE_TRANSMITTER_FILTER_HEADER_
#define _LIGHTMESSAGE_TRANSMITTER_FILTER_HEADER_

#define OID_ADTF_LIGHTMESSAGE_TRANSMITTER  "adtf.aadc.lightmessageTransmitter"

#define DEBUG_MODE_OUTPUT_VIDEO

#include "stdafx.h"
#include "TBoolSignalValue.h"

class LightMessageTransmitter: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LIGHTMESSAGE_TRANSMITTER, "LightMessageTransmitter", OBJCAT_DataFilter, "LightMessageTransmitter", 1, 0, 0, "FAUtonOHM");

protected:

	cInputPin		actionInput;

	cOutputPin		feedbackOutput;
	cOutputPin		headLightOutput;

	TFeedbackStruct	tFeedbackStruct_object;
	TActionStruct	tActionStruct_object;

	TBoolSignalValue HeadLight;

private:
	// synchronisation of OnPinEvent
	cCriticalSection criticalSection_OnPinEvent;
	cCriticalSection criticalSectionTimerSetup;
	cCriticalSection criticalSectionStringAccess;
	cCriticalSection criticalSectionLastActionAccess;
	cCriticalSection criticalSectionRunningStateAccess;
	cCriticalSection criticalSectionSendStringAccess;

	// running state
	tBool runningState;

	// last received action command
	tUInt32 last_received_action;

	// properties
	tBool debugModeEnabled; // debug mode

	tFloat32 dit_time;	// dit time limit in ms

	cString send_string; // string to send

	cString current_char_string;
	cString current_morse_code;

	tUInt32 char_counter;
	tUInt32 symbol_counter;
	tUInt32 symbol_length;

	struct morse_code{
		const char *morse[37];
		char alph[37];
	};

	morse_code codes; // morse code table

    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;

public:
	LightMessageTransmitter(const tChar*);
	virtual ~LightMessageTransmitter();

	tResult CreateInputPins(__exception);
	tResult CreateOutputPins(__exception);

	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);

    /*! overrides cFilter */
    virtual tResult Start(__exception = NULL);

    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

private:

	tResult ProcessAction(TActionStruct::ActionSub);

	// Get next character in send string
	tChar GetNextChar();

	// get next symbol in current send string character
	tChar GetNextSymbol();

	/*! creates the timer for light message transmits*/
	tResult createTimer();

	/*! destroys the timer for light message transmits*/
	tResult destroyTimer(__exception = NULL);

	/* access to last received action command */
	tResult SetLastAction(tUInt32);
	tUInt32 GetLastAction();

	/* access to runningState */
	tResult SetRunningState(tBool);
	tBool GetRunningState();

	/* access to string to send */
	tResult SetSendString(cString);
	cString GetSendString();

};

#endif // _LIGHTMESSAGE_TRANSMITTER_FILTER_HEADER_
