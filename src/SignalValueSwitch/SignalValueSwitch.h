/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for switching between two tSignalValues (based on a boolean input)
**********************************************************************
* $Author::  $ fink, schoen  $Date:: 2016-01-12 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/


#ifndef _SIGNAL_VALUE_SWITCH_H_
#define _SIGNAL_VALUE_SWITCH_H_

#define OID_ADTF_SIGNAL_VALUE_SWITCH "adtf.user.signal_value_switch"

#include "TSignalValue.h"
#include "TBoolSignalValue.h"

//*************************************************************************************************
class SignalValueSwitch : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SIGNAL_VALUE_SWITCH, "Signal Value Switch", OBJCAT_DataFilter, "Signal Value Switch", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin 		signal_1_input;
	cInputPin 		signal_2_input;
	cInputPin		switch_input;
	cOutputPin    	signal_output;


public:
	SignalValueSwitch(const tChar* __info);
	virtual ~SignalValueSwitch();

private: //private methods
	tResult ProcessSwitchInput(IMediaSample* pSample);
	tBool GetSignalSwitchBool();
	tResult TransmitTSignal(const tFloat32 f32Value, const tUInt32 arduinoTimestamp, tTimeStamp tsInputTimee);

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource,
					   tInt nEventCode,
					   tInt nParam1,
					   tInt nParam2,
					   IMediaSample* pMediaSample);
private:
	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);


	/** Necessary for input and output signal values **/
	TSignalValue tSignalValueIn1;
	TSignalValue tSignalValueIn2;
	TSignalValue tSignalValueOut;

	/** Necessary for boolean input **/
	TBoolSignalValue tBoolSignalValue;
	TBoolSignalValue::Data boolInput;


	// critical sections
	cCriticalSection	m_oProcessSwitchInput;
	cCriticalSection	m_oTransmitTSigalValue;


};

//*************************************************************************************************
#endif // _SIGNAL_VALUE_SWITCH_H_
