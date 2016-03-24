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


#include "stdafx.h"
#include "SignalValueSwitch.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Signal Value Switch", OID_ADTF_SIGNAL_VALUE_SWITCH, SignalValueSwitch);

SignalValueSwitch::SignalValueSwitch(const tChar* __info) :
		cFilter(__info) {
}

SignalValueSwitch::~SignalValueSwitch() {

}

tResult SignalValueSwitch::CreateInputPins(__exception)
{
	/*************************************************/
	/*	Input 1 (tSignalValue) */

	// Create input pin for signal input 1
	RETURN_IF_FAILED(signal_1_input.Create("Sig_FALSE", tSignalValueIn1.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&signal_1_input));

	/*************************************************/
	/*	Input 2 (tSignalValue) */

	// Create input pin for signal input 2
	RETURN_IF_FAILED(signal_2_input.Create("Sig_TRUE", tSignalValueIn2.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&signal_2_input));

	/*************************************************/
	/*	Switch Input (tBoolSignalValue) */

	// Create input pin for switch input
	RETURN_IF_FAILED(switch_input.Create("Switch", tBoolSignalValue.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&switch_input));

	RETURN_NOERROR;
}

tResult SignalValueSwitch::CreateOutputPins(__exception)
{
	// create output pin for tSignalValue Output
	RETURN_IF_FAILED(signal_output.Create("Signal_Output", tSignalValueOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&signal_output));

	RETURN_NOERROR;
}

tResult SignalValueSwitch::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		tSignalValueIn1.StageFirst(__exception_ptr);
		tSignalValueIn2.StageFirst(__exception_ptr);
		tSignalValueOut.StageFirst(__exception_ptr);
		tBoolSignalValue.StageFirst(__exception_ptr);

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface

	}
	else if (eStage == StageGraphReady)
	{
		tSignalValueIn1.StageGraphReady();
		tSignalValueIn2.StageGraphReady();
		tSignalValueOut.StageGraphReady();
		tBoolSignalValue.StageGraphReady();
	}

	RETURN_NOERROR;
}

tResult SignalValueSwitch::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult SignalValueSwitch::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult SignalValueSwitch::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult SignalValueSwitch::ProcessSwitchInput(IMediaSample* pMediaSample) {
	__synchronized_obj(m_oProcessSwitchInput);
	tBoolSignalValue.Read(pMediaSample, &boolInput);

	RETURN_NOERROR;
}

tBool SignalValueSwitch::GetSignalSwitchBool() {
	__synchronized_obj(m_oProcessSwitchInput);
	return boolInput.bValue;
}

tResult SignalValueSwitch::TransmitTSignal(const tFloat32 f32Value, const tUInt32 arduinoTimestamp,
		tTimeStamp tsInputTime) {
	__synchronized_obj(m_oTransmitTSigalValue);
	//create new media sample

	TSignalValue::Data data;
	data.f32_value = f32Value;
	data.ui32_arduinoTimestamp = arduinoTimestamp;

	tSignalValueOut.Transmit(&signal_output, data, tsInputTime);

	RETURN_NOERROR;
}

tResult SignalValueSwitch::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is (and if output exists?)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		if (pSource == &switch_input) {
			ProcessSwitchInput(pMediaSample);
		} else if (pSource == &signal_1_input) {
			TSignalValue::Data data; // save input 1 signal
			tTimeStamp mediaSampleTime_1;

			tSignalValueIn1.Read(pMediaSample, &data);
			mediaSampleTime_1 = pMediaSample->GetTime();

			// write out input 1 / Default/Start input pin
			if (!GetSignalSwitchBool()) {
				TransmitTSignal(data.f32_value, data.ui32_arduinoTimestamp, mediaSampleTime_1);
			}

		} else if (pSource == &signal_2_input) {
			TSignalValue::Data data; // save input 1 signal
			tTimeStamp mediaSampleTime_1;

			tSignalValueIn2.Read(pMediaSample, &data);
			mediaSampleTime_1 = pMediaSample->GetTime();

			// write out input 1
			if (GetSignalSwitchBool()) {
				TransmitTSignal(data.f32_value, data.ui32_arduinoTimestamp, mediaSampleTime_1);
			}
		}
	}

	RETURN_NOERROR;
}
