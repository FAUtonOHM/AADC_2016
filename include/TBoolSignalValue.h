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
 * $Author:: fink $   $Date:: 2016-01-27 #$
 **********************************************************************/

#ifndef T_BOOLSIGNALVALUE_H_
#define T_BOOLSIGNALVALUE_H_

#include <adtf_ucom.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

class TBoolSignalValue {

	tInt size;

	struct IDs {
		tBool set;

		tBufferID bValue;
		tBufferID ui32_arduinoTimestamp;
	} ids;

	ucom::cObjectPtr<IMediaTypeDescription> description;
	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:

	struct Data {
		tUInt32 ui32_arduinoTimestamp;
		tBool bValue;

		Data() {
			bValue = tFalse;
			ui32_arduinoTimestamp = 0;
		}
	};

	TBoolSignalValue() {
		size = 0;

		ids.set = tFalse;
		ids.bValue = tFalse;
		ids.ui32_arduinoTimestamp = 0;

		stageFirstCalled = tFalse;
	}

	tResult StageFirst( __exception) {
		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for tSignalValue
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);

		type = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_POINTER_NULL(type);

		//get mediatype description for tBoolSignalValue data type
		RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&description));

		stageFirstCalled = tTrue;

		RETURN_NOERROR;
	}

	tResult StageGraphReady() {
		ids.set = tFalse;

		//allocate memory with the size given by the descriptor tPoseStruct
		cObjectPtr<IMediaSerializer> serializer;
		description->GetMediaSampleSerializer(&serializer);
		size = serializer->GetDeserializedSize();

		RETURN_NOERROR;
	}

	ucom::cObjectPtr<IMediaType> GetMediaType() {
		return type;
	}

	tResult Read(IMediaSample* mediaSample, Data *data) {
		ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderInput;
		adtf::cScopedMediaDescriptionReadLock descriptionReadLock(description, mediaSample, &pCoderInput);

		if(pCoderInput == 0) {
			LOG_WARNING (cString::Format("TBoolSignalValue::Read pCoderInput is nullpointer"));
			RETURN_ERROR(ERR_POINTER);
		}

		// get the IDs for the items in the media sample
		if (!ids.set) {
			pCoderInput->GetID("bValue", ids.bValue);
			pCoderInput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
			ids.set = tTrue;
		}

		//get values from media sample
		pCoderInput->Get(ids.bValue, (tVoid*) &(data->bValue));
		pCoderInput->Get(ids.ui32_arduinoTimestamp,(tVoid*) &(data->ui32_arduinoTimestamp));

		RETURN_NOERROR;
	}

	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
		if(size == 0 || !stageFirstCalled) {
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("TBoolSignalValue: Transmit failed on pin %s due to size not set or not initialized", outputPin->GetName()));
		}

		ucom::cObjectPtr<IMediaSample> newMediaSample;
		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
			newMediaSample->AllocBuffer(size);

			{
				ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderOutput;
				adtf::cScopedMediaDescriptionReadLock descriptionWriteLock(description, newMediaSample, &pCoderOutput);

				if(pCoderOutput == 0) {
					LOG_WARNING (cString::Format("TBoolSignalValue::Transmit pCoderInput is nullpointer"));
					RETURN_ERROR(ERR_POINTER);
				}

				// get the IDs for the items in the media sample
				if (!ids.set) {
					pCoderOutput->GetID("bValue", ids.bValue);
					pCoderOutput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
					ids.set = tTrue;
				}

				// set values in new media sample
				pCoderOutput->Set(ids.bValue, &data.bValue);
				pCoderOutput->Set(ids.ui32_arduinoTimestamp, &data.ui32_arduinoTimestamp);
			}

			//transmit media sample over output pin
			newMediaSample->SetTime(outputTime);
			outputPin->Transmit(newMediaSample);
		}

		RETURN_NOERROR;
	}

	tResult Transmit(cOutputPin *outputPin, tBool bValue, tUInt32 ui32_arduinoTimestamp, tTimeStamp outputTime) {
		TBoolSignalValue::Data data;
		data.bValue = bValue;
		data.ui32_arduinoTimestamp = ui32_arduinoTimestamp;
		return Transmit(outputPin, data, outputTime);
	}
};

#endif // T_BOOLSIGNALVALUE_H_
