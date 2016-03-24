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
 * $Author:: fink $   $Date:: 2016-02-12 #$
 **********************************************************************/

#ifndef T_WHEELDATA_H_
#define T_WHEELDATA_H_

#include <adtf_ucom.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

#include <iostream>

class TWheelData {

	tInt size;

	struct IDs {
		tBool set;

		tBufferID ui32_arduinoTimestamp;
		tBufferID ui32_wheelTach;
		tBufferID i8_wheelDir;
	} ids;

	ucom::cObjectPtr<IMediaTypeDescription> description;
	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:

	struct Data {
		tUInt32 ui32_arduinoTimestamp;
		tUInt32 ui32_wheelTach;
		tInt8 i8_wheelDir;

		Data() {
			ui32_wheelTach = 0;
			ui32_arduinoTimestamp = 0;
			i8_wheelDir = 0;
		}
	};

	TWheelData() {
		size = 0;

		ids.set = tFalse;
		ids.ui32_arduinoTimestamp = 0;
		ids.ui32_wheelTach = 0;
		ids.i8_wheelDir = 0;

		stageFirstCalled = tFalse;
	}

	tResult StageFirst( __exception) {
		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for TWheelData
		tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
		RETURN_IF_POINTER_NULL(strDescWheelData);

		type = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_POINTER_NULL(type);

		//get mediatype description for TWheelData data type
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
			LOG_WARNING (cString::Format("TWheelData: pCoderInput is nullpointer"));
			RETURN_ERROR(ERR_POINTER);
		}

		// get the IDs for the items in the media sample
		if (!ids.set) {
			pCoderInput->GetID("ui32WheelTach", ids.ui32_wheelTach);
			pCoderInput->GetID("i8WheelDir", ids.i8_wheelDir);
			pCoderInput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
			ids.set = tTrue;
		}

		//get values from media sample
		pCoderInput->Get(ids.ui32_wheelTach, (tVoid*) &(data->ui32_wheelTach));
		pCoderInput->Get(ids.i8_wheelDir, (tVoid*) &(data->i8_wheelDir));
		pCoderInput->Get(ids.ui32_arduinoTimestamp,(tVoid*) &(data->ui32_arduinoTimestamp));

		RETURN_NOERROR;
	}

	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
		if(size == 0 || !stageFirstCalled) {
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("TWheelData: Transmit failed on pin %s due to size not set or not initialized", outputPin->GetName()));
		}

		ucom::cObjectPtr<IMediaSample> newMediaSample;
		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
			newMediaSample->AllocBuffer(size);

			{
				ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderOutput;
				adtf::cScopedMediaDescriptionReadLock descriptionWriteLock(description, newMediaSample, &pCoderOutput);

				if(pCoderOutput == 0) {
					LOG_WARNING (cString::Format("TWheelData::Transmit pCoderInput is nullpointer"));
					RETURN_ERROR(ERR_POINTER);
				}

				// get the IDs for the items in the media sample
				if (!ids.set) {
					pCoderOutput->GetID("ui32WheelTach", ids.ui32_wheelTach);
					pCoderOutput->GetID("i8WheelDir", ids.i8_wheelDir);
					pCoderOutput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
					ids.set = tTrue;
				}

				// set values in new media sample
				pCoderOutput->Set(ids.ui32_wheelTach, (tVoid*)&data.ui32_wheelTach);
				pCoderOutput->Set(ids.i8_wheelDir, (tVoid*)&data.i8_wheelDir);
				pCoderOutput->Set(ids.ui32_arduinoTimestamp, (tVoid*)&data.ui32_arduinoTimestamp);
			}

			//transmit media sample over output pin
			newMediaSample->SetTime(outputTime);
			outputPin->Transmit(newMediaSample);
		}

		RETURN_NOERROR;
	}

};

#endif // T_WHEELDATA_H_
