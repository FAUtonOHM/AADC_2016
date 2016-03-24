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
 * $Author:: fink $   $Date:: 2016-02-09 #$
 **********************************************************************/

#ifndef T_ULTRASONICSTRUCT_H_
#define T_ULTRASONICSTRUCT_H_

#include <adtf_ucom.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

#include <iostream>

class TUltrasonicStruct {

	tInt size;

	struct IDs {
		tBool set;

		tBufferID tbufID_tFrontLeft;
		tBufferID tbufID_tFrontCenterLeft;
		tBufferID tbufID_tFrontCenter;
		tBufferID tbufID_tFrontCenterRight;
		tBufferID tbufID_tFrontRight;
		tBufferID tbufID_tSideLeft;
		tBufferID tbufID_tSideRight;
		tBufferID tbufID_tRearLeft;
		tBufferID tbufID_tRearCenter;
		tBufferID tbufID_tRearRight;
	} ids;

	ucom::cObjectPtr<IMediaTypeDescription> description;
	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:
	struct signal_value {
		tUInt32 ui32_arduinoTimestamp;
		tFloat32 f32_value;

		signal_value() {
			f32_value = 0;
			ui32_arduinoTimestamp = 0;
		}
	};

	struct Data {
		signal_value FrontLeft;
		signal_value FrontCenterLeft;
		signal_value FrontCenter;
		signal_value FrontCenterRight;
		signal_value FrontRight;
		signal_value SideLeft;
		signal_value SideRight;
		signal_value RearLeft;
		signal_value RearCenter;
		signal_value RearRight;
	};

	TUltrasonicStruct() {
		size = 0;

		ids.set = tFalse;
		ids.tbufID_tFrontCenter= 0;
		ids.tbufID_tFrontCenterLeft = 0;
		ids.tbufID_tFrontCenterRight = 0;
		ids.tbufID_tFrontLeft = 0;
		ids.tbufID_tFrontRight = 0;
		ids.tbufID_tRearCenter = 0;
		ids.tbufID_tRearLeft = 0;
		ids.tbufID_tRearRight = 0;
		ids.tbufID_tSideLeft = 0;
		ids.tbufID_tSideRight = 0;

		stageFirstCalled = tFalse;
	}

	tResult StageFirst( __exception) {
		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for tUltrasonicStruct
		tChar const * strDescUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strDescUltrasonicStruct);

		type = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUltrasonicStruct,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_POINTER_NULL(type);

		//get mediatype description for tUltrasonicStruct data type
		RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&description));

		stageFirstCalled = tTrue;

		RETURN_NOERROR;
	}

	tResult StageGraphReady() {
		ids.set = tFalse;

		//allocate memory with the size given by the descriptor tUltrasonicStruct
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
			LOG_WARNING (cString::Format("tUltrasonicStruct: pCoderInput is nullpointer"));
			RETURN_ERROR(ERR_POINTER);
		}

		// get the IDs for the items in the media sample
		if (!ids.set) {
			pCoderInput->GetID("tFrontLeft", ids.tbufID_tFrontLeft);
			pCoderInput->GetID("tFrontCenterLeft", ids.tbufID_tFrontCenterLeft);
			pCoderInput->GetID("tFrontCenter", ids.tbufID_tFrontCenter);
			pCoderInput->GetID("tFrontCenterRight", ids.tbufID_tFrontCenterRight);
			pCoderInput->GetID("tFrontRight", ids.tbufID_tFrontRight);
			pCoderInput->GetID("tSideLeft", ids.tbufID_tSideLeft);
			pCoderInput->GetID("tSideRight", ids.tbufID_tSideRight);
			pCoderInput->GetID("tRearLeft", ids.tbufID_tRearLeft);
			pCoderInput->GetID("tRearCenter", ids.tbufID_tRearCenter);
			pCoderInput->GetID("tRearRight", ids.tbufID_tRearRight);
			ids.set = tTrue;
		}

		//get values from media sample
		pCoderInput->Get(ids.tbufID_tFrontCenter, (tVoid*) &(data->FrontCenter));
		pCoderInput->Get(ids.tbufID_tFrontCenterLeft, (tVoid*) &(data->FrontCenterLeft));
		pCoderInput->Get(ids.tbufID_tFrontCenterRight, (tVoid*) &(data->FrontCenterRight));
		pCoderInput->Get(ids.tbufID_tFrontLeft, (tVoid*) &(data->FrontLeft));
		pCoderInput->Get(ids.tbufID_tFrontRight, (tVoid*) &(data->FrontRight));
		pCoderInput->Get(ids.tbufID_tRearCenter, (tVoid*) &(data->RearCenter));
		pCoderInput->Get(ids.tbufID_tRearLeft, (tVoid*) &(data->RearLeft));
		pCoderInput->Get(ids.tbufID_tRearRight, (tVoid*) &(data->RearRight));
		pCoderInput->Get(ids.tbufID_tSideLeft, (tVoid*) &(data->SideLeft));
		pCoderInput->Get(ids.tbufID_tSideRight, (tVoid*) &(data->SideRight));

		RETURN_NOERROR;
	}

	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
		if(size == 0 || !stageFirstCalled) {
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("tUltrasonicStruct: Transmit failed on pin %s due to size not set or not initialized", outputPin->GetName()));
		}

		ucom::cObjectPtr<IMediaSample> newMediaSample;
		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
			newMediaSample->AllocBuffer(size);

			{
				ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderOutput;
				adtf::cScopedMediaDescriptionReadLock descriptionWriteLock(description, newMediaSample, &pCoderOutput);

				if(pCoderOutput == 0) {
					LOG_WARNING (cString::Format("tUltrasonicStruct::Transmit pCoderInput is nullpointer"));
					RETURN_ERROR(ERR_POINTER);
				}

				// get the IDs for the items in the media sample
				if (!ids.set) {
					pCoderOutput->GetID("tFrontLeft", ids.tbufID_tFrontLeft);
					pCoderOutput->GetID("tFrontCenterLeft", ids.tbufID_tFrontCenterLeft);
					pCoderOutput->GetID("tFrontCenter", ids.tbufID_tFrontCenter);
					pCoderOutput->GetID("tFrontCenterRight", ids.tbufID_tFrontCenterRight);
					pCoderOutput->GetID("tFrontRight", ids.tbufID_tFrontRight);
					pCoderOutput->GetID("tSideLeft", ids.tbufID_tSideLeft);
					pCoderOutput->GetID("tSideRight", ids.tbufID_tSideRight);
					pCoderOutput->GetID("tRearLeft", ids.tbufID_tRearLeft);
					pCoderOutput->GetID("tRearCenter", ids.tbufID_tRearCenter);
					pCoderOutput->GetID("tRearRight", ids.tbufID_tRearRight);
					ids.set = tTrue;
				}

				// set values in new media sample
				pCoderOutput->Set(ids.tbufID_tFrontCenter, (tVoid*) &(data.FrontCenter));
				pCoderOutput->Set(ids.tbufID_tFrontCenterLeft, (tVoid*) &(data.FrontCenterLeft));
				pCoderOutput->Set(ids.tbufID_tFrontCenterRight, (tVoid*) &(data.FrontCenterRight));
				pCoderOutput->Set(ids.tbufID_tFrontLeft, (tVoid*) &(data.FrontLeft));
				pCoderOutput->Set(ids.tbufID_tFrontRight, (tVoid*) &(data.FrontRight));
				pCoderOutput->Set(ids.tbufID_tRearCenter, (tVoid*) &(data.RearCenter));
				pCoderOutput->Set(ids.tbufID_tRearLeft, (tVoid*) &(data.RearLeft));
				pCoderOutput->Set(ids.tbufID_tRearRight, (tVoid*) &(data.RearRight));
				pCoderOutput->Set(ids.tbufID_tSideLeft, (tVoid*) &(data.SideLeft));
				pCoderOutput->Set(ids.tbufID_tSideRight, (tVoid*) &(data.SideRight));
			}

			//transmit media sample over output pin
			newMediaSample->SetTime(outputTime);
			outputPin->Transmit(newMediaSample);
		}

		RETURN_NOERROR;
	}

};

#endif // T_ULTRASONICSTRUCT_H_
