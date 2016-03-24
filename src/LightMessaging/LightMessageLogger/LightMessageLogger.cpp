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

#include "LightMessageLogger.h"

ADTF_FILTER_PLUGIN("LightMessageLogger", OID_ADTF_LIGHTMESSAGE_LOGGER, LightMessageLogger);

#define LMLO_DEBUG_TO_CONSOLE "Debug::Debug Output to Console"
#define LMLO_CAR_NAME "CarName: This is Car 1?"

LightMessageLogger::LightMessageLogger(const tChar* __info) : cAsyncDataTriggeredFilter(__info){

	debugModeEnabled = tFalse;

	// JSON file
	logfile_content.header_meta = "{\n   \"meta\": [ \n       {\n";
	logfile_content.header = "\n       }\n   ],\n   \"chat\": [ \n";
	logfile_content.content.clear();
	logfile_content.footer = "\n   ] \n}";

	new_content_counter = 1;
	carIsCarOne = tTrue;

	// Car names
	own_car_name = "Car 1";
	respondent_car_name = "Car 2";

	SetPropertyBool(LMLO_DEBUG_TO_CONSOLE, tFalse);
	SetPropertyStr(LMLO_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance).");

	SetPropertyBool(LMLO_CAR_NAME, tTrue);
	SetPropertyStr(LMLO_CAR_NAME NSSUBPROP_DESCRIPTION, "If enabled this car is called Car 1");

}

LightMessageLogger::~LightMessageLogger() {

}

tResult LightMessageLogger::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	RETURN_NOERROR;
}

tResult LightMessageLogger::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	RETURN_NOERROR;
}

tResult LightMessageLogger::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));


	} else if (eStage == StageNormal) {

		/* read property whether debug mode is required */
		debugModeEnabled = tBool(GetPropertyBool(LMLO_DEBUG_TO_CONSOLE));
		carIsCarOne = tBool(GetPropertyBool(LMLO_CAR_NAME));

		// invert car names if protperty is enabled
		if(!carIsCarOne) {
			cString tmp_own_car_name = own_car_name;
			own_car_name = respondent_car_name;
			respondent_car_name = tmp_own_car_name;
		}

		// clear file content
		logfile_content.content.clear();
		new_content_counter = 1;

		// clear file
		fstream chat_file;
		chat_file.open("/home/aadc/AADC/utilities/Kuer/data.json", ios::out | ios::trunc);
		chat_file << "";
		chat_file.close();

	} else if (eStage == StageGraphReady) {
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());

	}
	RETURN_NOERROR;
}


tResult LightMessageLogger::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr) {

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult LightMessageLogger::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	__synchronized_obj(criticalSection_OnPinEvent);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &actionInput) {
			TActionStruct::ActionSub actionSub_tmp;
			actionSub_tmp = tActionStruct_object.Read_Action(pMediaSample, F_LIGHT_MESSAGE_LOGGER);
			ProcessAction(actionSub_tmp);
		}
	}

	RETURN_NOERROR;
}

tResult LightMessageLogger::AppendToLog(LightMessageLogger::logfile_message message_to_append) {
	__synchronized_obj(criticalSection_FileLogAccess);

	if(message_to_append.car_name != "" && message_to_append.message != "") {

		// insert last received message
		logfile_content.content.push_back(message_to_append);

		// get system time
        time_t rawtime;
        struct tm * timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char dateAndTime[80];
        strftime(dateAndTime, 80, "%H:%M:%S", timeinfo);
        logfile_content.content[logfile_content.content.size()-1].time = cString(dateAndTime);

		fstream chat_file;
		chat_file.open("/home/aadc/AADC/utilities/Kuer/data.json", ios::out | ios::trunc);

		// write file Header
		chat_file << logfile_content.header_meta;
		chat_file << "           \"counter\": \"" << new_content_counter << "\"";
		chat_file << logfile_content.header;

		// write file Content
		for(tUInt32 i = 0; i < logfile_content.content.size(); i++) {
			chat_file << "       {\n           \"time\": \"" << logfile_content.content[i].time << "\", \n"
					"           \"name\": \"" << logfile_content.content[i].car_name << "\", \n"
					"           \"message\": \"" << logfile_content.content[i].message << "\"\n       }";
			if (i < (logfile_content.content.size() - 1)) {
				chat_file << ", \n";
			}
		}

		// write file footer
		chat_file << logfile_content.footer;

		chat_file.close();

		new_content_counter = new_content_counter + 3;

	}

	RETURN_NOERROR;
}

LightMessageLogger::logfile LightMessageLogger::GetLogContent() {
	__synchronized_obj(criticalSection_FileLogAccess);

	return logfile_content;
}

/* process incoming action command; if enabled and started, process corresponding command */
tResult LightMessageLogger::ProcessAction(TActionStruct::ActionSub actionSub_rec){
	if(actionSub_rec.enabled && actionSub_rec.started) {

		logfile_message tmp_log_message;

		switch(actionSub_rec.command) {

			/* Clear History */
			case AC_LML_CLEAR_HISTORY:
				logfile_content.content.clear();
				//new_content_counter = 0; // do not reset counter
				break;

			case AC_LML_LOG_START_SENDER:
			case AC_LML_LOG_START_RECEIVER:
				tmp_log_message.message = "Hi! Kannst du mich sehen?";
				break;
			case AC_LML_LOG_STARTCONFIRMATION_SENDER:
			case AC_LML_LOG_STARTCONFIRMATION_RECEIVER:
				tmp_log_message.message = "Ah, Hallo! Ja, ich sehe dich!";
				break;

			 /* Slave Direction */
			case AC_LML_LOG_REQUEST_SLAVE_DIRECTION_SENDER:
			case AC_LML_LOG_REQUEST_SLAVE_DIRECTION_RECEIVER:
				tmp_log_message.message = "Was hast du vor, wohin möchtest du?";
				break;
			case AC_LML_LOG_SLAVE_TURN_LEFT_SENDER:
			case AC_LML_LOG_SLAVE_TURN_LEFT_RECEIVER:
				tmp_log_message.message = "Ich möchte links abbiegen.";
				break;
			case AC_LML_LOG_SLAVE_TURN_RIGHT_SENDER:
			case AC_LML_LOG_SLAVE_TURN_RIGHT_RECEIVER:
				tmp_log_message.message = "Ich möchte rechts abbiegen.";
				break;
			case AC_LML_LOG_SLAVE_STRAIGHT_SENDER:
			case AC_LML_LOG_SLAVE_STRAIGHT_RECEIVER:
				tmp_log_message.message = "Ich möchte geradeaus fahren.";
				break;

			/* Master Direction */
			case AC_LML_LOG_MASTER_TURN_LEFT_SENDER:
			case AC_LML_LOG_MASTER_TURN_LEFT_RECEIVER:
				tmp_log_message.message = "Oh... okay! Und ich möchte nach links!";
				break;
			case AC_LML_LOG_MASTER_TURN_RIGHT_SENDER:
			case AC_LML_LOG_MASTER_TURN_RIGHT_RECEIVER:
				tmp_log_message.message = "Oh... okay! Und ich möchte nach rechts!";
				break;
			case AC_LML_LOG_MASTER_STRAIGHT_SENDER:
			case AC_LML_LOG_MASTER_STRAIGHT_RECEIVER:
				tmp_log_message.message = "Oh... okay! Und ich möchte geradeaus!";
				break;

			/* Conflict */
			case AC_LML_LOG_CONFLICT_SITUATION_SENDER:
			case AC_LML_LOG_CONFLICT_SITUATION_RECEIVER:
				tmp_log_message.message = "Hm... Dann haben wir ein Problem!";
				break;
			case AC_LML_LOG_PRIORITY_QUESTION_SENDER:
			case AC_LML_LOG_PRIORITY_QUESTION_RECEIVER:
				tmp_log_message.message = "Wer soll nun zuerst fahren?";
				break;
			case AC_LML_LOG_PRIORITY_ANSWER_SENDER:
			case AC_LML_LOG_PRIORITY_ANSWER_RECEIVER:
				tmp_log_message.message = "Du warst als Erster an der Kreuzung... fahr du!";
				break;

			/* Farewell */
			case AC_LML_LOG_MASTER_GOODBYE_SENDER:
			case AC_LML_LOG_MASTER_GOODBYE_RECEIVER:
				tmp_log_message.message = "Alles klar. Dankeschön! Dann fahr ich jetzt los!";
				break;
			case AC_LML_LOG_SLAVE_GOODBYE_SENDER:
			case AC_LML_LOG_SLAVE_GOODBYE_RECEIVER:
				tmp_log_message.message = "Ok, ich warte, bis du weg bist. Tschüss!";
				break;
			case AC_LML_LOG_END_COMMUNICATION_SENDER:
			case AC_LML_LOG_END_COMMUNICATION_RECEIVER:
				tmp_log_message.message = "Tschüss und schönen Tag!";
				break;

			default:
				break;
		}

		// Set Sender/Receiver
		if(tmp_log_message.message != "") {
			if(actionSub_rec.command % 10 == 0) {
				tmp_log_message.car_name = own_car_name;
			} else {
				tmp_log_message.car_name = respondent_car_name;
			}
			// Write to JSON file
			AppendToLog(tmp_log_message);
		}

		// Send feedback
		if(tmp_log_message.car_name != "" && tmp_log_message.message != "") {
			// send feedback
			TFeedbackStruct::Data tmp_feedback;
			tmp_feedback.ui32_filterID = F_LIGHT_MESSAGE_LOGGER;
			tmp_feedback.ui32_status = actionSub_rec.command + 1;
			tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
		}

		if(debugModeEnabled) {
			LOG_WARNING(cString::Format("LightMessageLogger: Activated with ActionCommand %d, Logged String: %s", actionSub_rec.command, tmp_log_message.message.GetPtr()));
		}
	}
	RETURN_NOERROR;
}
