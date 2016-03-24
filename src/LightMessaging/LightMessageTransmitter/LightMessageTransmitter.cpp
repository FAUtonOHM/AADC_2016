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

#include "LightMessageTransmitter.h"

ADTF_FILTER_PLUGIN("LightMessageTransmitter", OID_ADTF_LIGHTMESSAGE_TRANSMITTER, LightMessageTransmitter);

#define LMTX_DEBUG_TO_CONSOLE "Debug::Debug Output to Console"

#define LMTX_TIME_LIMITS_DIT "Time Limits::DIT time limit in ms"
#define LMTX_TIME_LIMITS_DAH "Time Limits::DAH time limit in ms"
#define LMTX_TIME_LIMITS_WORDSPACE "Time Limits:: Wordspace time limit in ms"

LightMessageTransmitter::LightMessageTransmitter(const tChar* __info) : cAsyncDataTriggeredFilter(__info){

	debugModeEnabled = tFalse;

	m_hTimerOutput = NULL;

	dit_time = 100; // dit time limit in ms

	SetPropertyBool(LMTX_DEBUG_TO_CONSOLE,tFalse);
	SetPropertyStr(LMTX_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance).");

	SetPropertyFloat(LMTX_TIME_LIMITS_DIT, 100);
	SetPropertyFloat(LMTX_TIME_LIMITS_DIT NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LMTX_TIME_LIMITS_DIT NSSUBPROP_DESCRIPTION,
				"DIT time limit in ms");

	//codes.alph = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0',' '};
	//codes.morse = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--..",".----","..---","...--","....-",".....","-....","--...","---..","----.","-----"," "};

	codes.alph[0] = 'A';
	codes.alph[1] = 'B';
	codes.alph[2] = 'C';
	codes.alph[3] = 'D';
	codes.alph[4] = 'E';
	codes.alph[5] = 'F';
	codes.alph[6] = 'G';
	codes.alph[7] = 'H';
	codes.alph[8] = 'I';
	codes.alph[9] = 'J';
	codes.alph[10] = 'K';
	codes.alph[11] = 'L';
	codes.alph[12] = 'M';
	codes.alph[13] = 'N';
	codes.alph[14] = 'O';
	codes.alph[15] = 'P';
	codes.alph[16] = 'Q';
	codes.alph[17] = 'R';
	codes.alph[18] = 'S';
	codes.alph[19] = 'T';
	codes.alph[20] = 'U';
	codes.alph[21] = 'V';
	codes.alph[22] = 'W';
	codes.alph[23] = 'X';
	codes.alph[24] = 'Y';
	codes.alph[25] = 'Z';
	codes.alph[26] = ' ';

	codes.morse[0] = "-.---..";		// A: ".-";
	codes.morse[1] = "---.-.-.-..";	// B: "-...";
	codes.morse[2] = "---.-.---.-..";	// C: "-.-.";
	codes.morse[3] = "---.-.-..";		// D: "-..";
	codes.morse[4] = "-..";			// E: ".";
	codes.morse[5] = "-.-.---.-..";	// F: "..-.";
	codes.morse[6] = "---.---.-..";	// G: "--.";
	codes.morse[7] = "-.-.-.-..";		// H: "....";
	codes.morse[8] = "-.-..";			// I: "..";
	codes.morse[9] = "---.---.---..";	// J: "---";
	codes.morse[10] = "---.-.---..";	// K: "-.-";
	codes.morse[11] = "-.---.-.-..";	// L: ".-..";
	codes.morse[12] = "---.---..";		// M: "--";
	codes.morse[13] = "---.-..";		// N: "-.";
	codes.morse[14] = "---.---.---..";	// O: "---";
	codes.morse[15] = "-.---.---.-..";	// P: ".--.";
	codes.morse[16] = "---.---.-.---..";// Q: "--.-";
	codes.morse[17] = "-.---.-..";		// R: ".-.";
	codes.morse[18] = "-.-.-..";		// S: "...";
	codes.morse[19] = "---..";			// T: "-";
	codes.morse[20] = "-.-.---..";		// U: "..-";
	codes.morse[21] = "-.-.-.---..";	// V: "...-";
	codes.morse[22] = "-.---.---..";	// W: ".--";
	codes.morse[23] = "---.-.-.---.."; // X: "-..-";
	codes.morse[24] = "---.-.---.---..";// Y: "-.--";
	codes.morse[25] = "---.---.-.-..";	// Z: "--..";
	codes.morse[26] = "...";			// Space: " ";
}

LightMessageTransmitter::~LightMessageTransmitter() {

}

tResult LightMessageTransmitter::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	RETURN_NOERROR;
}

tResult LightMessageTransmitter::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	// create tBoolSignalValues for light outputs
	RETURN_IF_FAILED(headLightOutput.Create("headLight", HeadLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&headLightOutput));

	RETURN_NOERROR;
}

tResult LightMessageTransmitter::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(HeadLight.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));


	} else if (eStage == StageNormal) {

		runningState = tFalse; // not running
		send_string = "";

		// current char counter
		char_counter = 0;

		// current symbol counter and length
		symbol_counter = 0;
		symbol_length = 0;

		current_char_string.Clear();
		current_morse_code.Clear();

		/* read property whether debug mode is required */
		debugModeEnabled = tBool(GetPropertyBool(LMTX_DEBUG_TO_CONSOLE));

		// time limit properties
		dit_time = tFloat32(GetPropertyFloat(LMTX_TIME_LIMITS_DIT));

		/* Send EEEEEEEEEE */
		/*
		SetSendString("EEEEEEEEEEEEEEEEEEEEEEEEEEEAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		SetRunningState(tTrue);
		debugModeEnabled = tTrue;
		*/


	} else if (eStage == StageGraphReady) {
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());
		RETURN_IF_FAILED(HeadLight.StageGraphReady());

        createTimer(); // -> to actionInput
	}
	RETURN_NOERROR;
}

tResult LightMessageTransmitter::Start(__exception)
{
    return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult LightMessageTransmitter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER) {
    	if (GetRunningState()) {
    		tChar return_char = GetNextSymbol();

			TBoolSignalValue::Data headLightData;
			headLightData.ui32_arduinoTimestamp = 0;

    		if(return_char == '.') {
    			headLightData.bValue = tFalse;
    		} else if(return_char == '-') {
    			headLightData.bValue = tTrue;
		// end of morse code reached, send one cycle False (light off)
		} else if(return_char == 'c') {
			headLightData.bValue = tFalse;	
    		// No more characters
    		} else if(return_char == '\0') {
    			headLightData.bValue = tFalse; // turn off lights
    			SetRunningState(tFalse);

    			// send feedback
    			TFeedbackStruct::Data tmp_feedback;
    			tmp_feedback.ui32_filterID = F_LIGHT_MESSAGE_TRANSMITTER;
    			tmp_feedback.ui32_status = GetLastAction() + 1;
    			tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());

    			if(debugModeEnabled) {
    				LOG_WARNING(cString::Format("LightMessageTrans: End of string reached, deactivated"));
    			}
    		} else {
    			headLightData.bValue = tFalse;
    			LOG_ERROR(cString::Format("LightMessageTrans: Error: received unknown character"));
    		}

			HeadLight.Transmit(&headLightOutput, headLightData, _clock->GetStreamTime());

    	}
	}

    return cAsyncDataTriggeredFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult LightMessageTransmitter::Stop(__exception)
{
    __synchronized_obj(criticalSectionTimerSetup);

    destroyTimer(__exception_ptr);

    return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}


tResult LightMessageTransmitter::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr) {

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}


tResult LightMessageTransmitter::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	__synchronized_obj(criticalSection_OnPinEvent);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &actionInput) {
			TActionStruct::ActionSub actionSub_tmp;
			actionSub_tmp = tActionStruct_object.Read_Action(pMediaSample, F_LIGHT_MESSAGE_TRANSMITTER);
			ProcessAction(actionSub_tmp);
			SetLastAction(actionSub_tmp.command);
		}
	}

	RETURN_NOERROR;
}

/* process incoming action command; if enabled and started, process corresponding command */
tResult LightMessageTransmitter::ProcessAction(TActionStruct::ActionSub actionSub_rec){
	if(actionSub_rec.enabled && actionSub_rec.started) {
		if(!GetRunningState()) {
			symbol_length = 0;
			symbol_counter = 0;
			char_counter = 0;

			current_char_string.Clear();
			current_morse_code.Clear();

			switch(actionSub_rec.command) {
				case AC_LMT_TRANSMIT_START:
					SetSendString("EEEEEEEEEE");
					break;
				case AC_LMT_TRANSMIT_START_CONFIRMATION:
					SetSendString("TTTTTTTTTT");
					break;
				 /* Slave Direction */
				case AC_LMT_TRANSMIT_REQUEST_SLAVE_DIRECTION:
					SetSendString("WWWWWWWWWW");
					break;
				case AC_LMT_TRANSMIT_SLAVE_TURN_LEFT:
					SetSendString("MMMMMMMMMM");
					break;
				case AC_LMT_TRANSMIT_SLAVE_TURN_RIGHT:
					SetSendString("HHHHHHHHHH");
					break;
				case AC_LMT_TRANSMIT_SLAVE_STRAIGHT:
					SetSendString("AAAAAAAAAA");
					break;
				/* Master Direction */
				case AC_LMT_TRANSMIT_MASTER_TURN_LEFT:
					SetSendString("NNNNNNNNNN");
					break;
				case AC_LMT_TRANSMIT_MASTER_TURN_RIGHT:
					SetSendString("OOOOOOOOOO");
					break;
				case AC_LMT_TRANSMIT_MASTER_STRAIGHT:
					SetSendString("IIIIIIIIII");
					break;
				/* Conflict */
				case AC_LMT_TRANSMIT_CONFLICT_SITUATION:
					SetSendString("GGGGGGGGGG");
					break;
				case AC_LMT_TRANSMIT_PRIORITY_QUESTION:
					SetSendString("KKKKKKKKKK");
					break;
				case AC_LMT_TRANSMIT_PRIORITY_ANSWER:
					SetSendString("DDDDDDDDDD");
					break;
				/* Farewell */
				case AC_LMT_TRANSMIT_MASTER_GOODBYE:
					SetSendString("RRRRRRRRRR");
					break;
				case AC_LMT_TRANSMIT_SLAVE_GOODBYE:
					SetSendString("UUUUUUUUUU");
					break;
				case AC_LMT_TRANSMIT_END_COMMUNICATION:
					SetSendString("SSSSSSSSSS");
					break;
				default:
					SetSendString("");
					SetRunningState(tFalse);
			}

			if(GetSendString() != "") {
				SetRunningState(tTrue);
			}

			if(debugModeEnabled) {
				LOG_WARNING(cString::Format("LightMessageTrans: Activated with ActionCommand %d, SendString: %s", actionSub_rec.command, GetSendString().GetPtr()));
			}
		}
	}
	else{
		if(GetRunningState()) {
			SetRunningState(tFalse);
			//LOG_WARNING(cString::Format("LightMessageTrans: Deactivated with ActionCommand"));
		}
	}
	RETURN_NOERROR;
}

tResult LightMessageTransmitter::SetLastAction(tUInt32 received_action) {
    __synchronized_obj(criticalSectionLastActionAccess);

    last_received_action = received_action;
	RETURN_NOERROR;
}


tUInt32 LightMessageTransmitter::GetLastAction() {
    __synchronized_obj(criticalSectionLastActionAccess);

	return last_received_action;
}

tResult LightMessageTransmitter::SetRunningState(tBool received_state) {
    __synchronized_obj(criticalSectionRunningStateAccess);

    runningState = received_state;
	RETURN_NOERROR;
}

tBool LightMessageTransmitter::GetRunningState() {
    __synchronized_obj(criticalSectionRunningStateAccess);

	return runningState;
}

tResult LightMessageTransmitter::SetSendString(cString send_string_tmp) {
    __synchronized_obj(criticalSectionSendStringAccess);

    send_string = send_string_tmp;
	RETURN_NOERROR;
}

cString LightMessageTransmitter::GetSendString() {

    __synchronized_obj(criticalSectionSendStringAccess);

	return send_string;
}

tChar LightMessageTransmitter::GetNextChar() {
    __synchronized_obj(criticalSectionStringAccess);

	char char_tmp = '\0';

    if(char_counter < (tUInt32)GetSendString().GetLength()) {
    	char send_string_char = GetSendString()[char_counter];
    	char_counter++;
    	return send_string_char;
    } else {
		return char_tmp;
    }
}

tChar LightMessageTransmitter::GetNextSymbol() {
    __synchronized_obj(criticalSectionStringAccess);

	char char_tmp = '\0';

	// Get first/next char from string
	if(current_char_string.GetLength() == 0 && symbol_counter == 0 && symbol_length == 0) {
		char current_char = GetNextChar();

    	if(current_char != '\0') {

			current_char_string.Append(current_char);

			// search for requested character
			for(tUInt32 i = 0; i <= 26; i++) {
				// compare received sign to morse code
				cString codes_alpha;
				codes_alpha.Append(codes.alph[i]);
				if(codes_alpha.CompareNoCase(current_char_string) == 0) {
					current_morse_code.Append(codes.morse[i]);
					if(debugModeEnabled) {
						LOG_WARNING(cString::Format("LightMessageTrans: Received Character %s; Morse Code: %s; i: %d", current_char_string.GetPtr(), current_morse_code.GetPtr(), i));
					}
					break;
				}
			}

			// length of morse string (morse symbols for character)
			symbol_length = current_morse_code.GetLength();

    	} else {

    		//runningState = tFalse;
    		return char_tmp;
		}
	}

	// return current morse symbol
    if(symbol_counter < symbol_length) {
    	char return_char = current_morse_code[symbol_counter];
    	symbol_counter++;
		if(debugModeEnabled) {
			cString tmp_morse_symbol;
			tmp_morse_symbol.Append(return_char);
			if(debugModeEnabled) {
				LOG_WARNING(cString::Format("LightMessageTrans: Found Morse symbol: %s; increased symbol counter, new: %d", tmp_morse_symbol.GetPtr(), symbol_counter));
			}
		}
    	return return_char;
    // end of symbol reached, get next character
    } else {
		symbol_length = 0;
		symbol_counter = 0;
		current_char_string.Clear();
		current_morse_code.Clear();

		char char_tmp_ = 'c';
		return char_tmp_;
    }

}

tResult LightMessageTransmitter::createTimer()
{
	// creates timer with dit time given by property
	__synchronized_obj(criticalSectionTimerSetup);

    if (m_hTimerOutput == NULL) {
    	 m_hTimerOutput = _kernel->TimerCreate(tTimeStamp(dit_time*1000), 0, static_cast<IRunnable*>(this),
                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
		if(debugModeEnabled) {
			LOG_WARNING(cString::Format("LightMessageTrans: Timer created with %f milliseconds", dit_time));
		}
     } else {
    	 LOG_ERROR("LightMessageTrans: Timer is already running. Unable to create a new one.");
     }
	RETURN_NOERROR;
}

tResult LightMessageTransmitter::destroyTimer(__exception)
{
	__synchronized_obj(criticalSectionTimerSetup);
	//destroy timer
	if (m_hTimerOutput != NULL) {
		tResult nResult = _kernel->TimerDestroy(m_hTimerOutput);
		if (IS_FAILED(nResult)) {
			LOG_ERROR("LightMessageTrans: Unable to destroy the timer.");
			THROW_ERROR(nResult);
		}
		m_hTimerOutput = NULL;
	}
	//check if handle for some unknown reason still exists
	else {
		if(debugModeEnabled) {
			LOG_WARNING("LightMessageTrans: Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
		}
		tHandle hFoundHandle = _kernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
		if (hFoundHandle) {
			tResult nResult = _kernel->TimerDestroy(hFoundHandle);
			if (IS_FAILED(nResult)) {
				LOG_ERROR("LightMessageTrans: Unable to destroy the found timer.");
				THROW_ERROR(nResult);
			}
		}
	}

	RETURN_NOERROR;
}
