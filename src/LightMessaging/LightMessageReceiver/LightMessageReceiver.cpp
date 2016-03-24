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

#include "LightMessageReceiver.h"

ADTF_FILTER_PLUGIN("LightMessageReceiver", OID_ADTF_LIGHTMESSAGE_RECEIVER, LightMessageReceiver);

#define LM_DEBUG_TO_CONSOLE "Debug::Debug Output to Console"

#define LM_PIXEL_LIMITS_THRESHOLD "Pixel Limits::Differential Image Pixel Threshold"
#define LM_PIXEL_LIMITS_MINCHANGED "Pixel Limits::Minimum changed pixels"
#define LM_PIXEL_LIMITS_MAXCHANGED "Pixel Limits::Maximum changed pixels"

#define LM_TIME_LIMITS_DIT "Time Limits::DIT time limit in ms"
#define LM_TIME_LIMITS_DAH "Time Limits::DAH time limit in ms"
#define LM_TIME_LIMITS_WORDSPACE "Time Limits:: Wordspace time limit in ms"

#define LM_CAR_PICTURE_FILENAME "Car Picture::filename"

LightMessageReceiver::LightMessageReceiver(const tChar* __info) : cAsyncDataTriggeredFilter(__info) {

	debugModeEnabled = tFalse;

	pixel_threshold = 80; // pixel threshold for pixel to be counted as changed (positive or negative)
	min_changed_pixel = 15; // min pixel changes for detecting light toggle
	max_changed_pixel = 200; // max pixel changes for detecting light toggle

	dit_time = 200; // dit time limit in ms
	dah_time = 400; // dah time limit in ms
	wordspace_time = 800; // wordspace time limit in ms

	SetPropertyBool(LM_DEBUG_TO_CONSOLE,tFalse);
	SetPropertyStr(LM_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance).");

	SetPropertyInt(LM_PIXEL_LIMITS_THRESHOLD, 80);
	SetPropertyInt(LM_PIXEL_LIMITS_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_PIXEL_LIMITS_THRESHOLD NSSUBPROP_DESCRIPTION,
				"Pixel threshold for pixel to be counted as changed");

	SetPropertyInt(LM_PIXEL_LIMITS_MINCHANGED, 15);
	SetPropertyInt(LM_PIXEL_LIMITS_MINCHANGED NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_PIXEL_LIMITS_MINCHANGED NSSUBPROP_DESCRIPTION,
				"Min. pixel changes for detecting light toggle");

	SetPropertyInt(LM_PIXEL_LIMITS_MAXCHANGED, 200);
	SetPropertyInt(LM_PIXEL_LIMITS_MAXCHANGED NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_PIXEL_LIMITS_MAXCHANGED NSSUBPROP_DESCRIPTION,
				"Max. pixel changes for detecting light toggle");

	SetPropertyFloat(LM_TIME_LIMITS_DIT, 200);
	SetPropertyFloat(LM_TIME_LIMITS_DIT NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_TIME_LIMITS_DIT NSSUBPROP_DESCRIPTION,
				"DIT time limit in ms");

	SetPropertyFloat(LM_TIME_LIMITS_DAH, 400);
	SetPropertyFloat(LM_TIME_LIMITS_DAH NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_TIME_LIMITS_DAH NSSUBPROP_DESCRIPTION,
				"DAH time limit in ms");

	SetPropertyFloat(LM_TIME_LIMITS_WORDSPACE, 800);
	SetPropertyFloat(LM_TIME_LIMITS_WORDSPACE NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr(LM_TIME_LIMITS_WORDSPACE NSSUBPROP_DESCRIPTION,
				"Wordspace time limit in ms");

	SetPropertyStr(LM_CAR_PICTURE_FILENAME, "../../../../src/aadcUser/src/LightMessaging/LightMessageReceiver/car.jpg");
	SetPropertyBool(LM_CAR_PICTURE_FILENAME NSSUBPROP_FILENAME, tTrue);

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

	codes.morse[0] = ".-";
	codes.morse[1] = "-...";
	codes.morse[2] = "-.-.";
	codes.morse[3] = "-..";
	codes.morse[4] = ".";
	codes.morse[5] = "..-.";
	codes.morse[6] = "--.";
	codes.morse[7] = "....";
	codes.morse[8] = "..";
	codes.morse[9] = "---";
	codes.morse[10] = "-.-";
	codes.morse[11] = ".-..";
	codes.morse[12] = "--";
	codes.morse[13] = "-.";
	codes.morse[14] = "---";
	codes.morse[15] = ".--.";
	codes.morse[16] = "--.-";
	codes.morse[17] = ".-.";
	codes.morse[18] = "...";
	codes.morse[19] = "-";
	codes.morse[20] = "..-";
	codes.morse[21] = "...-";
	codes.morse[22] = ".--";
	codes.morse[23] = "-..-";
	codes.morse[24] = "-.--";
	codes.morse[25] = "--..";
	codes.morse[26] = " ";

}

LightMessageReceiver::~LightMessageReceiver() {

}

tResult LightMessageReceiver::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	// Video Input
	RETURN_IF_FAILED(inputVideoImagePin.Create("RGB_video", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&inputVideoImagePin));

	RETURN_NOERROR;
}

tResult LightMessageReceiver::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	// video output
	RETURN_IF_FAILED(outputVideoPin.Create("Debug_RGB", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&outputVideoPin));
	#endif

	RETURN_NOERROR;
}

tResult LightMessageReceiver::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));


	} else if (eStage == StageNormal) {

		firstFrame = tTrue; // received frame is first video frame
		saved_diffimage = tFalse; // saved first image for differential image calculation
		runningState = tFalse; // not running
		counter_images = 0; // tmp, count video media samples
		light_status = tFalse;
		timer_running = tFalse; // receiving messages

		received_char.Clear();
		received_string.Clear();
		receive_string = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

		/* read property whether debug mode is required */
		debugModeEnabled = tBool(GetPropertyBool(LM_DEBUG_TO_CONSOLE));

		/* Print received characters */
		/*
		debugModeEnabled = tTrue;
		SetRunningState(tTrue);
		*/

		// pixel limit properties
		pixel_threshold = tUInt32(GetPropertyInt(LM_PIXEL_LIMITS_THRESHOLD));
		min_changed_pixel = tUInt32(GetPropertyInt(LM_PIXEL_LIMITS_MINCHANGED));
		max_changed_pixel = tUInt32(GetPropertyInt(LM_PIXEL_LIMITS_MAXCHANGED));

		// time limit properties
		dit_time = tFloat32(GetPropertyFloat(LM_TIME_LIMITS_DIT));
		dah_time = tFloat32(GetPropertyFloat(LM_TIME_LIMITS_DAH));
		wordspace_time = tFloat32(GetPropertyFloat(LM_TIME_LIMITS_WORDSPACE));

		carImageFilename = GetPropertyStr(LM_CAR_PICTURE_FILENAME);

		// create absolute path
		ADTF_GET_CONFIG_FILENAME(carImageFilename);
		cFilename absFilename = carImageFilename.CreateAbsolutePath(".");

		if (cFileSystem::Exists(absFilename) == tFalse) {
			LOG_ERROR("LightMessageReceiver: car.jpg does not exist");
			RETURN_NOERROR;
		}

		car = cv::imread(absFilename.GetPtr(), CV_LOAD_IMAGE_COLOR);
	} else if (eStage == StageGraphReady) {
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());
	}
	RETURN_NOERROR;
}

tResult LightMessageReceiver::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr) {

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}


tResult LightMessageReceiver::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	__synchronized_obj(criticalSection_OnPinEvent);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &inputVideoImagePin) {
				/* get format of video/image stream, only necessary at first frame (if successful, repeated otherwise) */
				if (firstFrame) {
					if(debugModeEnabled) {
						LOG_WARNING(cString::Format("LightMessageRec: First frame detected; properties pixel: threshold: %d, min: %d, max: %d; time: dit: %f, dah: %f, wordspace: %f", pixel_threshold, min_changed_pixel, max_changed_pixel, dit_time, dah_time, wordspace_time));
					}
					cObjectPtr<IMediaType> type;
					RETURN_IF_FAILED(inputVideoImagePin.GetMediaType(&type));

					cObjectPtr<IMediaTypeVideo> typeVideoImage;
					RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&typeVideoImage));

					const tBitmapFormat* format = typeVideoImage->GetFormat();
					if (format == NULL) {
						LOG_ERROR("LightMessageRec: No Bitmap information found on VideoImagePin.");
						RETURN_ERROR(ERR_NOT_SUPPORTED);
					}
					/* assign format of received input video to variable 'format' */
					inputFormat.nPixelFormat = format->nPixelFormat;
					inputFormat.nWidth = format->nWidth;
					inputFormat.nHeight = format->nHeight;
					inputFormat.nBitsPerPixel = format->nBitsPerPixel;
					inputFormat.nBytesPerLine = format->nBytesPerLine;
					inputFormat.nSize = format->nSize;
					inputFormat.nPaletteSize = format->nPaletteSize;

					/* check for format errors */
					if (inputFormat.nWidth <= 0 || inputFormat.nWidth > 700 || inputFormat.nHeight <= 0
							|| inputFormat.nHeight > 500) {
						RETURN_AND_LOG_ERROR_STR(ERR_UNEXPECTED, cString::Format("LightMessageRec: Unexpected video image size, since format with width: %d, height: %d is not supported!",inputFormat.nWidth, inputFormat.nHeight));
					}

					//* if receiving the input format information and processing of first frame was successful, flag is set to false */
					firstFrame = tFalse;

					if(GetRunningState()) {
						RETURN_IF_FAILED(ProcessVideo(pMediaSample));
					}
				} else {
					/* Process received video image */
					if(GetRunningState()) {
						RETURN_IF_FAILED(ProcessVideo(pMediaSample));
					}
				}
		}
		else if (pSource == &actionInput) {
			TActionStruct::ActionSub actionSub_tmp;
			actionSub_tmp = tActionStruct_object.Read_Action(pMediaSample, F_LIGHT_MESSAGE_RECEIVER);
			ProcessAction(actionSub_tmp);
			SetLastAction(actionSub_tmp.command);
		}
	}

	RETURN_NOERROR;
}

/* process incoming action command; if enabled and started, process corresponding command */
tResult LightMessageReceiver::ProcessAction(TActionStruct::ActionSub actionSub_rec){
	if(actionSub_rec.enabled && actionSub_rec.started) {

		if(!GetRunningState()) {
			light_status = tFalse;
			timer_running = tFalse; // receiving messages

			received_char.Clear();
			received_string.Clear();

			switch(actionSub_rec.command) {
				case AC_LMR_RECEIVE_START:
					SetReceiveString("E");
					break;
				case AC_LMR_RECEIVE_START_CONFIRMATION:
					SetReceiveString("TTT");
					break;
				 /* Slave Direction */
				case AC_LMR_RECEIVE_REQUEST_SLAVE_DIRECTION:
					SetReceiveString("WWW");
					break;
				case AC_LMR_RECEIVE_SLAVE_DIRECTION:
					SetReceiveString("MMM"); // M (left), H (right), A (straight)
					// special case: multiple input strings and feedbacks required
					break;
				/* Master Direction */
				case AC_LMR_RECEIVE_MASTER_DIRECTION:
					SetReceiveString("NNN"); // N (left), O (right), I (straight)
					// special case: multiple input strings and feedbacks required
					break;
				/* Conflict */
				case AC_LMR_RECEIVE_CONFLICT_SITUATION:
					SetReceiveString("GGG");
					break;
				case AC_LMR_RECEIVE_PRIORITY_QUESTION:
					SetReceiveString("KKK");
					break;
				case AC_LMR_RECEIVE_PRIORITY_ANSWER:
					SetReceiveString("DDD");
					break;
				/* Farewell */
				case AC_LMR_RECEIVE_MASTER_GOODBYE:
					SetReceiveString("RRR");
					break;
				case AC_LMR_RECEIVE_SLAVE_GOODBYE:
					SetReceiveString("UUU");
					break;
				case AC_LMR_RECEIVE_END_COMMUNICATION:
					SetReceiveString("SSS");
					break;
				default:
					SetReceiveString("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
					SetRunningState(tFalse);
			}

			if(GetReceiveString() != "ABCDEFGHIJKLMNOPQRSTUVWXYZ") {
				SetRunningState(tTrue);
			}

			if(debugModeEnabled) {
				LOG_WARNING(cString::Format("LightMessageReceiver: Activated with ActionCommand %d, String to receive: %s", actionSub_rec.command, GetReceiveString().GetPtr()));
			}
		}
	}
	else{
		if(GetRunningState()) {
			SetRunningState(tFalse);
		}
		//LOG_ERROR(cString::Format("LightMessageRec: Error occurred. Process Action was called, but actionSub command was disabled. Please check."));
	}
	RETURN_NOERROR;
}

tResult LightMessageReceiver::SetLastAction(tUInt32 received_action) {
    __synchronized_obj(criticalSectionLastActionAccess);

    last_received_action = received_action;
	RETURN_NOERROR;
}


tUInt32 LightMessageReceiver::GetLastAction() {
    __synchronized_obj(criticalSectionLastActionAccess);

	return last_received_action;
}

tResult LightMessageReceiver::SetRunningState(tBool received_state) {
    __synchronized_obj(criticalSectionRunningStateAccess);

    carPosition = Point(0,0);
    runningState = received_state;
	RETURN_NOERROR;
}

tBool LightMessageReceiver::GetRunningState() {
    __synchronized_obj(criticalSectionRunningStateAccess);

	return runningState;
}

tResult LightMessageReceiver::SetReceiveString(cString receive_string_tmp) {
    __synchronized_obj(cCritialSectionReceiveStringAccess);

    receive_string = receive_string_tmp;
	RETURN_NOERROR;
}

cString LightMessageReceiver::GetReceiveString() {

    __synchronized_obj(cCritialSectionReceiveStringAccess);

	return receive_string;
}

tResult LightMessageReceiver::ProcessReceivedChar(cString received_morse_char) {

	__synchronized_obj(criticalSectionStringAccess);

	for(tUInt32 i = 0; i <= 26; i++) {
		// compare received sign to morse code
		if(received_morse_char.CompareNoCase(codes.morse[i]) == 0) {
			cString received_str_tmp;
			received_str_tmp.Append(codes.alph[i]);
			if(received_str_tmp.CompareNoCase(" ") == 0) {
				if(debugModeEnabled) {
					LOG_WARNING(cString::Format("LightMessageRec: Skipped blank space"));
				}
			} else {
				received_string.Append(codes.alph[i]);
				if(debugModeEnabled) {
					LOG_WARNING(cString::Format("LightMessageRec: received character %s, all: %s, i: %d, awaiting: %s", received_str_tmp.GetPtr(), received_string.GetPtr(), i, GetReceiveString().GetPtr()));
				}
			}
			break;
		}
	}

	cString tmp_string_to_receive = GetReceiveString();

	// feedback
	TFeedbackStruct::Data tmp_feedback;
	tmp_feedback.ui32_filterID = F_LIGHT_MESSAGE_RECEIVER;

	// check if right string received
	cString last_charakters = received_string.Right(tmp_string_to_receive.GetLength());

	switch(GetLastAction()) {
		/* Slave Direction */
		case AC_LMR_RECEIVE_SLAVE_DIRECTION:
			// M (left), H (right), A (straight)
			// special case: multiple input strings and feedbacks required

			if(last_charakters.CompareNoCase("MMM") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_SLAVE_TURN_LEFT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			} else if (last_charakters.CompareNoCase("HHH") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_SLAVE_TURN_RIGHT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			} else if (last_charakters.CompareNoCase("AAA") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_SLAVE_STRAIGHT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			}
			break;

		/* Master Direction */
		case AC_LMR_RECEIVE_MASTER_DIRECTION:
			// N (left), O (right), I (straight)
			// special case: multiple input strings and feedbacks required

			if(last_charakters.CompareNoCase("NNN") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_MASTER_TURN_LEFT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			} else if (last_charakters.CompareNoCase("OOO") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_MASTER_TURN_RIGHT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			} else if (last_charakters.CompareNoCase("III") == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = FB_LMR_RECEIVE_MASTER_STRAIGHT;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());
			}
			break;

		/* Main case */
		default:
			if(last_charakters.CompareNoCase(tmp_string_to_receive) == 0) {
				SetRunningState(tFalse);
				tmp_feedback.ui32_status = GetLastAction() + 1;
				tFeedbackStruct_object.Transmit(&feedbackOutput, tmp_feedback, _clock->GetStreamTime());

				if(debugModeEnabled) {
					LOG_WARNING(cString::Format("LightMessageReceiver: Finished! Awaited string: %s, received string: %s", tmp_string_to_receive.GetPtr(), received_string.GetPtr()));
				}
			}
			break;
	}

	RETURN_NOERROR;
}

cString LightMessageReceiver::GetReceivedString() {

	__synchronized_obj(criticalSectionStringAccess);

	return received_string;

}


/* Performs/Calls all necessary steps to read video image from input */
tResult LightMessageReceiver::ProcessVideo(IMediaSample* sample) {
	/* Create new Mat matrix */
	Mat video_image(inputFormat.nHeight, inputFormat.nWidth, CV_8UC3);
	RETURN_IF_FAILED(GetInputVideoImage(video_image, sample));
	/* Original data is now referenced by 'video_image' of type Mat, but is still pointing
	 *  on the original input data! */

	double maxVal = 0, minVal = 0;
	Point minLoc, maxLoc;

	if(carPosition.x == 0 && carPosition.y == 0) {
		Rect expectedROI = Rect(0, 0, car.cols, 40);
		tInt32 heightOffset = 100;
		carROI = Rect(0, heightOffset, 440, 480 - heightOffset);
		matchTemplate(video_image(carROI), car, matchResult, CV_TM_SQDIFF);
		minMaxLoc(matchResult, &minVal, &maxVal, &carPosition, &maxLoc);

		carROI = Rect(carPosition.x - 10,carPosition.y + heightOffset + 25, expectedROI.width, expectedROI.height);

		carROI &= Rect(0,0, inputFormat.nWidth, inputFormat.nHeight);
		if(carROI.width < expectedROI.width || carROI.height < expectedROI.height) {
			carROI = Rect(0, heightOffset, 440, 480 - heightOffset);
		}
	}

	carROI = Rect(70, 120, 170, 120); // tmp: fix car ROI

	counter_images++; // tmp, count image samples TODO remove

	if(counter_images > 2000) {
		counter_images = 0;
	}

	if(saved_diffimage && (carROI.width != last_received_gray.cols || carROI.height != last_received_gray.rows)) {
		saved_diffimage = tFalse;
		RETURN_NOERROR;
	}

	// save differential image, last_reveived_gray valid?
	if(!saved_diffimage) {
		cv::cvtColor( video_image(carROI), last_received_gray, CV_RGB2GRAY);
		last_received_gray.convertTo(last_received_gray, CV_16SC1);
		saved_diffimage = tTrue;
		RETURN_NOERROR;
	}



	Mat diff_image(inputFormat.nHeight, inputFormat.nWidth, CV_16SC1);
	Mat gray_image;

	// convert RGB to gray image
	cv::cvtColor( video_image(carROI), gray_image, CV_RGB2GRAY);

	// diff image: signed image necessary to differentiate between rising/falling
	//cv::absdiff(gray_image, last_received_gray, diff_image); // abs diff

	gray_image.convertTo(gray_image,CV_16SC1); // convert gray image to signed image
	last_received_gray.convertTo(last_received_gray,CV_16SC1); // convert last received gray image to signed image
	cv::subtract(last_received_gray, gray_image, diff_image); // signed differential image

	tUInt32 count_pos = 0;
	tUInt32 count_neg = 0;

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
		Mat out(inputFormat.nHeight, inputFormat.nWidth, CV_8UC1);
		cv::cvtColor(video_image, out, CV_RGB2GRAY);
	#endif

	// count positive and negative pixels
	for (tUInt32 y = 0; y < static_cast<tUInt32>(diff_image.rows); y++) {
		for (tUInt32 x = 0; x < static_cast<tUInt32>(diff_image.cols); x++)
		{
			 tInt16 pixel_value = diff_image.at<tInt16>(y, x);
			 if(pixel_value < -(tInt16)pixel_threshold) count_pos++;
			 if(pixel_value > (tInt16)pixel_threshold) count_neg++;
			#ifdef DEBUG_MODE_OUTPUT_VIDEO
			 out.at<tInt8>(carROI.y + y, carROI.x + x) = abs(diff_image.at<tInt16>(y, x));
			#endif
		}
	}

#ifdef DEBUG_MODE_OUTPUT_VIDEO
	cv::rectangle(out, carROI, Scalar(255),2, 8);
#endif

	tTimeStamp timediff = 0;
	tFloat32 timediff_in_ms = 0;

	// rising edge
	if(count_pos > min_changed_pixel && count_pos < max_changed_pixel && !light_status) {
		light_status = tTrue;
		if(!timer_running) {
			start_time = _clock->GetStreamTime();
			timer_running = tTrue;
			#ifdef DEBUG_MODE_WRITE_IMAGES
				putText(out, cv::String(cString::Format("Started on rising edge, light status: %d", timediff_in_ms, light_status).GetPtr()),
					Point2f(20, 65), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
			#endif
			if(debugModeEnabled) {
				LOG_WARNING("LightMessageRec: first timer started on rising edge");
			}
		} else {
			timediff = (_clock->GetStreamTime() - start_time);
			timediff_in_ms = timediff / 1000;
			start_time = _clock->GetStreamTime();
			if(timediff_in_ms < dit_time) {
				if(debugModeEnabled) {
					//LOG_WARNING("LightMessage: symbol space");
				}
			} else if(timediff_in_ms < dah_time) {
				if(debugModeEnabled) {
					LOG_WARNING("LightMessage: letter space");
					//LOG_WARNING(received_char.c_str());
				}

				#ifdef DEBUG_MODE_WRITE_IMAGES
					putText(out, cv::String(cString::Format("Received letter space, timer: %f, light status: %d", timediff_in_ms, light_status).GetPtr()),
						Point2f(20, 65), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
				#endif

				ProcessReceivedChar(received_char);
				received_char.Clear();

			} else if(timediff_in_ms < wordspace_time) {
				if(debugModeEnabled) {
					LOG_WARNING("LightMessageRec: word space");
				}

				#ifdef DEBUG_MODE_WRITE_IMAGES
					putText(out, cv::String(cString::Format("Received word space, timer: %f, light status: %d", timediff_in_ms, light_status).GetPtr()),
							Point2f(20, 65), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
				#endif

				ProcessReceivedChar(received_char);
				ProcessReceivedChar(" ");
				received_char.Clear();
			}
		}
	// falling edge
	} else if(count_neg > min_changed_pixel && count_neg < max_changed_pixel && light_status) {
		light_status = tFalse;
		if(!timer_running) {
			start_time = _clock->GetStreamTime();
			timer_running = tTrue;
			if(debugModeEnabled) {
				LOG_WARNING("LightMessageRec: first timer started on falling edge");
			}
		} else {
			timediff = (_clock->GetStreamTime() - start_time);
			timediff_in_ms = timediff / 1000;
			start_time = _clock->GetStreamTime();
			if(timediff_in_ms < dit_time) {
				if(debugModeEnabled) {
					//LOG_WARNING("LightMessageRec: dit");
				}
				#ifdef DEBUG_MODE_WRITE_IMAGES
					putText(out, cv::String(cString::Format("Received dit, timer: %f, light status: %d", timediff_in_ms, light_status).GetPtr()),
							Point2f(20, 65), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
				#endif
				received_char.Append('.');
			} else if(timediff_in_ms < dah_time) {
				if(debugModeEnabled) {
					//LOG_WARNING("LightMessageRec: dah");
				}
				#ifdef DEBUG_MODE_WRITE_IMAGES
					putText(out, cv::String(cString::Format("Received dah, timer: %f, light status: %d", timediff_in_ms, light_status).GetPtr()),
						Point2f(20, 65), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
				#endif
				received_char.Append('-');
			}
		}
	}

	// save last received gray image
	gray_image.copyTo(last_received_gray);

	/*
	///////////////////////////
	// legacy, replaced with signed diff image!
	cv::threshold(diff_image, diff_image, 120, 255, cv::THRESH_BINARY); 	// threshold
	tUInt32 nonzero = (tUInt32)cv::countNonZero(diff_image); 	// count zero pixels in diff image

	if(0 && nonzero > 5 && nonzero < 100) {
		std::basic_string<char> filename = (std::basic_string<char>)cString::Format("/tmp/image_%d.jpg",counter_images);
		//imwrite( filename, video_image );
	}

	//if(nonzero > 5 && nonzero < 200) LOG_WARNING(cString::Format("LightMessage: non zero pixels: %d",nonzero));
	//LOG_WARNING(cString::Format("LightMessage: non zero pixels: %d, start_time: %d, status: %d, pos: %d, neg: %d, timediff: %f", nonzero, counter_images, light_status, count_pos, count_neg, timediff_in_ms));

	*/

	#ifdef DEBUG_MODE_WRITE_IMAGES
		putText(out, cv::String(cString::Format("Frame: %d, Count pos: %d, count neg: %d, status: %d", counter_images, count_pos, count_neg, light_status).GetPtr()),
				Point2f(20, 25), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));
		putText(out, cv::String(cString::Format("String: %s, awaited string: %s", GetReceivedString().GetPtr(), GetReceiveString().GetPtr()).GetPtr()),
				Point2f(20, 45), CV_FONT_NORMAL, 0.4, Scalar(255, 255, 255));

		std::basic_string<char> filename = (std::basic_string<char>)cString::Format("/tmp/image_%d.jpg",counter_images);
		imwrite( filename, out );
	#endif

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	/* Create output header for showing debug media stream on video output */
	//Mat out(inputFormat.nHeight, inputFormat.nWidth, CV_8UC3);
	//diff_image.convertTo(last_received_gray,CV_8UC3);
	//out = diff_image;
	//generate output
	cObjectPtr<IMediaSample> videoSample;
	if (IS_OK(AllocMediaSample(&videoSample))) {
		tBitmapFormat outputFormat = inputFormat;
		outputFormat.nWidth = out.cols;
		outputFormat.nHeight = out.rows;
		outputFormat.nBitsPerPixel = out.channels() * 8;
		outputFormat.nBytesPerLine = out.cols * out.channels();
		outputFormat.nSize = outputFormat.nBytesPerLine * out.rows;

		if (out.type() == CV_8UC3) {
			outputFormat.nPixelFormat = cImage::PF_RGB_888;
		}

		videoSample->Update(sample->GetTime(), out.data, tInt32(outputFormat.nSize), 0);

		outputVideoPin.SetFormat(&outputFormat, NULL);
		outputVideoPin.Transmit(videoSample);
	}
	#endif

	RETURN_NOERROR;
}

tResult LightMessageReceiver::GetInputVideoImage(cv::Mat &image, IMediaSample* sample) {
	if (inputFormat.nPixelFormat != cImage::PF_RGB_888) {
		RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED, "LightMessageRec: Only 8 bit RGB images are supported.");
	}
	/* Create buffer-pointer to set on data of input stream and create temporary header for referencing to data */
	const tVoid* srcBuffer;
	IplImage* img = cvCreateImageHeader(cvSize(inputFormat.nWidth, inputFormat.nHeight), IPL_DEPTH_8U, 3);
	RETURN_IF_FAILED(sample->Lock(&srcBuffer));
	img->imageData = (char*) srcBuffer;
	/* set reference onto data to 'image' (method argument)*/
	image = cvarrToMat(img); //convert to Mat-type
	cvReleaseImage(&img);	// release img to decrease reference-counter, expressing img-reference will not be needed anymore
	sample->Unlock(srcBuffer); //unlock buffer again

	/*IMPORTANT NOTICE: During this procedure, data is NOT copied to a new memory location nor is it duplicated!
	 * Only a new reference (new header) is now used to express the location of the original data!  (managed via a reference-counting mechanism of opencv) */

	RETURN_NOERROR;
}
