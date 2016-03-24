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
 * $Author:: schoen $   $Date:: 2016-02-17 #$
 * $Author:: fink $   $Date:: 2016-03-11 #$
 **********************************************************************/

#ifndef ROADSIGNDETECTIONADAPTER_H_
#define ROADSIGNDETECTIONADAPTER_H_

#define OID_ADTF_ROADSIGNEDETECTIONFILTER "adtf.aadc.roadSignDetection"


/*!
This filter searches for Aruco markers in the given frame on the input video pin. It uses mainly the Aruco lib to find the markers and get the marker Id and their additional parameters and mark them optionally in the video frame. If one or more markers are detected in the frame samples on the pins RoadSign and RoadSign_ext are generated containing the parameters of the sign. 
The samples of the pin RoadSign include the marker identifier and the image size of the marker. The samples of the pint RoadSign_ext include the marker identifier, the image size of the marker and the rotation and translation vector. For further description of the samples refer to chapter 3.5.

For the calculation of the rotation and translation vector the intrinsic parameters of the camera has to be known. These parameters can be get with the camera calibration filter and the calibration file generated by that filter loaded with in the  property Calibration File for used Camera. 
For the correct assignment of the marker IDs the dictionary file has to be set in the properties as well. The dictionary file includes the assignment of the bit codes of the markers to the predefined ID and has the following content:
%YAML:1.0
nmarkers: 12
markersize: 3
marker_0: "010010111"
marker_1: "001100010"
marker_2: "001001111"
marker_3: "110011101"
marker_4: "011001010"
marker_5: "110101111"
marker_6: "000111101"
marker_7: "011101000"
marker_8: "001010101"
marker_9: "010011001"
marker_10: "110011010"
marker_11: "010010100"

The IDs are mapped to the signs as the following table shows:
 MARKER_ID_UNMARKEDINTERSECTION	0
 MARKER_ID_STOPANDGIVEWAY	1
 MARKER_ID_PARKINGAREA	2
 MARKER_ID_HAVEWAY	3
 MARKER_ID_AHEADONLY	4
 MARKER_ID_GIVEWAY	5
 MARKER_ID_PEDESTRIANCROSSING	6
 MARKER_ID_ROUNDABOUT	7
 MARKER_ID_NOOVERTAKING	8
 MARKER_ID_NOENTRYVEHICULARTRAFFIC	9
 MARKER_ID_ONEWAYSTREET 	11

The named are enums are also contained in the file aadc_roadSign_enums.h in src\aadcDemo\include\aadc_roadSign_enums.h .
*/

#include <vector>
#include <deque>
#include "TSignalValue.h"
#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "ScmCommunication.h"
#include "RoadSign.h"
#include "TTrafficSign.h"

using namespace adtf;

class RoadSignDetectionAdapter : public adtf::cAsyncDataTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ROADSIGNEDETECTIONFILTER, "RoadSign", adtf::OBJCAT_DataFilter, "RoadSign", 1, 0, 1, "FAUtonOHM");
public:

    RoadSignDetectionAdapter(const tChar* __info);
    virtual ~RoadSignDetectionAdapter();
    tResult Init(tInitStage eStage, __exception);    
    tResult OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* mediaSample);
    tResult OnAsyncPinEvent(adtf::IPin* pSource, tInt nEventCode,tInt nParam1,tInt nParam2,adtf::IMediaSample* pMediaSample);
    tResult Shutdown(tInitStage eStage, __exception = NULL);

protected:
    /*! input Pin for video */
    cVideoPin videoInput;
    /*! output Pin for video */
    cVideoPin videoOutput;

    /*! output Pin for detected Sign as tInt */
    cOutputPin trafficSignOutput;

    /*!  */
    cInputPin steeringInput;

    /*! */
    cInputPin speedInput;

    /*! */
    cInputPin actionInput;

    /*! */
    cOutputPin feedbackOutput;


private:
    /*! bitmapformat of input image */    
    tBitmapFormat      inputFormat;                        

    /*! bitmapformat of output image */    
    tBitmapFormat      outputFormat;

    /*! bitmapformat of reduced output image */
    tBitmapFormat reducedOutputFormat;

    /*! */
    TTrafficSign tTrafficSign;

    /*! */
    TSignalValue tSignalValueSteering;

    /*! */
    TSignalValue tSignalValueSpeed;

    /*! */
    TActionStruct tActionStruct;

    /*! */
    TFeedbackStruct tFeedbackStruct;

    /*! indicates whether information is printed to the console or not */
    tBool debugModeEnabled;
    tBool debugExecutionTime;

    /*! indicates whether the camera parameters are loaded or not */
    tBool cameraParamsLoaded;

    /*! indicates what is transmitted over the output pin*/
    tBool debugMode;
    tBool m_bReduceImageResolution;

    /*! the aruco elements: */
    /*! the aruco detector for the markers*/
    aruco::MarkerDetector markerDetector;
    /*! the aruco markers */
    std::vector<aruco::Marker> cacheMarkers;
    /*! size of the markers*/
    tFloat32 markerSize;
    /*! the intrinsic parameter of the camera*/
    aruco::CameraParameters cameraParameters;
    aruco::CameraParameters cameraParameters_downsampled;
    /*! the dictionary for the aruco lib*/
    aruco::Dictionary dictionary;

    cv::Mat intrinsics;
    cv::Mat distortion;

    tFloat32 cameraPitch;

    struct Thresholds {
		/**
		 * in RAD
		 */
		tFloat32 COS_ANGLE_LIMIT_XY;

		/**
		 * in RAD
		 */
		tFloat32 COS_ANGLE_UPPER_LIMIT_Z;

		/**
		 * in RAD
		 */
		tFloat32 COS_ANGLE_LOWER_LIMIT_Z;

		/**
		 *
		 */
		tFloat32 CAMERA_Y_LIMIT;

		/**
		 * in meter
		 */
		tFloat32 MAX_DISTANCE;

		/**
		 * in degrees
		 */
		tFloat32 STEERING_LIMIT;
    } thresholds;

    /**
     * speed in meter per second
     */
	tFloat32 lastSpeed;

    /**
     * between -30 and 30 degree. Unit is RAD.
     */
    tFloat32 lastSteeringAngle;

    /**
     * last detected priority sign
     */
    TFeedbackStruct::Data lastPrioritySign;

	/**
	 * filename where pitch angle and offset is stored
	 */
	cFilename filePitch;

	TActionStruct::ActionSub actionSub;

	tFloat32 ms_sum; // Debugging: overall execution time (aruco detection)
	tUInt32 number_of_signs; // Debugging: count number of detected markers
	tUInt32 frame_counter; // Debugging: received video frame counter

	tUInt32 same_sign_counter; // sign has to be detected multiple times to be valid
	tInt32 prev_closest_sign; // save id of last detected nearest sign

	tBool parking_sign_received; // save parking sign status

	struct Sync {
		/**
		 * Synchronization of OnPinEvent
		 */
		cCriticalSection criticalSection_OnPinEvent;

		ucom::cObjectPtr<IMediaSample> lastActionMediaSample;

		int bufferCount;
		bool skipNextFrame;
		bool log_warning;
		bool log_console;
		const char* name;

		Sync() {
			bufferCount = 0;
			skipNextFrame = false;
			log_warning = false;
			log_console = false;
			name = "RoadSign";
			lastActionMediaSample = 0;
		}

		//helper
		inline void PrintCleared() {
			if(log_warning)
				LOG_WARNING(cString::Format("%s: cleared image queue", name));
			if(log_console)
				std::cout << name << ":cleared image queue" << std::endl;
		}

		inline void DecreaseBufferCount() {
			bufferCount--;
			if(log_warning)
				LOG_WARNING(cString::Format("%s: processed image buffer count = %d", name, bufferCount));

			if(log_console)
				std::cout << name << ": processed image buffer count = " << bufferCount << std::endl;

			if(bufferCount < 0) {
				bufferCount = 0;
				LOG_WARNING(cString::Format("%s: image queue buffer count < 0", name));
			}
		}

		inline void IncreaseBufferCount() {
			bufferCount++;
			if(log_warning)
				LOG_WARNING(cString::Format("%s: image queue buffer count = %d", name, bufferCount));

			if(log_console)
				std::cout << name << ": image queue buffer count = " << bufferCount << std::endl;
		}
	} sync;

	tResult ProcessAction(IMediaSample* mediaSample);

    /*! function process the video data
    @param pSample the new media sample to be processed
    */    
    tResult ProcessVideo(IMediaSample* pSample);

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    @param pFormat the new format for the input and input pin
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the m_output image format
    @param pFormat the new format for the output pin
    */
    tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);

    tResult ReadFromFile(tFloat32 *pitch);

    /**
     * TODO: this is disabled
     * @param sign
     * @param steeringAngle
     * @param speed
     * @return
     */
    tBool MatchToTrajectory(RoadSign sign, tFloat32 steeringAngle, tFloat32 speed);
};
#endif //ROADSIGNDETECTIONADAPTER_H_
