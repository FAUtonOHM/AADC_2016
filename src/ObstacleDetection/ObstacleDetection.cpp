/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements an obstacle detection based on a received depth image from xtion camera
**********************************************************************
* $Author:: mahill $   $Date:: 2016-03-17 23:47:07#$ $Rev:: 0.4.0   $
**********************************************************************/

#include "ObstacleDetection.h"

/* Focal length and center points of camera */
#define OD_CAMERA_FX 280.610668533f
#define OD_CAMERA_FY 282.969891862f
#define OD_CAMERA_CX 162.0f
#define OD_CAMERA_CY 112.0f

/* macro for size of CV_16UC1 for validation image*/
#define OD_VI_CV8UC1_MAX_RANGE 255 // (256 -1 )
#define OD_VI_CV16UC1_MAX_RANGE 65535 // (256*256 - 1)

/* IMPORTANT: also check DEBUG_MODE_OUTPUT_VIDEO in header file */

//#define DEBUG_TO_FILE_POINTCLOUD // log second generated point cloud to file /tmp/point_cloud.dat
//#define DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED // log second generated point cloud to file /tmp/point_cloud_transformed.dat

/* Regions of interest */
/* INTERSECTION */
/* Region for oncoming traffic at intersection*/
#define OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X 1.10f
#define OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y 0.6f
#define OD_ROI_INTERSECTION_ONCOMING_HEIGHT_X 0.6f		// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_INTERSECTION_ONCOMING_WIDTH_Y -0.3f		// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for cross-traffic from right hand side at intersection */
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X 0.6f
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y -0.2f
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X 0.35f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y -0.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* OVERTAKING */
/* Region for oncoming traffic during drive-mode */
#define OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X 0.3f
#define OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y 0.55f
#define OD_ROI_OVERTAKE_ONCOMING_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_ONCOMING_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on original lane during overtake, right before going back into original lane */
#define OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X 0.3f
#define OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y -0.27f
#define OD_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT*/
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X 0.05f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y 0.15f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT LEFT HALF ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF*/
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_X 0.05f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y 0.35f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* REGIONS RIGHT BEFORE PARKING OR PULL OUT MANEUVERS */
/* Region for traffic on oncoming lane that needs to be checked before starting cross-parking maneuver */
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X 0.1f
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y 0.45f
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X 0.7f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y -0.35f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on lane in front that needs to be checked before starting pull out from cross-parking maneuver */
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X 0.1f
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y 0.7f
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X 0.65f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y -0.7f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

ADTF_FILTER_PLUGIN("ObstacleDetection", OID_ADTF_OBSTACLE_DETECTION, ObstacleDetection);

#define OD_PITCH_STORAGE "Camera parameters::Camera rotation::pitch angle file"
#define OD_CAMERA_PITCH "Camera parameters::Camera rotation::pitch"
#define OD_USE_PITCH_FROM_FILE "Camera parameters::Camera rotation::use pitch angle from file"
#define OD_CAMERA_PITCH_CORRECTION "Camera parameters::Camera rotation::pitch correction factor"

#define OD_CAM_POSITION_OFFSET_X "Camera parameters::Camera position::offset in x direction"
#define OD_CAM_POSITION_OFFSET_Y "Camera parameters::Camera position::offset in y direction"
#define OD_CAM_POSITION_OFFSET_Z "Camera parameters::Camera position::offset in z direction"
#define OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB "Camera parameters::Camera position::offset between depth-cam and rgb-cam"

//#define OD_CAR_CARWIDTH "Field of view::width of car"
#define OD_CAR_MAX_FIELD_OF_VIEW_X "Check ROI::Field of view::x:maximal field of view in x-direction (distance)"
#define OD_CAR_MIN_FIELD_OF_VIEW_X "Check ROI::Field of view::x:minimal field of view in x-direction (distance)"
#define OD_CAR_MAX_FIELD_OF_VIEW_Z "Check ROI::Field of view::z:maximal field of view in z-direction (height)"
#define OD_CAR_MIN_FIELD_OF_VIEW_Z "Check ROI::Field of view::z:minimal field of view in z-direction (height)"

#define OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z "Validation Mask::Max 'object height' for lines regarded 'valid'"

#define OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND "Check ROI::Intersection::lower bound of 'free-frames'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND "Check ROI::Intersection::upper bound of accepted hits for 'cross-traffic-right'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Intersection::upper bound of accepted hits for 'oncoming traffic'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Intersection::lower bound of 'occupied-frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND "Check ROI::Overtake::straight in front::lower bound of 'free-frames' for static obstacle in front"
#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND "Check ROI::Overtake::straight in front::upper bound of accepted hits for static obstacle in front"
#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Overtake::straight in front::lower bound for 'occupied frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND "Check ROI::Overtake::oncoming lane::lower bound of 'free-frames'"
#define OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Overtake::oncoming lane::upper bound of accepted hits for 'oncoming traffic'"
#define OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Overtake::oncoming lane::lower bound for 'occupied frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND "Check ROI::Cross Parking::pull-out: upper bound of accepted hits for 'cross-traffic'"
#define OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Cross Parking::oncoming: upper bound of accepted hits for 'oncoming-traffic'"
#define OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Cross Parking::lower bound for 'occupied frames' for 'static-obstacle-status'"
#define OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND "Check ROI::Cross Parking::pull-out: lower bound of 'free-frames'"
#define OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND "Check ROI::Cross Parking::oncoming: lower bound of 'free-frames'"

#define OD_DEBUG_TYPE "Check ROI::Debug::Debug Type"

#define OD_OACC_PAR_WHEELBASE "OpticalACC::parameters::wheelbase"
//#define OD_OACC_PAR_CARWIDTH "OpticalACC::parameters::carwidth"
#define OD_OACC_PAR_AXISTOBUMPER_FRONT "OpticalACC::parameters::distance front axis to front bumper"
#define OD_OACC_PAR_OBLIQUEANGLE_FRONT "OpticalACC::parameters::oblique angle front"
// #define OD_OACC_PAR_OBLIQUEANGLE_REAR "OpticalACC::parameters::oblique angle rear" // not yet in use
#define OD_OACC_CONFIG_XML_FILE "OpticalACC::parameters::xml-configuration-file speed-reduction"

#define OD_OACC_ROI_SEGMENTLINE "OpticalACC::ROI::length of segment-line (check-distance)"
#define OD_OACC_ROI_Y_SEARCHTOLERANCE "OpticalACC::ROI::width/2 of search-corridor in y-direction"
#define OD_OACC_ROI_HEIGHTTHRESHOLD_MIN "OpticalACC::ROI::minimal obstacle height threshold"
#define OD_OACC_ROI_HEIGHTTHRESHOLD_MAX "OpticalACC::ROI::maximal obstacle height threshold"

#define OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION "OpticalACC::OccupancyGrid::Resolution of gridcell element"
#define OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD "OpticalACC::OccupancyGrid::Counter-threshold for gridcell to be seen as occupied"
#define OD_OACC_OCCUPANCY_GRID_DEBUG_LOG "OpticalACC::OccupancyGrid::Debug messages"

#define OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC "OpticalACC::Debug::enable output dynamic driving lane as binary"
#define OD_OACC_DEBUG_INFO "OpticalACC::Debug::log pixel-count to console"
#define OD_OACC_DEBUG_XML_BORDER_WARNING "OpticalACC::Debug::log xml border warning"
#define OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE "OpticalACC::Debug::print xml table"
#define OD_GCL_DEBUG_INFO "OpticalACC::Debug::GCL Debug output"
#define OD_GCL_DEBUG_INFO_LOG "OpticalACC::Debug::GCL Debug Log"

#define OD_DEBUG_TO_CONSOLE "Debug::Debug Output to Console"
#define OD_EXTENDED_DEBUG_TO_CONSOLE "Debug::Extended Debug Output to Console"




ObstacleDetection::ObstacleDetection(const tChar* __info) : cAsyncDataTriggeredFilter(__info) {

	SetPropertyStr(OD_PITCH_STORAGE, "./../../../../../Camera_Pitch.txt");
	SetPropertyBool(OD_PITCH_STORAGE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(OD_PITCH_STORAGE NSSUBPROP_DESCRIPTION,"Location of the file which is used for storing the pitch angle.");

	SetPropertyBool(OD_USE_PITCH_FROM_FILE, tTrue);
	SetPropertyStr(OD_USE_PITCH_FROM_FILE NSSUBPROP_DESCRIPTION, "If enabled, the camera pitch angle stored in a file will be used.");
	usePitchFromFile = tTrue;

	SetPropertyFloat(OD_CAMERA_PITCH, -10);
	SetPropertyFloat(OD_CAMERA_PITCH NSSUBPROP_MAX, 0);
	SetPropertyFloat(OD_CAMERA_PITCH NSSUBPROP_MIN, -20);
	SetPropertyStr(OD_CAMERA_PITCH NSSUBPROP_DESCRIPTION, "Pitch angle of the camera in degree.");

	SetPropertyFloat(OD_CAMERA_PITCH_CORRECTION, 1);
	SetPropertyFloat(OD_CAMERA_PITCH_CORRECTION NSSUBPROP_MIN, 0);
	SetPropertyFloat(OD_CAMERA_PITCH_CORRECTION NSSUBPROP_MAX, 3);
	SetPropertyStr(OD_CAMERA_PITCH_CORRECTION NSSUBPROP_DESCRIPTION, "Pitch angle correction factor in degree. Will be ADDED to pitch (pitch + cor_factor).");

	SetPropertyFloat(OD_CAM_POSITION_OFFSET_X, 0.2);
	SetPropertyStr(OD_CAM_POSITION_OFFSET_X NSSUBPROP_DESCRIPTION, "X-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive x-direction.");

	SetPropertyFloat(OD_CAM_POSITION_OFFSET_Y, -0.047);
	SetPropertyStr(OD_CAM_POSITION_OFFSET_Y NSSUBPROP_DESCRIPTION, "Y-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive y-direction.");

	SetPropertyFloat(OD_CAM_POSITION_OFFSET_Z, -0.22);
	SetPropertyStr(OD_CAM_POSITION_OFFSET_Z NSSUBPROP_DESCRIPTION, "Z-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive z-direction.");

	SetPropertyFloat(OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB, 0.04);
	SetPropertyStr(OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB NSSUBPROP_DESCRIPTION, "Horizontal offset of the depth-camera-center from the rgb-camera-center in METER.");

//	SetPropertyFloat(OD_CAR_CARWIDTH, 0.40);
//	SetPropertyStr(OD_CAR_CARWIDTH NSSUBPROP_DESCRIPTION, "Width of the car in meter.");

	SetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_X, 2.0);
	SetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_X NSSUBPROP_MAX, 2.5);
	SetPropertyStr(OD_CAR_MAX_FIELD_OF_VIEW_X NSSUBPROP_DESCRIPTION, "Maximal x-distance of points in METER that should be taken into account, from origin of local coordinate system.");

	SetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_X, 0.05);
	SetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_X NSSUBPROP_MIN, 0);
	SetPropertyStr(OD_CAR_MIN_FIELD_OF_VIEW_X NSSUBPROP_DESCRIPTION, "Minimal x-distance of points in METER that should be taken into account, from origin of local coordinate system.");

	SetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_Z, 0.4);
	SetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_Z NSSUBPROP_MAX, 1.5);
	SetPropertyStr(OD_CAR_MAX_FIELD_OF_VIEW_Z NSSUBPROP_DESCRIPTION, "Maximal height of points in METER that should be taken into account, from origin of local coordinate system.");

	SetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_Z, 0.05);
	SetPropertyStr(OD_CAR_MIN_FIELD_OF_VIEW_Z NSSUBPROP_DESCRIPTION, "Minimal height of points in METER that should be taken into account, from origin of local coordinate system.");

	SetPropertyFloat(OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z, 0.03);
	SetPropertyStr(OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z NSSUBPROP_DESCRIPTION, "Maximal height in METER that is still regarded as 'ground' (can contain lines), above the origin of origin of local coordinate system.");

	SetPropertyInt(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND,30);
	SetPropertyStr(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND NSSUBPROP_DESCRIPTION, "Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle.");

	SetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for 'cross-traffic-right roi' without counting area as obstacle.");


	SetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND,5);
	SetPropertyStr(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND NSSUBPROP_DESCRIPTION, "Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback.");


	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND,5);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND NSSUBPROP_DESCRIPTION, "Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle.");

	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND,15);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND NSSUBPROP_DESCRIPTION, "Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for static obstacle straight in front without counting area as obstacle.");

	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND,5);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND NSSUBPROP_DESCRIPTION, "Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND,5);
	SetPropertyStr(OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND NSSUBPROP_DESCRIPTION, "Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback.");


	/* cross parking and pull out afterwards */
	SetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle.");

	SetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND,100);
	SetPropertyStr(OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND NSSUBPROP_DESCRIPTION, "Maximal number of cloud points that are tolerated for 'cross traffic roi' without counting area as obstacle.");

	SetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND,5);
	SetPropertyStr(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND NSSUBPROP_DESCRIPTION, "Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND,15);
	SetPropertyStr(OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND NSSUBPROP_DESCRIPTION, "Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback.");

	SetPropertyInt(OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND,15);
	SetPropertyStr(OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND NSSUBPROP_DESCRIPTION, "Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback.");


	/* Optical ACC */
	SetPropertyFloat(OD_OACC_PAR_WHEELBASE, 0.37);
	SetPropertyFloat(OD_OACC_PAR_WHEELBASE NSSUBPROP_MIN, 0.3);
	SetPropertyFloat(OD_OACC_PAR_WHEELBASE NSSUBPROP_MAX, 0.45);
	SetPropertyStr(OD_OACC_PAR_WHEELBASE NSSUBPROP_DESCRIPTION, "Length of wheelbase of car in METER.");

//	SetPropertyFloat(OD_OACC_PAR_CARWIDTH, 0.26);
//	SetPropertyFloat(OD_OACC_PAR_CARWIDTH NSSUBPROP_MIN, 0.20);
//	SetPropertyFloat(OD_OACC_PAR_CARWIDTH NSSUBPROP_MAX, 0.35);
//	SetPropertyStr(OD_OACC_PAR_CARWIDTH NSSUBPROP_DESCRIPTION, "Width of car in METER.");


	SetPropertyFloat(OD_OACC_PAR_AXISTOBUMPER_FRONT, 0.13);
	SetPropertyFloat(OD_OACC_PAR_AXISTOBUMPER_FRONT NSSUBPROP_MIN, 0.00);
	SetPropertyFloat(OD_OACC_PAR_AXISTOBUMPER_FRONT NSSUBPROP_MAX, 0.20);
	SetPropertyStr(OD_OACC_PAR_AXISTOBUMPER_FRONT NSSUBPROP_DESCRIPTION, "Distance between front axis and front bumper-end in METER.");

	SetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_FRONT, 0.0);
	SetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_FRONT NSSUBPROP_MAX, 0.26);
	SetPropertyStr(OD_OACC_PAR_OBLIQUEANGLE_FRONT NSSUBPROP_DESCRIPTION, "Oblique angle of front wheels in GRAD.");

	//SetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_REAR, 0.0);
	//SetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_REAR NSSUBPROP_MAX, 0.26);
	//SetPropertyStr(OD_OACC_PAR_OBLIQUEANGLE_REAR NSSUBPROP_DESCRIPTION, "Oblique angle of rear wheels.");

	SetPropertyStr(OD_OACC_CONFIG_XML_FILE, "../../../../utilities/ObstacleDetectionACC/ObstacleDetecACC.xml");
	SetPropertyBool(OD_OACC_CONFIG_XML_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(OD_OACC_CONFIG_XML_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr(OD_OACC_CONFIG_XML_FILE NSSUBPROP_DESCRIPTION, "The path of the XML, defining the behaviour of the OpticalACC, has to be set here.");

	SetPropertyFloat(OD_OACC_ROI_SEGMENTLINE, 1.30);
	SetPropertyFloat(OD_OACC_ROI_SEGMENTLINE NSSUBPROP_MIN, 0.30);
	SetPropertyFloat(OD_OACC_ROI_SEGMENTLINE NSSUBPROP_MAX, 2.00);
	SetPropertyStr(OD_OACC_ROI_SEGMENTLINE NSSUBPROP_DESCRIPTION, "Length of the circular segment-line to be checked in front of car (distance in METER), starting at front axis.");

	SetPropertyFloat(OD_OACC_ROI_Y_SEARCHTOLERANCE, 0.16);
	SetPropertyFloat(OD_OACC_ROI_Y_SEARCHTOLERANCE NSSUBPROP_MIN, 0.05);
	SetPropertyFloat(OD_OACC_ROI_Y_SEARCHTOLERANCE NSSUBPROP_MAX, 0.30);
	SetPropertyStr(OD_OACC_ROI_Y_SEARCHTOLERANCE NSSUBPROP_DESCRIPTION, "Half width of area in front of car to be checked, given in METER.");

	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN, 0.1);
	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN NSSUBPROP_MIN, -0.05);
	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN NSSUBPROP_MAX, 0.25);
	SetPropertyStr(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN NSSUBPROP_DESCRIPTION, "Minimal height an object can have to be regarded as obstacle, given in METER.");

	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX, 0.28);
	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX NSSUBPROP_MIN, 0.26);
	SetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX NSSUBPROP_MAX, 3.0);
	SetPropertyStr(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX NSSUBPROP_DESCRIPTION, "Maximal height an object can have to be regarded as obstacle, given in METER.");

	SetPropertyInt(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION, 5);
	SetPropertyStr(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION NSSUBPROP_VALUELISTNOEDIT, "1@Cell 2x2 (Image 160x120)|2@Cell 4x4 (Image 80x60)|3@Cell 5x5 (Image 64x48)|4@Cell 8x8 (Image 40x30)|5@Cell 10x10 (Image 32x24)|6@Cell 16x16(Image 20x15)|7@ Cell 20x20 (Image 16x12)");
	SetPropertyStr(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION NSSUBPROP_DESCRIPTION, "Resolution of the one gridcell element (in brackets 'downsampled' resolution of actual input depth image).");


	SetPropertyFloat(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD, 60.0);
	SetPropertyFloat(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD NSSUBPROP_MIN, 1.0);
	SetPropertyStr(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD NSSUBPROP_DESCRIPTION, "Occupancy-threshold: Minimum number of data-points per gridcell that gridcell is seen as occupied. DEPENDS on Resolution, has to be chosen accordingly!");

	SetPropertyBool(OD_OACC_OCCUPANCY_GRID_DEBUG_LOG,tFalse);
	SetPropertyStr(OD_OACC_OCCUPANCY_GRID_DEBUG_LOG NSSUBPROP_DESCRIPTION, "If enabled additional debug info concerning the OccupancyGrid will be printed to console (Warning: decreases performance).");
	debug_occupancyGrid = tFalse;

	SetPropertyBool(OD_OACC_DEBUG_INFO,tFalse);
	SetPropertyStr(OD_OACC_DEBUG_INFO NSSUBPROP_DESCRIPTION, "If enabled the count of pixels regarded as obstacle is printed to the console if GREATER than zero (Warning: decreases performance).");
	debugACCObstaclepixelcount = tFalse;

	SetPropertyBool(OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE,tFalse);
	SetPropertyStr(OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE NSSUBPROP_DESCRIPTION, "If enabled values specified by the xml-table are printed to the console during loading.");
	xml_PrintInitialtable = tFalse;

	SetPropertyBool(OD_OACC_DEBUG_XML_BORDER_WARNING,tFalse);
	SetPropertyStr(OD_OACC_DEBUG_XML_BORDER_WARNING NSSUBPROP_DESCRIPTION, "If enabled warnings are printed to console if requested value is out of xml-data-range (Warning: decreases performance).");
	xml_BorderWarningModeEnabled = tFalse;


	SetPropertyBool(OD_DEBUG_TO_CONSOLE,tFalse);
	SetPropertyStr(OD_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance).");
	debugModeEnabled = tFalse;

	SetPropertyBool(OD_EXTENDED_DEBUG_TO_CONSOLE,tFalse);
	SetPropertyStr(OD_EXTENDED_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional extended debug information is printed to the console (Warning: decreases performance).");
	extendeddebugModeEnabled = tFalse;

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
		SetPropertyInt(OD_DEBUG_TYPE, 4);
	    SetPropertyStr(OD_DEBUG_TYPE NSSUBPROP_VALUELISTNOEDIT, "1@all | 2@oncoming traffic | 3@crosstraffic right|4@no modification|5@oncoming overturn|6@back to original lane after overturn|7@static obstacle straight|8@parking cross oncoming|9@parking cross pull-out-check");
	    SetPropertyStr(OD_DEBUG_TYPE NSSUBPROP_DESCRIPTION, "Type that is used for debug output video.");
	#endif

	SetPropertyBool(OD_GCL_DEBUG_INFO,tFalse);
	SetPropertyStr(OD_GCL_DEBUG_INFO NSSUBPROP_DESCRIPTION, "If enabled the GCL-output is activated and info will be printed.");
	showGCLDebug = tFalse;

	SetPropertyBool(OD_GCL_DEBUG_INFO_LOG,tFalse);
	SetPropertyStr(OD_GCL_DEBUG_INFO_LOG NSSUBPROP_DESCRIPTION, "If enabled additional log info concerning GCL data will be printed.");
	showGCLDebug_extendedLog = tFalse;


	SetPropertyBool(OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC,tFalse);
	SetPropertyStr(OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC NSSUBPROP_DESCRIPTION, "If enabled, current driving corridor will be made available as binary image on output.");
	showOaccBinaryImageDebug = tFalse;





}

ObstacleDetection::~ObstacleDetection() {
	#ifdef DEBUG_TO_FILE_POINTCLOUD
	file_pointcloud.close();
	#endif
	#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
	file_pointcloud_trans.close();
	#endif
}

tResult ObstacleDetection::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("Action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	// Video Input
	RETURN_IF_FAILED(inputDepthImagePin.Create("Depth_Image", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&inputDepthImagePin));

	/* Create input pin for steeringAngle */
	RETURN_IF_FAILED(steeringAngleInput.Create("Steering_Angle", tSignalValueSteeringInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&steeringAngleInput));

	/* Create input pin for steeringAngle */
	RETURN_IF_FAILED(targetSpeedInput.Create("Target_Speed", tSignalValueSpeedInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&targetSpeedInput));

	RETURN_NOERROR;
}

tResult ObstacleDetection::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("Feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	// video output
	RETURN_IF_FAILED(outputVideoPin.Create("Debug_Region_Video", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&outputVideoPin));
	#endif

	// Video Valid output
	RETURN_IF_FAILED(outputVideoValidPin.Create("Binary_Valid_Image", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&outputVideoValidPin));

	// Video Debug OptACC output
	RETURN_IF_FAILED(outputDebugOptACCPin.Create("OptACC_Binary_Debug_Image", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
	RETURN_IF_FAILED(RegisterPin(&outputDebugOptACCPin));

	/* Create output pin for targetSpeed*/
	RETURN_IF_FAILED(targetSpeedOutput.Create("Mod_Target_Speed", tSignalValueOutput_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&targetSpeedOutput));

    //GLC Output
    cObjectPtr<IMediaType> pCmdType = NULL;
    RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
    RETURN_IF_FAILED(m_oGCLOutput.Create("GCL_Output",pCmdType, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

	RETURN_NOERROR;
}

tResult ObstacleDetection::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{

		#ifdef DEBUG_TO_FILE_POINTCLOUD
		// log file for point-cloud based on second frame of depth image
		LOG_WARNING("ObstacleDetec: debug mode: 'logging' for point-cloud enabled, writing to /tmp/point_cloud.dat");
		file_pointcloud.open("/tmp/point_cloud.dat", ios::out | ios::trunc);
		file_pointcloud << "# Point cloud data x, y, z\n";
		//file_pointcloud.close();
		#endif

		#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
		// log file for transformed point-cloud based on second frame of depth image
		LOG_WARNING("ObstacleDetec: debug mode: 'logging' for transformed point-cloud enabled, writing to /tmp/point_cloud_trans.dat");
		file_pointcloud_trans.open("/tmp/point_cloud_trans.dat", ios::out | ios::trunc);
		file_pointcloud_trans << "# Transformed Point cloud data x, y, z\n";
		//file_pointcloud_trans.close();
		#endif


		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSteeringInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSpeedInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueOutput_object.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));


	} else if (eStage == StageNormal) {

		firstFrame = tTrue;
		runningState = tFalse;

		#ifdef DEBUG_TO_FILE_POINTCLOUD
			pointcloud_to_file = tFalse;
		#endif
		#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
			transformed_pointcloud_to_file = tFalse;
			transformed_logging_completed = tFalse;
		#endif

		/* assign camera offset */
		camera_x_offset = GetPropertyFloat(OD_CAM_POSITION_OFFSET_X);
		camera_y_offset = GetPropertyFloat(OD_CAM_POSITION_OFFSET_Y);
		camera_z_offset = GetPropertyFloat(OD_CAM_POSITION_OFFSET_Z);

		camera_DepthToRgbOffest = GetPropertyFloat(OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB);

		/* assign car properties */
//		width_of_car = GetPropertyFloat(OD_CAR_CARWIDTH);
		distance_threshold_max = GetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_X);
		distance_threshold_min = GetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_X);
		height_threshold_max = GetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_Z);
		height_threshold_min = GetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_Z);

		/* assign height-property for validation of the rbg-image distinguishing 'ground/no-ground'*/
		height_threshold_validmask = GetPropertyFloat(OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z);


		/* Optical ACC */
		OACC_wheelbase = GetPropertyFloat(OD_OACC_PAR_WHEELBASE);
//		OACC_carWidth = GetPropertyFloat(OD_OACC_PAR_CARWIDTH);
		OACC_frontAxisToFrontBumper = GetPropertyFloat(OD_OACC_PAR_AXISTOBUMPER_FRONT);
		OACC_obliqueAngleFront = GetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_FRONT);
		OACC_circSegmentDistance = GetPropertyFloat(OD_OACC_ROI_SEGMENTLINE);
		OACC_ySearchTolerance = GetPropertyFloat(OD_OACC_ROI_Y_SEARCHTOLERANCE);
		OACC_obstacleHeightThreshold_min = GetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN);
		OACC_obstacleHeightThreshold_max = GetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX);

		OACC_GridCellResolutionIndicator = GetPropertyInt(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION);
		OACC_GridCellResolution = GetResolutionFromIndicator(OACC_GridCellResolutionIndicator);
		if(OACC_GridCellResolution == 0){
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_ARG,cString::Format("ObstacleDetec: Error occurred, gridcell resolution could not be loaded. Please check."));
		}
		OACC_GridCellOccupiedCounterThreshold = GetPropertyInt(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD);
		debug_occupancyGrid = GetPropertyBool(OD_OACC_OCCUPANCY_GRID_DEBUG_LOG);

		//Get path of configuration file for "parking-mode-ACC"
		xml_configFileOpticalACC = GetPropertyStr(OD_OACC_CONFIG_XML_FILE);
		xml_BorderWarningModeEnabled = GetPropertyBool(OD_OACC_DEBUG_XML_BORDER_WARNING);
		xml_PrintInitialtable = GetPropertyBool(OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE);

		curSteeringAngle.f32_value = 90.0f;
		curTargetSpeedLimit = 10.0f;

		/* initialize instances for regions of interest */
		/* INTERSECTION MODE*/
		/* OnComing Traffic at intersection */
		roi_intersection_onComingTraffic.bottomleft_corner.x = OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X;
		roi_intersection_onComingTraffic.bottomleft_corner.y = OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y;
		roi_intersection_onComingTraffic.roi_y_width = OD_ROI_INTERSECTION_ONCOMING_WIDTH_Y;
		roi_intersection_onComingTraffic.roi_x_height = OD_ROI_INTERSECTION_ONCOMING_HEIGHT_X;
		/* CrossTraffic from right hand side at intersection */
		roi_intersection_crossTrafficRight.bottomleft_corner.x = OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X;
		roi_intersection_crossTrafficRight.bottomleft_corner.y = OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y;
		roi_intersection_crossTrafficRight.roi_y_width = OD_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y;
		roi_intersection_crossTrafficRight.roi_x_height = OD_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X;

		/* OVERTURN MODE */
		/* OnComing Traffic during 'on-the-road-mode' */
		roi_overtake_onComingTraffic.bottomleft_corner.x = OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X;
		roi_overtake_onComingTraffic.bottomleft_corner.y = OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y;
		roi_overtake_onComingTraffic.roi_y_width = OD_ROI_OVERTAKE_ONCOMING_WIDTH_Y;
		roi_overtake_onComingTraffic.roi_x_height = OD_ROI_OVERTAKE_ONCOMING_HEIGHT_X;

		/* overturn: currently on oncoming lane, check 'own original' lane to go back during 'on-the-road-mode' */
		roi_overtake_originalLane.bottomleft_corner.x = OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X;
		roi_overtake_originalLane.bottomleft_corner.y = OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y;
		roi_overtake_originalLane.roi_y_width = OD_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y;
		roi_overtake_originalLane.roi_x_height = OD_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X;

		/* static obstacle in front of the car, straight direction */
		roi_overtake_ownLaneStraight.bottomleft_corner.x = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X;
		roi_overtake_ownLaneStraight.bottomleft_corner.y = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y;
		roi_overtake_ownLaneStraight.roi_y_width = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y;
		roi_overtake_ownLaneStraight.roi_x_height = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X;

		/* static obstacle in front left half of the car, straight direction */
		roi_overtake_ownLaneStraight_lefthalf.bottomleft_corner.x = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_X;
		roi_overtake_ownLaneStraight_lefthalf.bottomleft_corner.y = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y;
		roi_overtake_ownLaneStraight_lefthalf.roi_y_width = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y;
		roi_overtake_ownLaneStraight_lefthalf.roi_x_height = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X;

		/* PARKING MODE */
		/* overturn: currently on oncoming lane, check 'own original' lane to go back during 'on-the-road-mode' */
		roi_parking_cross_onComing.bottomleft_corner.x = OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X;
		roi_parking_cross_onComing.bottomleft_corner.y = OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y;
		roi_parking_cross_onComing.roi_y_width = OD_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y;
		roi_parking_cross_onComing.roi_x_height = OD_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X;

		/* static obstacle in front of the car, straight direction */
		roi_parking_cross_crossTraffic.bottomleft_corner.x = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X;
		roi_parking_cross_crossTraffic.bottomleft_corner.y = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y;
		roi_parking_cross_crossTraffic.roi_y_width = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y;
		roi_parking_cross_crossTraffic.roi_x_height = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X;


		/* read properties of tolerated values and necessary number of free frames */
		/* intersection mode */
		intrsc_freeFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND);
		freeFrameCounter = 0;

		intrsc_OccupiedCoordinateCounterUpperBound_crossRight = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND);
		OccupiedCoordinateCounter_crossRight = 0;
		intrsc_OccupiedCoordinateCounterUpperBound_onComing = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND);
		OccupiedCoordinateCounter_onComing = 0;
		intrsc_occupiedFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND);
		/* OVETAKE */

		ovrtk_freeFrameCounterLowerBound_oncoming = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND);
		ovrtk_OccupiedCoordinateCounterUpperBound_onComing = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND);
		overtake_oncoming_occupiedFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND);
		overtake_ownlane_occupiedFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND);
		/* static obstacle straight in front */
		ovrtk_freeFrameCounterLowerBound_ownLaneStraight = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND);
		ovrtk_OccupiedCoordinateCounterUpperBound_ownLaneStraight = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND);
		occupiedFrameCounter = 0;

		/*parking mode */
		crsPrkng_OccupiedCoordinateCounterUpperBound_crossRight = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND);
		crsPrkng_OccupiedCoordinateCounterUpperBound_onComing = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND);
		crsPrkng_occupiedFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND);
		crsPrkng_freeFrameCounterLowerBound_crossRight = GetPropertyInt(OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND);
		crsPrkng_freeFrameCounterLowerBound_oncoming = GetPropertyInt(OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND);

		/* read property whether debug mode is required */
		debugModeEnabled = GetPropertyBool(OD_DEBUG_TO_CONSOLE);
		extendeddebugModeEnabled = GetPropertyBool(OD_EXTENDED_DEBUG_TO_CONSOLE);
		#ifdef DEBUG_MODE_OUTPUT_VIDEO
		debugType = GetPropertyInt(OD_DEBUG_TYPE);
		#endif
		showGCLDebug = GetPropertyBool(OD_GCL_DEBUG_INFO);
		showGCLDebug_extendedLog = GetPropertyBool(OD_GCL_DEBUG_INFO_LOG);

		debugACCObstaclepixelcount = GetPropertyBool(OD_OACC_DEBUG_INFO);
		showOaccBinaryImageDebug = GetPropertyBool(OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC);

		/* get information about pitch of camera */
		filePitch = GetPropertyStr(OD_PITCH_STORAGE);
		usePitchFromFile = GetPropertyBool(OD_USE_PITCH_FROM_FILE);
		pitch = GetPropertyFloat(OD_CAMERA_PITCH);
		if(usePitchFromFile) RETURN_IF_FAILED(ReadFromFile(&pitch));
		pitchCorrection = GetPropertyFloat(OD_CAMERA_PITCH_CORRECTION);

		/* convert pitch from 'deg' to 'rad', taking correction factor into account */
		m_pitch_rad = ((pitch + pitchCorrection)/ 180.0f) * static_cast<tFloat32>(M_PI);
		cos_m_pitch_rad = cosf(m_pitch_rad);
		sin_m_pitch_rad = sinf(m_pitch_rad);
		if(debugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Pitch in degree %f, pitch in rad %f",pitch,m_pitch_rad));

		sin_m_dynRange_rad = sinf(-m_pitch_rad);
		cos_m_dynRange_rad = cosf(-m_pitch_rad);
		OACC_operational_status = OPT_ACC_ENABLED;

		#ifdef DEBUG_MODE_OUTPUT_VIDEO
		debugLogEventToggle = 0;
		#endif

		//load xml files for linear interpolation
		THROW_IF_FAILED(LoadConfigurationData(xml_configFileOpticalACC,oAcc_xml_xValues,oAcc_xml_yValues));

	} else if (eStage == StageGraphReady) {
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSteeringInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSpeedInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueOutput_object.StageGraphReady());
	}
	RETURN_NOERROR;
}

tResult ObstacleDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr) {

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}


tResult ObstacleDetection::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &inputDepthImagePin) {
			__synchronized_obj(criticalSection_VideoDataAccess);
			/* always activate filter and image processing since some functionalities like obstacle-detection in current driving lane and
			 *  validation of recognized lines for lanefollower (done by linespecifier) is running in reflex-mode (always on) */
		#ifdef DEBUG_MODE_OUTPUT_VIDEO
				TActionStruct::ActionSub actionSub_sim;
				actionSub_sim.enabled = tTrue;
				actionSub_sim.started = tTrue;
				switch(debugType){
					case 1:  actionSub_sim.command = AC_OD_INTERSECTION_CHECK_ALL;
							break;
					case 2: actionSub_sim.command = AC_OD_INTERSECTION_CHECK_ONCOMING_TRAFFIC;
							break;
					case 3: actionSub_sim.command = AC_OD_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT;
							break;
					case 4: break;
					case 5: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_ONCOMING_TRAFFIC;
							break;
					case 6: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE;
							break;
					case 7: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT;
							break;
					case 8: actionSub_sim.command = AC_OD_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC;
							break;
					case 9: actionSub_sim.command = AC_OD_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC;
							break;
				}
		#endif
				/* get format of video/image stream, only necessary at first frame (if successful, operation repeated otherwise) */
				if (firstFrame) {
					if(debugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: First frame detected."));
					cObjectPtr<IMediaType> mediaType;
					RETURN_IF_FAILED(inputDepthImagePin.GetMediaType(&mediaType));

					cObjectPtr<IMediaTypeVideo> typeDepthImage;
					RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&typeDepthImage));

					const tBitmapFormat* format = typeDepthImage->GetFormat();
					if (format == NULL) {
						LOG_ERROR("ObstacleDetec: No Bitmap information found on DepthImagePin.");
						RETURN_ERROR(ERR_NOT_SUPPORTED);
					}
					/* assign format of received input video to variable 'inputFormat' */
					inputFormat.nPixelFormat = format->nPixelFormat;
					inputFormat.nWidth = format->nWidth;
					inputFormat.nHeight = format->nHeight;
					inputFormat.nBitsPerPixel = format->nBitsPerPixel;
					inputFormat.nBytesPerLine = format->nBytesPerLine;
					inputFormat.nSize = format->nSize;
					inputFormat.nPaletteSize = format->nPaletteSize;

					/* check for format errors */
					if (inputFormat.nWidth <= 0 || inputFormat.nWidth > 400 || inputFormat.nHeight <= 0
							|| inputFormat.nHeight > 400) {
						RETURN_AND_LOG_ERROR_STR(ERR_UNEXPECTED, cString::Format("ObstacleDetec: Unexpected depth image size, since format with width: %d, height: %d is not supported!",inputFormat.nWidth, inputFormat.nHeight));
					}

					//* if receiving the input format information and processing of first frame was successful, flag is set to false */
					firstFrame = tFalse;

					/* Process first video frame if format was successfully set */
					RETURN_IF_FAILED(ProcessVideo(pMediaSample));
				}
				else {
					/* Process received video image, format has previously been set */
					RETURN_IF_FAILED(ProcessVideo(pMediaSample));

					#ifdef DEBUG_MODE_OUTPUT_VIDEO
					/* in case of 'no-modifiaction' no processAction is executed */
					if(debugType != 4){
						RETURN_IF_FAILED(ProcessAction(actionSub_sim));
					}
					#else
					if(GetRunningState()){
						/* Process current actionSub command from SCM */
						TActionStruct::ActionSub tmp_actionSub = GetCurrActionSub();
						RETURN_IF_FAILED(ProcessAction(tmp_actionSub));
					}
					#endif
				}

		}
		else if (pSource == &actionInput) {
			TActionStruct::ActionSub actionSub_tmp;
			actionSub_tmp = tActionStruct_object.Read_Action(pMediaSample, F_OBSTACLE_DETECTION);
			if(actionSub_tmp.enabled && actionSub_tmp.command == AC_OD_OPTICAL_ACC_ENABLE){
				SetOperationalStatus(OPT_ACC_ENABLED);
			}
			else if(actionSub_tmp.enabled && actionSub_tmp.command == AC_OD_OPTICAL_ACC_DISABLE){
				SetOperationalStatus(OPT_ACC_DISABLED);
			}
			else{
				tBool runningState_tmp = GetRunningState();
				if(actionSub_tmp.enabled && actionSub_tmp.started && !runningState_tmp) {
					SetCurrActionSub(actionSub_tmp);
					SetRunningState(tTrue);
				}
				/* if other actionSub has been send, stop processing of current actionSub.comand and deactivate */
				else if(!actionSub_tmp.enabled && runningState_tmp){
					SetRunningState(tFalse);
				}
			}
		}
		else if (pSource == &steeringAngleInput) {
			TSignalValue::Data steeringAngle_tmp;
			RETURN_IF_FAILED(tSignalValueSteeringInput_object.Read(pMediaSample, &steeringAngle_tmp));
			SetSteeringAngle(steeringAngle_tmp);
		}
		else if (pSource == &targetSpeedInput) {
			TSignalValue::Data targetSpeed_tmp;
			RETURN_IF_FAILED(tSignalValueSpeedInput_object.Read(pMediaSample, &targetSpeed_tmp));
			/* Check whether target speed is above recommended speed limit, and adjust accordingly */
			tFloat32 cur_targetSpeedLimit = GetTargetSpeedLimit();
			if(targetSpeed_tmp.f32_value > cur_targetSpeedLimit){
				targetSpeed_tmp.f32_value = cur_targetSpeedLimit;
			}
			RETURN_IF_FAILED(TransmitTargetSpeed(targetSpeed_tmp));
		}
	}

	RETURN_NOERROR;
}

/* process incoming action command; if enabled and started, process corresponding command */
tResult ObstacleDetection::ProcessAction(TActionStruct::ActionSub actionSub){
		if(actionSub.enabled && actionSub.started) {

			/* check command input */
			if(actionSub.command == AC_OD_INTERSECTION_CHECK_ALL){
				/* call method to check both oncoming traffic and traffic from right-hand-side */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_intersection_onComingTraffic,roi_intersection_crossTrafficRight,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No obstacle detected at all (in intersection mode), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'INTERSECTION - CHECK_ALL'. Return-status: %d",occupancy_status));
				}
				else if(occupancy_status == OCCUPIED_SPACE){
					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming or crossing obstacle detected (in intersection mode)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in intersection check-all mode)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_INTERSECTION_CHECK_ONCOMING_TRAFFIC){
				/* call method to check oncoming traffic only*/
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_intersection_onComingTraffic,REGIONTYPE_INTRSC_ONCOMING,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in intersection mode), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'INTERSECTION - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else  if(occupancy_status == OCCUPIED_SPACE){
					if(debugModeEnabled  && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT){
				/* call method to check traffic from right-hand-side only */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_intersection_crossTrafficRight,REGIONTYPE_INTRSC_CROSSRIGHT,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No crossing obstacle detected (in intersection mode), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'INTERSECTION - CROSSING RIGHT'. Return-status: %d",occupancy_status));
				}
				else  if(occupancy_status == OCCUPIED_SPACE){
					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Crossing obstacle detected!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_OVERTAKE_CHECK_ONCOMING_TRAFFIC){
				/* call method to check traffic on oncoming lane only */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_overtake_onComingTraffic,REGIONTYPE_OVRTK_ONCOMING,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in driving mode), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else  if(occupancy_status == OCCUPIED_SPACE){
					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected (in driving mode)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE){
				/* call method to check traffic on actual own original lane after overtake */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_overtake_originalLane,REGIONTYPE_OVRTK_ONCOMING,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else  if(occupancy_status == OCCUPIED_SPACE){
					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT){
				/* call method to check traffic on actual own lane after overtake */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_overtake_ownLaneStraight,REGIONTYPE_OVRTK_OWNLANE,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);

					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else if(occupancy_status == OCCUPIED_SPACE){

					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected!"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF){
				/* call method to check traffic on actual own lane after overtake */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_overtake_ownLaneStraight_lefthalf,REGIONTYPE_OVRTK_OWNLANE,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);

					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else if(occupancy_status == OCCUPIED_SPACE){

					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected!"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC){
				/* call method to check traffic on oncoming lane right before starting cross-parking maneuver */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_parking_cross_onComing,REGIONTYPE_CRS_PRKNG_ONCOMING,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);

					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No oncoming obstacle detected (in parking mode, crossparking), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'PARKING_CROSS - ONCOMING'. Return-status: %d",occupancy_status));
				}
				else if(occupancy_status == OCCUPIED_SPACE){

					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: Oncoming obstacle detected (in cross-parking mode, parking)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, parking)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else if(actionSub.command == AC_OD_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC){
				/* call method to check traffic on oncoming lane right before starting cross-parking maneuver */
				occupancy occupancy_status = INITIALILZED;
				occupancy_status = CheckROIforObstacles(roi_parking_cross_crossTraffic,REGIONTYPE_CRS_PRKNG_CROSSTRAFIC,m_cloud);
				if(occupancy_status == FREE_SPACE){
					RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
					SetRunningState(tFalse);

					if(debugModeEnabled && debugLogEventToggle != 1){
						LOG_WARNING(cString::Format("ObstacleDetec: No crosstraffic obstacle detected (in parking mode, crossparking), free space!"));
						debugLogEventToggle = 1;
					}
				}
				else if(occupancy_status == ERROR || occupancy_status == INITIALILZED){
					LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'PARKING_CROSS - CROSSTRAFFIC'. Return-status: %d",occupancy_status));
				}
				else if(occupancy_status == OCCUPIED_SPACE){

					if(debugModeEnabled && debugLogEventToggle != 2){
						LOG_WARNING(cString::Format("ObstacleDetec: CrossTraffic obstacle detected (in cross-parking mode, cross pullout)!"));
						debugLogEventToggle = 2;
					}
				}
				else if(occupancy_status == OCCUPIED_STATIC){
					RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
					SetRunningState(tFalse);
					if(debugModeEnabled && debugLogEventToggle != 3){
						LOG_WARNING(cString::Format("ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
						debugLogEventToggle = 3;
					}
				}
			}
			else{
				LOG_ERROR(cString::Format("ObstacleDetec: Error occurred. Process Action was called with non-existing actionSub command. Please check."));
			}
		}
		else{
			LOG_ERROR(cString::Format("ObstacleDetec: Error occurred. Process Action was called, but actionSub command was disabled. Please check."));
		}
	RETURN_NOERROR;
}

/*  check ROI passed to method for obstacles, just using counter in first version.
 *  -> functionality: check specified region of interest for objects, that means for points in point cloud that are in specified x-y-rectangle,
 *  and have a height greater than threshold-height; -> for each coordinate that fulfills this condition, increase region counter (one for each region);
 *  check counter-value at the end, and if limit is exceeded, then an obstacle is assumed to be there, so no feedback is sent (or static feedback)! (free-frame-counter is set to zero again);
 *  -> if the limit is NOT exceeded, that means there was supposedly no obstacle, then the "free-frame-counter" is increased by one;
 *  if free-frame-counter is greater than a certain number, the feedback "no obstacle" is sent to the SCM */
ObstacleDetection::occupancy ObstacleDetection::CheckROIforObstacles(const roi regionToCheck, regionType type, const std::vector<cv::Point3f> &cloud) {
	// reset counters
	OccupiedCoordinateCounter_onComing = 0;
	OccupiedCoordinateCounter_crossRight = 0;
	/* Iterate through all components of point cloud, check set properties */
	for(tUInt32 i = 0 ; i < cloud.size() ; i++){
		/* first take only the valid field of view for the camera to check rest */
		if(cloud[i].x > distance_threshold_min  && cloud[i].x < distance_threshold_max){
			if(cloud[i].z > height_threshold_min && cloud[i].z < height_threshold_max){
				/* check inside the propagated region of interest */
				if((cloud[i].x >= regionToCheck.bottomleft_corner.x) && (cloud[i].x <= (regionToCheck.bottomleft_corner.x + regionToCheck.roi_x_height))) {
					if((cloud[i].y <= regionToCheck.bottomleft_corner.y) && (cloud[i].y >= (regionToCheck.bottomleft_corner.y + regionToCheck.roi_y_width))) {
						if(type == REGIONTYPE_INTRSC_CROSSRIGHT) {
							OccupiedCoordinateCounter_crossRight++;
						}
						else if(type == REGIONTYPE_INTRSC_ONCOMING){
							OccupiedCoordinateCounter_onComing++;
						}
						else if(type == REGIONTYPE_OVRTK_ONCOMING){
							OccupiedCoordinateCounter_onComing++;
						}
						else if(type == REGIONTYPE_OVRTK_OWNLANE){
							OccupiedCoordinateCounter_onComing++;
						}
						else if(type == REGIONTYPE_CRS_PRKNG_ONCOMING){
							OccupiedCoordinateCounter_onComing++;
						}
						else if(type == REGIONTYPE_CRS_PRKNG_CROSSTRAFIC){
							OccupiedCoordinateCounter_crossRight++;
						}
					}
				}
			}
		}
	}

	if(extendeddebugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Occupied Counter oncoming: %d, cross right: %d", OccupiedCoordinateCounter_onComing, OccupiedCoordinateCounter_crossRight));

	/* recheck if correct type and if counter is below upper limit -> increase number of free frames in sequence */
	if((OccupiedCoordinateCounter_crossRight <= intrsc_OccupiedCoordinateCounterUpperBound_crossRight) && (type == REGIONTYPE_INTRSC_CROSSRIGHT)){
		freeFrameCounter++;
		occupiedFrameCounter = 0;
	}
	else if((OccupiedCoordinateCounter_crossRight <= crsPrkng_OccupiedCoordinateCounterUpperBound_crossRight) && (type == REGIONTYPE_CRS_PRKNG_CROSSTRAFIC)){
			freeFrameCounter++;
			occupiedFrameCounter = 0;
	}
	else if((OccupiedCoordinateCounter_onComing <= intrsc_OccupiedCoordinateCounterUpperBound_onComing) && (type == REGIONTYPE_INTRSC_ONCOMING)){
			freeFrameCounter++;
			occupiedFrameCounter = 0;
	}
	else if((OccupiedCoordinateCounter_onComing <= crsPrkng_OccupiedCoordinateCounterUpperBound_onComing) && (type == REGIONTYPE_CRS_PRKNG_ONCOMING)){
			freeFrameCounter++;
			occupiedFrameCounter = 0;
	}
	else if((OccupiedCoordinateCounter_onComing <= ovrtk_OccupiedCoordinateCounterUpperBound_onComing) && (type == REGIONTYPE_OVRTK_ONCOMING)){
			freeFrameCounter++;
			occupiedFrameCounter = 0;
	}
	else if((OccupiedCoordinateCounter_onComing <= ovrtk_OccupiedCoordinateCounterUpperBound_ownLaneStraight) && (type == REGIONTYPE_OVRTK_OWNLANE)){
			freeFrameCounter++;
			occupiedFrameCounter = 0;
	}
	else{
		/* if frame is not free, reset free frame counter to zero, start new counting-sequence */
		freeFrameCounter = 0;
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		/* in this case, the frame is occupied, so start counting how many frames in a row to see if it is a static obstacle */
		occupiedFrameCounter++;
		if(occupiedFrameCounter >= overtake_ownlane_occupiedFrameCounterLowerBound && type == REGIONTYPE_OVRTK_OWNLANE){
			occupiedFrameCounter = 0;
			return OCCUPIED_STATIC;
		}
		else if(occupiedFrameCounter >= overtake_oncoming_occupiedFrameCounterLowerBound && (type == REGIONTYPE_OVRTK_ONCOMING)){
			occupiedFrameCounter = 0;
			return OCCUPIED_STATIC;
		}
		else if(occupiedFrameCounter >= intrsc_occupiedFrameCounterLowerBound && (type == REGIONTYPE_INTRSC_CROSSRIGHT || type == REGIONTYPE_INTRSC_ONCOMING)){
			occupiedFrameCounter = 0;
			return OCCUPIED_STATIC;
		}
		else if(occupiedFrameCounter >= crsPrkng_occupiedFrameCounterLowerBound && (type == REGIONTYPE_CRS_PRKNG_CROSSTRAFIC || type == REGIONTYPE_CRS_PRKNG_ONCOMING)){
			occupiedFrameCounter = 0;
			return OCCUPIED_STATIC;
		}

		return OCCUPIED_SPACE;
	}

	/* check if lower bound of frames without obstacle is reached */
	if((freeFrameCounter >= intrsc_freeFrameCounterLowerBound) && (type == REGIONTYPE_INTRSC_CROSSRIGHT || type == REGIONTYPE_INTRSC_ONCOMING)){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else if((freeFrameCounter >= ovrtk_freeFrameCounterLowerBound_oncoming) && (type == REGIONTYPE_OVRTK_ONCOMING)){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else if((freeFrameCounter >= ovrtk_freeFrameCounterLowerBound_ownLaneStraight) && (type == REGIONTYPE_OVRTK_OWNLANE)){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else if((freeFrameCounter >= crsPrkng_freeFrameCounterLowerBound_crossRight) && (type == REGIONTYPE_CRS_PRKNG_CROSSTRAFIC)){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else if((freeFrameCounter >= crsPrkng_freeFrameCounterLowerBound_oncoming) && (type == REGIONTYPE_CRS_PRKNG_ONCOMING)){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else{ // otherwise
		return STATE_PENDING;
	}
}

ObstacleDetection::occupancy ObstacleDetection::CheckROIforObstacles(roi regionToCheck_OnComing, roi regionToCheck_CrossRight, const std::vector<cv::Point3f> &cloud) {
	// reset counters
	OccupiedCoordinateCounter_onComing = 0;
	OccupiedCoordinateCounter_crossRight = 0;
	/* Iterate through all components of point cloud, check set properties */
	for(tUInt32 i = 0 ; i < m_cloud.size() ; i++){
		/* first take only the valid field of view for the camera to check rest */
		if(cloud[i].x > distance_threshold_min  && cloud[i].x < distance_threshold_max){
			if(cloud[i].z > height_threshold_min && cloud[i].z < height_threshold_max){


				/* check inside the propagated region for oncoming traffic */
				if((cloud[i].x >= regionToCheck_OnComing.bottomleft_corner.x) && (cloud[i].x <= (regionToCheck_OnComing.bottomleft_corner.x + regionToCheck_OnComing.roi_x_height))) {
					if((cloud[i].y <= regionToCheck_OnComing.bottomleft_corner.y) && (cloud[i].y >= (regionToCheck_OnComing.bottomleft_corner.y + regionToCheck_OnComing.roi_y_width))) {

							OccupiedCoordinateCounter_onComing++;
					}
				}
				/* check inside the propagated region for cross-traffic from right */
				if((cloud[i].x >= regionToCheck_CrossRight.bottomleft_corner.x) && (cloud[i].x <= (regionToCheck_CrossRight.bottomleft_corner.x + regionToCheck_CrossRight.roi_x_height))) {
					if((cloud[i].y <= regionToCheck_CrossRight.bottomleft_corner.y) && (cloud[i].y >= (regionToCheck_CrossRight.bottomleft_corner.y + regionToCheck_CrossRight.roi_y_width))) {

							OccupiedCoordinateCounter_crossRight++;
					}
				}
			}
		}
	}

	
	if(extendeddebugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Occupied Counter oncoming: %d, cross right: %d", OccupiedCoordinateCounter_onComing, OccupiedCoordinateCounter_crossRight));

	/* check if counters are !both! below upper limit -> increase number of free frames in sequence */
	if((OccupiedCoordinateCounter_crossRight <= intrsc_OccupiedCoordinateCounterUpperBound_crossRight)
			&& (OccupiedCoordinateCounter_onComing <= intrsc_OccupiedCoordinateCounterUpperBound_onComing)) {
		freeFrameCounter++;
		occupiedFrameCounter = 0;
	}
	else{
		/* if frame is not free, reset counter to zero, start new counting-sequence */
		freeFrameCounter = 0;
		occupiedFrameCounter++;
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		if(occupiedFrameCounter >= intrsc_occupiedFrameCounterLowerBound){
			occupiedFrameCounter = 0;
			return OCCUPIED_STATIC;
		}
		return OCCUPIED_SPACE;
	}

	/* check if lower bound of frames without obstacle is reached */
	if(freeFrameCounter >= intrsc_freeFrameCounterLowerBound){
		/* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
		OccupiedCoordinateCounter_crossRight = 0;
		OccupiedCoordinateCounter_onComing = 0;
		freeFrameCounter = 0;
		return FREE_SPACE;
	}
	else{ // otherwise
		return STATE_PENDING;
	}
}

/* Methods checks for Obstacles in current field of view, returns a reduced speed depending on position of recognized obstacles */
/* check the point-cloud for objects in the current driving lane */
// params: 	offset from front-of-car to wheels;
//			length of the circular segment, that means distance in front of car to check;
// 			oblique angle front;
// 			zones for points: FAR, INTERMEDIATE, NEAR in percent of line-segment?;
//			width of check-area in y-direction:  |<-- x -->|  ;
// inputs: steering angle, transformed point-cloud
// output: speed
ObstacleDetection::ProcessedCloudData ObstacleDetection::ProcessOpticalAccAndGenerateValidationImage(const TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &cloud){
	cv::Mat valid_pixel_depth_image(inputFormat.nHeight, inputFormat.nWidth, CV_8UC1);
	tBool pixel_valid = tTrue;
	tBool valid_corridor_size = tTrue;
	ProcessedCloudData returnStruct;
	tUInt32 debug_counter = 0;

	/* Gridcell element and 2dimensional vector for Occupancy Grid */
	GridcellElement gridCell;
	std::vector<std::vector<GridcellElement> > occupancyGrid(inputFormat.nHeight/OACC_GridCellResolution, std::vector<GridcellElement>(inputFormat.nWidth/OACC_GridCellResolution));
//	LOG_WARNING(cString::Format("ObstacleDetection: OccupancyGrid - size of Gridmap: width %d, height %d", occupancyGrid[0].size(), occupancyGrid.size()));
	tFloat32 corr_steeringAngleAbs = ((fabsf(steeringAngle.f32_value - 90.0f) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
	//	LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute : %f",corr_steeringAngleAbs));

	/* handle case that steringAngle would be smaller than zero due to obliqueAngle */
	if(corr_steeringAngleAbs < 0.0f){
		corr_steeringAngleAbs = 0.0f;
		LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute set to zero: %f",corr_steeringAngleAbs));
	}
	/* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
	if(corr_steeringAngleAbs < 0.005f){
		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path with steering < 0.05 taken!"));
		/* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
		tFloat32 x_oacc_rect_min = 0.0f;
		tFloat32 x_oacc_rect_max = OACC_circSegmentDistance - OACC_frontAxisToFrontBumper;
		//LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor -> xmin: %f, xmax: %f",x_oacc_rect_min,x_oacc_rect_max));
		if(x_oacc_rect_max <= 0.0f){
			LOG_WARNING(cString::Format("ObstacleDetec: OACC - search-corridor in x-direction too small, speed limit will be set to zero! Check search corridor properties."));
			valid_corridor_size = tFalse;
		}
		tFloat32 y_oacc_rect_min =  -OACC_ySearchTolerance;
		tFloat32 y_oacc_rect_max =   OACC_ySearchTolerance;
		//LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor -> ymin: %f, ymax: %f",y_oacc_rect_min,y_oacc_rect_max));

		/* Iterate through all components of point cloud, check for matching data-points */
		for(tUInt32 i = 0 ; i < cloud.size() ; i++){
			/* BEGIN: VALIDATION OUTPUT IMAGE */
			/* perform necessary splitting into parameters 'u' and  'v' needed for validation-image-creation */
			tUInt32 u = i % inputFormat.nWidth; 		// columns
			tUInt32 v = static_cast<tUInt32>(i / inputFormat.nWidth); 		// rows
			pixel_valid = tTrue;
			/* END: VALIDATION OUTPUT IMAGE */
			/* Occupancy Grid resolution scaling */
			tUInt32 a = static_cast<tUInt32>(u / OACC_GridCellResolution);  	// OCG columns
			tUInt32 b = static_cast<tUInt32>(v / OACC_GridCellResolution);	// OCG rows
//			LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid -rect- counter u: %d, v: %d, a: %d, b: %d",u,v,a,b));
			/* Check for proper height of obstacle, discard otherwise */
			if(cloud[i].z > OACC_obstacleHeightThreshold_min && cloud[i].z < OACC_obstacleHeightThreshold_max){
				//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed z-condition."));
				/* take only values in the valid rectangular search field */
				if(cloud[i].x >= x_oacc_rect_min   && cloud[i].x <= x_oacc_rect_max ){
					//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed x-condition."));
					if(cloud[i].y >= y_oacc_rect_min && cloud[i].y <= y_oacc_rect_max){
						//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed y-condition."));
							occupancyGrid[b][a].CoordinateSum.x += cloud[i].x;
							occupancyGrid[b][a].CoordinateSum.y += cloud[i].y;
							occupancyGrid[b][a].elem_counter++;
							debug_counter++;

					}
				}
			}
			/* BEGIN: VALIDATION OUTPUT IMAGE */
			/* check if height in the currently regarded pixel is too far above the ground */
			if(cloud[i].z > height_threshold_validmask){
				/* reduce invalid measurement points (received with x-distance smaller/equal to zero) */
				if(cloud[i].x > 0){
					pixel_valid = tFalse;
				}
			}
			if(pixel_valid){
				valid_pixel_depth_image.at<tUInt8>(v, u) = OD_VI_CV8UC1_MAX_RANGE; // white
			} else {
				valid_pixel_depth_image.at<tUInt8>(v, u) = 0; // black
			}
			/* END: VALIDATION OUTPUT IMAGE */
		}
		if(debug_counter > 0 && debugACCObstaclepixelcount){
			LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixels in rect-area, counter: %d!",debug_counter));
		}
	}
	else{
		/* radius between momentum-pole and middle of front-axis */
		tFloat32 segmentRadius = OACC_wheelbase / sinf(corr_steeringAngleAbs);
		tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
		/* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
		if((segmentAngle + corr_steeringAngleAbs) > CV_PI/2){
			segmentAngle = CV_PI/2 - corr_steeringAngleAbs;
		}

		/* minimum and maximum values of the reference line-segment */
		tFloat32 x_oacc_ref_min = 0.0f;
		tFloat32 x_oacc_ref_max = segmentRadius * sinf(corr_steeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;
		tFloat32 y_oacc_ref_min = 0.0f;
		tFloat32 y_oacc_ref_max = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - (segmentRadius * cosf(corr_steeringAngleAbs + segmentAngle));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor_REF -> xmin: %f, xmax: %f",x_oacc_ref_min,x_oacc_ref_max));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REF -> ymin: %f, ymax: %f",y_oacc_ref_min,y_oacc_ref_max));

		/* global minimum and maximum values including the y-search-tolerance for shrinking the general rectangular search corridor */
		tFloat32 x_oacc_ref_tol_min = x_oacc_ref_min;
		tFloat32 x_oacc_ref_tol_max = x_oacc_ref_max;
		tFloat32 y_oacc_ref_tol_min = 0.0f;
		tFloat32 y_oacc_ref_tol_max = 0.0f;
		/* In case that steering to left hand side, that means in positive y-direction */
		if((steeringAngle.f32_value - 90) < 0){
			y_oacc_ref_tol_min = y_oacc_ref_min - OACC_ySearchTolerance;
			y_oacc_ref_tol_max = y_oacc_ref_max + OACC_ySearchTolerance;
		}
		else{ //steering to right hand side
			y_oacc_ref_tol_min = -y_oacc_ref_max - OACC_ySearchTolerance;
			y_oacc_ref_tol_max = -y_oacc_ref_min + OACC_ySearchTolerance;
		}
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REAL -> ymin: %f, ymax: %f, curr.steeringAngle: %f",y_oacc_ref_tol_min,y_oacc_ref_tol_max,steeringAngle.f32_value));

		/* Relation between currently viewed/checked x_oacc and y_oacc */
		tFloat32 y_curOacc_ref = 0.0f;
		/* Iterate through all components of point cloud, check for matching data-points */
		for(tUInt32 i = 0 ; i < cloud.size() ; i++){
			/* BEGIN: VALIDATION OUTPUT IMAGE */
			/* perform necessary splitting into parameters 'u' and  'v' needed for validation-image-creation */
			tUInt32 u = i % inputFormat.nWidth; // columns
			tUInt32 v = i / inputFormat.nWidth; // rows
			pixel_valid = tTrue;
			/* END: VALIDATION OUTPUT IMAGE */
			/* Occupancy Grid resolution scaling */
			tUInt32 a = static_cast<tUInt32>(u / OACC_GridCellResolution);  	// OCG columns
			tUInt32 b = static_cast<tUInt32>(v / OACC_GridCellResolution);	// OCG rows
//			LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid -dyn- counter u: %d, v: %d, a: %d, b: %d",u,v,a,b));
			/* Check for proper height of obstacle, discard otherwise */
			if(cloud[i].z > OACC_obstacleHeightThreshold_min && cloud[i].z < OACC_obstacleHeightThreshold_max){
				/* take only values in the valid rectangular search field */
				if(cloud[i].x >= x_oacc_ref_tol_min   && cloud[i].x <= x_oacc_ref_tol_max ){
					if(cloud[i].y >= y_oacc_ref_tol_min && cloud[i].y <= y_oacc_ref_tol_max){
						/* check only values that fit the specific search corridor, given by the circular relation between x and y ; K-Y = y~ */
						/* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value - 90) < 0 */
						y_curOacc_ref = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((cloud[i].x + OACC_wheelbase)*(cloud[i].x + OACC_wheelbase)));
						/* case of steering right: */
						if((steeringAngle.f32_value - 90) > 0){
							y_curOacc_ref = - y_curOacc_ref;
						}
						/* actual check is performed */
						if((cloud[i].y >= (y_curOacc_ref - OACC_ySearchTolerance)) && (cloud[i].y <= y_curOacc_ref + OACC_ySearchTolerance)){
							/* At this point, currently checked data-point really is in the current sector of interest */
							occupancyGrid[b][a].CoordinateSum.x += cloud[i].x;
							occupancyGrid[b][a].CoordinateSum.y += cloud[i].y;
							occupancyGrid[b][a].elem_counter++;
							debug_counter ++;
						}
					}
				}
			}
			/* BEGIN: VALIDATION OUTPUT IMAGE */
			/* check if height in the currently regarded pixel is too far above the ground */
			if(cloud[i].z > height_threshold_validmask){
				/* reduce invalid measurement points (received with x-distance smaller/equal to zero) */
				if(cloud[i].x > 0){
					pixel_valid = tFalse;
				}
			}
			if(pixel_valid){
				valid_pixel_depth_image.at<tUInt8>(v, u) = OD_VI_CV8UC1_MAX_RANGE; // white
			} else {
				valid_pixel_depth_image.at<tUInt8>(v, u) = 0; // black
			}
			/* END: VALIDATION OUTPUT IMAGE */
		}

		if(debug_counter > 0 && debugACCObstaclepixelcount){
			LOG_WARNING(cString::Format("ObstacleDetection: OACC - found Obstacle-pixels in dynamic area, counter: %d",debug_counter));
		}
	}

	/* Parse created occupancy grid for occupied cell elements */
	tFloat32 min_distanceToGridCell = 10.0f; // set to high value
	tFloat32 tmp_distanceToGridCell = 0.0f;
	cv::Point2f cellBalancePoint;
	cv::Point2i debug_log_closestCellCoords;
	for(tUInt32 row = 0; row < inputFormat.nHeight/OACC_GridCellResolution; row++){
		for(tUInt32 col = 0; col < inputFormat.nWidth/OACC_GridCellResolution; col++){
			if(occupancyGrid[row][col].elem_counter >= OACC_GridCellOccupiedCounterThreshold){
				cellBalancePoint.x = occupancyGrid[row][col].CoordinateSum.x / static_cast<tFloat32>(occupancyGrid[row][col].elem_counter);
				cellBalancePoint.y = occupancyGrid[row][col].CoordinateSum.y / static_cast<tFloat32>(occupancyGrid[row][col].elem_counter);
				tmp_distanceToGridCell = cv::norm(cellBalancePoint);
				if(tmp_distanceToGridCell < min_distanceToGridCell){
					min_distanceToGridCell = tmp_distanceToGridCell;
					debug_log_closestCellCoords.x = row;
					debug_log_closestCellCoords.y = col;
				}
			}
		}
	}
	if(debug_occupancyGrid){
		LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid - minimum distance to occupied cell: %f; cell row: %d, col: %d", min_distanceToGridCell,debug_log_closestCellCoords.x,debug_log_closestCellCoords.y));
	}

	/* If search corridor is valid, use minimum distance to read the speedlimit from xml-file ! */
	if(valid_corridor_size){
		tFloat32 tmp_speedlimit =  GetLinearInterpolatedValue(min_distanceToGridCell,oAcc_xml_xValues,oAcc_xml_yValues);
		if(debug_occupancyGrid){
			LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid - calculated speed limit: %f for distance : %f", tmp_speedlimit, min_distanceToGridCell));
		}
		if(tmp_speedlimit < 0.0f){
			LOG_ERROR(cString::Format("ObstacleDetec: Invalid speed limit, error occurred in calculating interpolated value. Limit set to zero!"));
			tmp_speedlimit = 0.0f;
		}
		returnStruct.targetSpeedLimit = tmp_speedlimit;
	}
	else{
		returnStruct.targetSpeedLimit = 0.0f;
	}

	returnStruct.cv_validPixelDepthImage = valid_pixel_depth_image;
	return returnStruct;
}



cv::Mat ObstacleDetection::generateDebugOutputOpticACC(TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &cloud)
{
	//LOG_WARNING(cString::Format("ObstacleDetec: generateDebugOutputOpticACC called with steering angle: %f.",steeringAngle.f32_value));
	cv::Mat checked_pixel_depth_image(inputFormat.nHeight, inputFormat.nWidth, CV_16UC1);
	tBool pixel_no_obstacle = tTrue;
	tUInt64 counter_tmp = 0;

	tFloat32 corr_steeringAngleAbs = ((fabsf(steeringAngle.f32_value - 90.0f) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
//	LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute : %f",corr_steeringAngleAbs));

	/* handle case that steringAngle would be smaller than zero due to obliqueAngle */
	if(corr_steeringAngleAbs < 0.0f){
		corr_steeringAngleAbs = 0.0f;
		LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute set to zero: %f",corr_steeringAngleAbs));
	}
	/* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
	if(corr_steeringAngleAbs < 0.005f){
		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path with steering < 0.05 taken!"));
		/* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
		tFloat32 x_oacc_rect_min = 0.0f;
		tFloat32 x_oacc_rect_max = OACC_circSegmentDistance - OACC_frontAxisToFrontBumper;
		//LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor -> xmin: %f, xmax: %f",x_oacc_rect_min,x_oacc_rect_max));
		if(x_oacc_rect_max < 0.0f){
			LOG_WARNING(cString::Format("ObstacleDetec: OACC - search-corridor in x-direction too small, received input target speed will be returned unmodified!"));
		}
		tFloat32 y_oacc_rect_min =  -OACC_ySearchTolerance;
		tFloat32 y_oacc_rect_max =   OACC_ySearchTolerance;
		//LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor -> ymin: %f, ymax: %f",y_oacc_rect_min,y_oacc_rect_max));
		/* Iterate through all components of point cloud, check for matching data-points */
		for(tUInt32 i = 0 ; i < cloud.size() ; i++) {
			//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path1 - checking pointcloud!"));
			const tUInt32 u = i % inputFormat.nWidth; // columns
			const tUInt32 v = i / inputFormat.nWidth; // rows
			pixel_no_obstacle = tTrue;
			/* Check for proper height of obstacle, discard otherwise */
			if(cloud[i].z > OACC_obstacleHeightThreshold_min && cloud[i].z < OACC_obstacleHeightThreshold_max){
				//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed z-condition."));
				/* take only values in the valid rectangular search field */
				if(cloud[i].x >= x_oacc_rect_min   && cloud[i].x <= x_oacc_rect_max ){
					//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed x-condition."));
					if(cloud[i].y >= y_oacc_rect_min && cloud[i].y <= y_oacc_rect_max){
						//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed y-condition."));

						pixel_no_obstacle = tFalse;
						counter_tmp++;
						//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel, counter: %d!",counter_tmp));
					}
				}
			}

			if(pixel_no_obstacle){
				checked_pixel_depth_image.at<tUInt16>(v, u) = OD_VI_CV16UC1_MAX_RANGE; // white
			} else {
				checked_pixel_depth_image.at<tUInt16>(v, u) = 0; // black
			}
		}
		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel, counter: %d!",counter_tmp));
	}
	else{
		/* radius between momentum-pole and middle of front-axis */
		tFloat32 segmentRadius = OACC_wheelbase / sinf(corr_steeringAngleAbs);
		tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
		/* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
		if((segmentAngle + corr_steeringAngleAbs) > CV_PI/2){
			segmentAngle = CV_PI/2 - corr_steeringAngleAbs;
		}

		/* minimum and maximum values of the reference line-segment */
		tFloat32 x_oacc_ref_min = 0.0f;
		tFloat32 x_oacc_ref_max = segmentRadius * sinf(corr_steeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;
		tFloat32 y_oacc_ref_min = 0.0f;
		tFloat32 y_oacc_ref_max = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - (segmentRadius * cosf(corr_steeringAngleAbs + segmentAngle));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor_REF -> xmin: %f, xmax: %f",x_oacc_ref_min,x_oacc_ref_max));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REF -> ymin: %f, ymax: %f",y_oacc_ref_min,y_oacc_ref_max));

		/* global minimum and maximum values including the y-search-tolerance for shrinking the general rectangular search corridor */
		tFloat32 x_oacc_ref_tol_min = x_oacc_ref_min;
		tFloat32 x_oacc_ref_tol_max = x_oacc_ref_max;
		tFloat32 y_oacc_ref_tol_min = 0.0f;
		tFloat32 y_oacc_ref_tol_max = 0.0f;
		/* In case that steering to left hand side, that means in positive y-direction */
		if((steeringAngle.f32_value - 90) < 0){
			y_oacc_ref_tol_min = y_oacc_ref_min - OACC_ySearchTolerance;
			y_oacc_ref_tol_max = y_oacc_ref_max + OACC_ySearchTolerance;
		}
		else{ //steering to right hand side
			y_oacc_ref_tol_min = -y_oacc_ref_max - OACC_ySearchTolerance;
			y_oacc_ref_tol_max = -y_oacc_ref_min + OACC_ySearchTolerance;
		}
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REAL -> ymin: %f, ymax: %f, curr.steeringAngle: %f",y_oacc_ref_tol_min,y_oacc_ref_tol_max,steeringAngle.f32_value));
		/* Relation between currently viewed/checked x_oacc and y_oacc */
		tFloat32 y_curOacc_ref = 0.0f;
		/* Iterate through all components of point cloud, check for matching data-points */
		for(tUInt32 i = 0 ; i < cloud.size() ; i++) {
			const tUInt32 u = i % inputFormat.nWidth; // columns
			const tUInt32 v = i / inputFormat.nWidth; // rows
			pixel_no_obstacle = tTrue;
			/* Check for proper height of obstacle, discard otherwise */
			if(cloud[i].z > OACC_obstacleHeightThreshold_min && cloud[i].z < OACC_obstacleHeightThreshold_max){
				/* take only values in the valid rectangular search field */
				if(cloud[i].x >= x_oacc_ref_tol_min   && cloud[i].x <= x_oacc_ref_tol_max ){
					if(cloud[i].y >= y_oacc_ref_tol_min && cloud[i].y <= y_oacc_ref_tol_max){
						/* check only values that fit the specific search corridor, given by the circular relation between x and y ; K-Y = y~ */
						/* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value - 90) < 0 */
						y_curOacc_ref = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((cloud[i].x + OACC_wheelbase)*(cloud[i].x + OACC_wheelbase)));
						/* case of steering right: */
						if((steeringAngle.f32_value - 90) > 0){
							y_curOacc_ref = - y_curOacc_ref;
						}
						/* actual check is performed */
						if((cloud[i].y >= (y_curOacc_ref - OACC_ySearchTolerance)) && (cloud[i].y <= y_curOacc_ref + OACC_ySearchTolerance)){
							/* At this point, currently checked data-point really is in the current sector of interest */
							pixel_no_obstacle = tFalse;
							counter_tmp ++;
						}
					}
				}
			}
			if(pixel_no_obstacle){
				checked_pixel_depth_image.at<tUInt16>(v, u) = OD_VI_CV16UC1_MAX_RANGE; // white
			} else {
				checked_pixel_depth_image.at<tUInt16>(v, u) = 0; // black
			}
		}
		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel dynamic, counter: %d!",counter_tmp));
	}


	return checked_pixel_depth_image;
}

/* Deprecated Method, not in use anymore*/
//cv::Mat ObstacleDetection::generateValidPixelDepthFromTransformedCloud(const std::vector<cv::Point3f> &cloud){
//	cv::Mat valid_pixel_depth_image(inputFormat.nHeight, inputFormat.nWidth, CV_8UC1);
//	tBool pixel_valid = tTrue;
//
//	for(tUInt32 i = 0 ; i < cloud.size() ; i++) {
//		const tUInt32 u = i % inputFormat.nWidth; // columns
//		const tUInt32 v = i / inputFormat.nWidth; // rows
//
//		pixel_valid = tTrue;
//
//		/* check if height in the currently regarded pixel is too far above the ground */
//		if(cloud[i].z > height_threshold_validmask){
//			/* reduce invalid measurement points (received with x-distance smaller/equal to zero) */
//			if(cloud[i].x > 0){
//				pixel_valid = tFalse;
//			}
//		}
//
//		if(pixel_valid){
//			valid_pixel_depth_image.at<tUInt8>(v, u) = OD_VI_CV8UC1_MAX_RANGE; // white
//		} else {
//			valid_pixel_depth_image.at<tUInt8>(v, u) = 0; // black
//		}
//
//	}
//
//	return valid_pixel_depth_image;
//}

/* Transmit feedback struct with status no_obstacle to SCM */
tResult ObstacleDetection::TransmitFeedbackNoObstacle() {
	__synchronized_obj(criticalSection_TransmitFeedback);
	TFeedbackStruct::Data feedback;
	feedback.ui32_filterID = F_OBSTACLE_DETECTION;
	feedback.ui32_status = FB_OD_NO_OBSTACLE;
	RETURN_IF_FAILED(tFeedbackStruct_object.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime()));
	RETURN_NOERROR;
}

/* Transmit feedback struct with status 'Obstacle' to SCM */
tResult ObstacleDetection::TransmitFeedbackStaticObstacle() {
	__synchronized_obj(criticalSection_TransmitFeedback);
	TFeedbackStruct::Data feedback;
	feedback.ui32_filterID = F_OBSTACLE_DETECTION;
	feedback.ui32_status = FB_OD_STATIC_OBSTACLE;
	RETURN_IF_FAILED(tFeedbackStruct_object.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime()));
	RETURN_NOERROR;
}

tResult ObstacleDetection::TransmitTargetSpeed(TSignalValue::Data newTargetSpeed){
	__synchronized_obj(criticalSection_TransmitSpeed);
	RETURN_IF_FAILED(tSignalValueOutput_object.Transmit(&targetSpeedOutput, newTargetSpeed, _clock->GetStreamTime()));
	RETURN_NOERROR;
}


/* Performs/Calls all necessary steps to read depth image from input and transform to output point cloud */
tResult ObstacleDetection::ProcessVideo(IMediaSample* sample) {
	/* Get current values associated with this video frame */
	TSignalValue::Data steeringAngle_tmp = GetSteeringAngle();
	/* Create new Mat matrix */
	Mat depth_image(inputFormat.nHeight, inputFormat.nWidth, CV_16UC1);
	RETURN_IF_FAILED(GetInputDepthImage(depth_image, sample));
	/* Original data is now referenced by 'depth_image' of type Mat, but is still pointing
	 *  on the original input data! */

	/* generate Point Cloud from Depth Image */
	RETURN_IF_FAILED(generatePointCloudFromDepthImage(depth_image));

	/* transform Point Cloud : translation and rotation */
	RETURN_IF_FAILED(transformPointCloud(m_cloud));

	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	/* Create output header for showing debug media stream on video output */
	Mat out(inputFormat.nHeight, inputFormat.nWidth, CV_16UC1);
	out = generateDebugDepthFromTransformedCloud(m_cloud);
	//generate output
	cObjectPtr<IMediaSample> depthSample;
	if (IS_OK(AllocMediaSample(&depthSample))) {
		tBitmapFormat outputFormat = inputFormat;
		outputFormat.nWidth = out.cols;
		outputFormat.nHeight = out.rows;
		outputFormat.nBitsPerPixel = out.channels() * 16;
		outputFormat.nBytesPerLine = out.cols * 2 * out.channels(); // * 2: 16 bit
		outputFormat.nSize = outputFormat.nBytesPerLine * out.rows;

		if (out.type() == CV_16UC1) {
			outputFormat.nPixelFormat = cImage::PF_GREYSCALE_16;
		}

		depthSample->Update(sample->GetTime(), out.data, tInt32(outputFormat.nSize), 0);

		outputVideoPin.SetFormat(&outputFormat, NULL);
		outputVideoPin.Transmit(depthSample);
	}
	#endif

	/* Check for obstacles in current driving lane and additionally generate the validation image for line-checking */
	ProcessedCloudData processedCloudData = ProcessOpticalAccAndGenerateValidationImage(steeringAngle_tmp, m_cloud);
	/* only change input speed */
	if(GetOperationalStatus() == OPT_ACC_ENABLED){
		SetTargetSpeedLimit(processedCloudData.targetSpeedLimit);
	}
	else if(GetOperationalStatus() == OPT_ACC_DISABLED){
		SetTargetSpeedLimit(10.0f);
	}
	else{
		LOG_ERROR(cString::Format("ObstacleDetec: Error occurred, could not read operational status! Please check."));
	}

	/* Create GCL output to show which region is checked (to be overlaid the RGB image in video display)*/
	if (m_oGCLOutput.IsConnected() && showGCLDebug) {
		RETURN_IF_FAILED(CreateAndTransmitGCL_dynamicDrive(steeringAngle_tmp));
	}

	/* Create output header for debug opticalACC video output if output is connected and property is set */
	if (outputDebugOptACCPin.IsConnected() && showOaccBinaryImageDebug) {
		Mat out_OptACC_Debug = generateDebugOutputOpticACC(steeringAngle_tmp,m_cloud);

		//generate output
		cObjectPtr<IMediaSample> optACCSample;
		if (IS_OK(AllocMediaSample(&optACCSample))) {
			tBitmapFormat outputFormat_optACC = inputFormat;
			outputFormat_optACC.nWidth = out_OptACC_Debug.cols;
			outputFormat_optACC.nHeight = out_OptACC_Debug.rows;
			outputFormat_optACC.nBitsPerPixel = out_OptACC_Debug.channels() * 16;
			outputFormat_optACC.nBytesPerLine = out_OptACC_Debug.cols * 2 * out_OptACC_Debug.channels(); // * 2: 16 bit
			outputFormat_optACC.nSize = outputFormat_optACC.nBytesPerLine * out_OptACC_Debug.rows;

			if (out_OptACC_Debug.type() == CV_16UC1) {
				outputFormat_optACC.nPixelFormat = cImage::PF_GREYSCALE_16;
			}

			optACCSample->Update(sample->GetTime(), out_OptACC_Debug.data, tInt32(outputFormat_optACC.nSize), 0);

			outputDebugOptACCPin.SetFormat(&outputFormat_optACC, NULL);
			outputDebugOptACCPin.Transmit(optACCSample);
		}
	}

	/* Create output header for validation video output */
	//Mat out_valid_image = generateValidPixelDepthFromTransformedCloud(m_cloud);
	Mat out_valid_image = processedCloudData.cv_validPixelDepthImage;
	//generate output
	cObjectPtr<IMediaSample> depthValidSample;
	if (IS_OK(AllocMediaSample(&depthValidSample))) {
		tBitmapFormat outputFormat_valid = inputFormat;
		outputFormat_valid.nWidth = out_valid_image.cols;
		outputFormat_valid.nHeight = out_valid_image.rows;
		outputFormat_valid.nBitsPerPixel = out_valid_image.channels() * 8;
		outputFormat_valid.nBytesPerLine = out_valid_image.cols *1 * out_valid_image.channels(); // * 1: 8 bit
		outputFormat_valid.nSize = outputFormat_valid.nBytesPerLine * out_valid_image.rows;

		if (out_valid_image.type() == CV_8UC1) {
			outputFormat_valid.nPixelFormat = cImage::PF_GREYSCALE_8;
		}

		depthValidSample->Update(sample->GetTime(), out_valid_image.data, tInt32(outputFormat_valid.nSize), 0);

		outputVideoValidPin.SetFormat(&outputFormat_valid, NULL);
		outputVideoValidPin.Transmit(depthValidSample);
	}

	RETURN_NOERROR;
}


tResult ObstacleDetection::GetInputDepthImage(cv::Mat &image, IMediaSample* sample) {
	if (inputFormat.nPixelFormat != cImage::PF_GREYSCALE_16) {
		RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED, "ObstDetec: Only 16 bit greyscale images are supported.");
	}
	/* Create buffer-pointer to set on data of input stream and create temporary header for referencing to data */
	const tVoid* srcBuffer;
	IplImage* img = cvCreateImageHeader(cvSize(inputFormat.nWidth, inputFormat.nHeight), IPL_DEPTH_16U, 1);
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



tResult ObstacleDetection::generatePointCloudFromDepthImage(cv::Mat& depthImage)
{
	/* Check if depthImage has valid dataPointer data*/
	if (depthImage.data == NULL) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_ADDRESS,cString::Format("ObstacleDetec: Bad data in depthImage."));
	}

	/* clear existing pointcloud-list and reserve space for new list;
	 *  DO NOT use resize (with resize, list is filled with dummy-values) */
	m_cloud.clear();
	m_cloud.reserve(depthImage.rows * depthImage.cols);

	//tUInt16* ptr = (tUInt16*) (depthImage.data);
	for (tUInt32 y = 0; y < static_cast<tUInt32>(depthImage.rows); y++) {
		for (tUInt32 x = 0; x < static_cast<tUInt32>(depthImage.cols); x++)
		{
			cv::Point3f p;
			tFloat32 z = ((tFloat32)depthImage.at<tUInt16>(y, x)) * 0.001; // Convert from mm to meters
			//tFloat32 z = ((tFloat32)(*ptr) * 0.001;
			//tFloat32 z = ((tFloat32)(*ptr) / (256.0f*256.0f))*2.3f; // Convert from automatic camera range to meters

			p.x = ((tFloat32)x - OD_CAMERA_CX) * z / OD_CAMERA_FX;
//			if(debugModeEnabled) LOG_WARNING(cString::Format("p.x: %f",p.x));
			p.y = ((tFloat32)y - OD_CAMERA_CY) * z / OD_CAMERA_FY;
//			if(debugModeEnabled) LOG_WARNING(cString::Format("p.y %f",p.y));
			p.z = z;
//			if(debugModeEnabled) LOG_WARNING(cString::Format("p.z %f",p.z));

			#ifdef DEBUG_TO_FILE_POINTCLOUD
				if ( !pointcloud_to_file && !firstFrame ) {
					fstream file_pointcloud;
					file_pointcloud.open("/tmp/point_cloud.dat", ios::out | ios::app);
					file_pointcloud << p.x << " " << p.y << " " << p.z << "\n";
					file_pointcloud.close();
				}
			#endif
			/* if debug video should be created, all values are necessary for re-transformation from point cloud vector to image-array
			 *  therefore, no preprocessing will be performed; If NO DEBUG is requested, values in non-interesting/invalid area are
			 *  instantly deleted and not be taken into account in further processing (range can be set via properties) */
			#ifdef DEBUG_MODE_OUTPUT_VIDEO
				m_cloud.push_back(p);
			#else
			//if(p.z > distance_threshold_min  && p.z < distance_threshold_max){ // z-value has to be checked, as it corresponds to  x-value in car-coordinates
			//	if(p.y > height_threshold_max && p.y < height_threshold_min){ // inverted y-value has to be checked, as it corresponds to z-value in car-coordinates
					m_cloud.push_back(p);
			//	}
			//}
			#endif
			/* increase pointer to jump to memory location of next float value */
					//ptr++;
		}
	}

	#ifdef DEBUG_TO_FILE_POINTCLOUD
		if(!firstFrame)pointcloud_to_file = tTrue;
	#endif

	RETURN_NOERROR;
}


tResult ObstacleDetection::transformPointCloud(std::vector<cv::Point3f> &cloud)
{
//	if(debugModeEnabled) LOG_WARNING(cString::Format("TranformPointCloud started"));

	/* Rotational Matrix around x-axis, with translation in negative y and positive z direction;
	 *  pitch angle might have to be adapted (sign) */
//	cv::Mat T = (cv::Mat_<tFloat32>(4, 4) <<
//		1.0,  0.0, 0.0, 0.0,
//		0.0,  cos_m_pitch_rad, -sin_m_pitch_rad, -0.22,
//		0.0,  sin_m_pitch_rad, cos_m_pitch_rad, 0.2,
//		0.0,  0.0, 0.0, 1.0
//	);
	/* Transformation matrix coefficients; rotation and translation ;
	 * z points straight in front of car , y points downwards to the ground, x points to the right hand side! */
	const tFloat32 T_xx = 1.0f;
	const tFloat32 T_xy = 0.0f;
	const tFloat32 T_xz = 0.0f;
	const tFloat32 T_xo = camera_y_offset; // 0.0f; property is in car-coordinates !
	const tFloat32 T_yx = 0.0f;
	const tFloat32 T_yy = cos_m_pitch_rad;
	const tFloat32 T_yz = -sin_m_pitch_rad;
	const tFloat32 T_yo = camera_z_offset; // -0.22f; property is in car-coordinates !
	const tFloat32 T_zx = 0.0f;
	const tFloat32 T_zy = sin_m_pitch_rad;
	const tFloat32 T_zz = cos_m_pitch_rad;
	const tFloat32 T_zo = -camera_x_offset; //-0.2f; property is in car-coordinates !
//	const tFloat32 T_ox = 0.0f;
//	const tFloat32 T_oy = 0.0f;
//	const tFloat32 T_oz = 0.0f;
//	const tFloat32 T_oo = 1.0f;

	if(cloud.size() <= 0){
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("ObstDetec: Referenced pointcloud has invalid size: %d",cloud.size()));
	}

	/* perform actual transformation */
	for(tUInt32 i = 0; i < cloud.size(); i++){
		tFloat32 tmp_x = (T_xx * cloud[i].x) +  (T_xy * cloud[i].y) + (T_xz * cloud[i].z) + (T_xo * 1.0f);
		tFloat32 tmp_y = (T_yx * cloud[i].x) +  (T_yy * cloud[i].y) + (T_yz * cloud[i].z) + (T_yo * 1.0f);
		tFloat32 tmp_z = (T_zx * cloud[i].x) +  (T_zy * cloud[i].y) + (T_zz * cloud[i].z) + (T_zo * 1.0f);

		/* assign tmp_vals to point cloud values, change to car-coordinates */
		cloud[i].x =  tmp_z;
		cloud[i].y = -tmp_x;
		cloud[i].z = -tmp_y;
		#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
			if (!transformed_pointcloud_to_file && pointcloud_to_file ) {
				if(debugModeEnabled)LOG_WARNING(cString::Format("Transformed logging."));
				fstream file_pointcloud_trans;
				file_pointcloud_trans.open("/tmp/point_cloud_trans.dat", ios::out | ios::app);
				file_pointcloud_trans << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << "\n";
				file_pointcloud_trans.close();
				if(!transformed_logging_completed){
					transformed_logging_completed = tTrue;
				}
			}
		#endif
	}

	#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
		if(transformed_logging_completed){
			transformed_pointcloud_to_file = tTrue;
			//if(debugModeEnabled) LOG_WARNING(cString::Format("TranformPointCloud completed."));
		}
	#endif
	RETURN_NOERROR;
}


#ifdef DEBUG_MODE_OUTPUT_VIDEO
cv::Mat ObstacleDetection::generateDebugDepthFromTransformedCloud(const std::vector<cv::Point3f> &cloud)
{
	cv::Mat depth_with_obstacles(inputFormat.nHeight, inputFormat.nWidth, CV_16UC1);

	tBool adaptDepthColor = tFalse;

	for(tUInt32 i = 0 ; i<cloud.size() ; i++) {
		const tUInt32 u = i % inputFormat.nWidth; // columns
		const tUInt32 v = i / inputFormat.nWidth; // rows

		tBool obstacle_detected = tFalse;

		if(debugType == 4){
			adaptDepthColor = tTrue;
		}
		else{
			/* first take only the valid field of view for the camera to check rest */
			if(cloud[i].x > distance_threshold_min  && cloud[i].x < distance_threshold_max){
				if(cloud[i].z > height_threshold_min && cloud[i].z < height_threshold_max){

					/* check conditions depending on debug mode, e.g. OnComing, Cross, both */
					// debugType: 1 for 'check both areas', 2 for 'oncoming traffic', 3 for 'crosstrafic right', 4 for 'no modification'

					switch(debugType){
							case 1:	/* check both areas for obstacles and mark them in white */
								/* check for oncoming traffic at intersection */
								if((cloud[i].x >= roi_intersection_onComingTraffic.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_onComingTraffic.bottomleft_corner.x + roi_intersection_onComingTraffic.roi_x_height))) {
									if((cloud[i].y <= roi_intersection_onComingTraffic.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_onComingTraffic.bottomleft_corner.y + roi_intersection_onComingTraffic.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								/* check for crossTraffic from right hand side at intersection */
								if((cloud[i].x >= roi_intersection_crossTrafficRight.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_crossTrafficRight.bottomleft_corner.x + roi_intersection_crossTrafficRight.roi_x_height))) {
									if((cloud[i].y <= roi_intersection_crossTrafficRight.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_crossTrafficRight.bottomleft_corner.y + roi_intersection_crossTrafficRight.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 2: /* check for oncoming traffic at intersection */
								if((cloud[i].x >= roi_intersection_onComingTraffic.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_onComingTraffic.bottomleft_corner.x + roi_intersection_onComingTraffic.roi_x_height))) {
									if((cloud[i].y <= roi_intersection_onComingTraffic.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_onComingTraffic.bottomleft_corner.y + roi_intersection_onComingTraffic.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 3: /* check for crosstraffic from right hand side at intersection */
								if((cloud[i].x >= roi_intersection_crossTrafficRight.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_crossTrafficRight.bottomleft_corner.x + roi_intersection_crossTrafficRight.roi_x_height))) {
									if((cloud[i].y <= roi_intersection_crossTrafficRight.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_crossTrafficRight.bottomleft_corner.y + roi_intersection_crossTrafficRight.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 4: /* this point cannot be reached */
								LOG_ERROR(cString::Format("ObstacleDetec: Debug Error, invalid state reached during debug-video-creation!"));
								break;
							case 5: /* check for oncoming traffic in driving mode */
								if((cloud[i].x >= roi_overtake_onComingTraffic.bottomleft_corner.x) && (cloud[i].x <= (roi_overtake_onComingTraffic.bottomleft_corner.x + roi_overtake_onComingTraffic.roi_x_height))) {
									if((cloud[i].y <= roi_overtake_onComingTraffic.bottomleft_corner.y) && (cloud[i].y >= (roi_overtake_onComingTraffic.bottomleft_corner.y + roi_overtake_onComingTraffic.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 6: /* check for traffic on 'own' lane during overtake, before going back into lane */
								if((cloud[i].x >= roi_overtake_originalLane.bottomleft_corner.x) && (cloud[i].x <= (roi_overtake_originalLane.bottomleft_corner.x + roi_overtake_originalLane.roi_x_height))) {
									if((cloud[i].y <= roi_overtake_originalLane.bottomleft_corner.y) && (cloud[i].y >= (roi_overtake_originalLane.bottomleft_corner.y + roi_overtake_originalLane.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 7: /* check for traffic on 'own' lane during overtake, before going back into lane */
								if((cloud[i].x >= roi_overtake_ownLaneStraight.bottomleft_corner.x) && (cloud[i].x <= (roi_overtake_ownLaneStraight.bottomleft_corner.x + roi_overtake_ownLaneStraight.roi_x_height))) {
									if((cloud[i].y <= roi_overtake_ownLaneStraight.bottomleft_corner.y) && (cloud[i].y >= (roi_overtake_ownLaneStraight.bottomleft_corner.y + roi_overtake_ownLaneStraight.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 8: /* check for traffic on oncoming lane, right before parking trans */
								if((cloud[i].x >= roi_parking_cross_onComing.bottomleft_corner.x) && (cloud[i].x <= (roi_parking_cross_onComing.bottomleft_corner.x + roi_parking_cross_onComing.roi_x_height))) {
									if((cloud[i].y <= roi_parking_cross_onComing.bottomleft_corner.y) && (cloud[i].y >= (roi_parking_cross_onComing.bottomleft_corner.y + roi_parking_cross_onComing.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
							case 9: /* check for cross-traffic, right before pulling out of maneuver 'parking trans' */
								if((cloud[i].x >= roi_parking_cross_crossTraffic.bottomleft_corner.x) && (cloud[i].x <= (roi_parking_cross_crossTraffic.bottomleft_corner.x + roi_parking_cross_crossTraffic.roi_x_height))) {
									if((cloud[i].y <= roi_parking_cross_crossTraffic.bottomleft_corner.y) && (cloud[i].y >= (roi_parking_cross_crossTraffic.bottomleft_corner.y + roi_parking_cross_crossTraffic.roi_y_width))) {

										obstacle_detected = tTrue;
									}
								}
								break;
					}
				}
			}
		}
		
		if(!adaptDepthColor){
			if(obstacle_detected){
				depth_with_obstacles.at<tUInt16>(v, u) = 256*256-1; // white
			}
			else{
				depth_with_obstacles.at<tUInt16>(v, u) = 0; // black
			}
		}
		else{ //adapt color, but leave rest unmanipulated
			tFloat32 x_tmp = cloud[i].x;
			if(cloud[i].x > 3.0f) x_tmp = 3.0f;
			depth_with_obstacles.at<tUInt16>(v, u) = static_cast<tUInt16>((x_tmp /3.0f)*65535.0f);
		}

	}

	return depth_with_obstacles;
}
#endif



/* method to read the pitch from a previously created file, supposedly by camera-pitch-estimation functionality */
tResult ObstacleDetection::ReadFromFile(tFloat32 *pitch) {
	if (filePitch.GetLength() < 2){
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"ObstacleDetec: No filename for the pitch angle is given! If default should be used, set corresponding property.");
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(filePitch);
	cFilename absFilename = filePitch.CreateAbsolutePath(".");

	if (cFileSystem::Exists(filePitch) == tFalse ) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"ObstacleDetec: File for stored pitch angle does not exist.");
	}

	cFile file;
	file.Open(absFilename.GetPtr(), cFile::OM_Read);

	cString tmp;
	file.ReadLine(tmp); // comment line
	file.ReadLine(tmp);

	if(tmp.GetLength() >= 1) {
		*pitch = tmp.AsFloat64();
	} else {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("ObstacleDetec: Error in opening the file %s. If value set by property should be used, please set corresponding flag!", absFilename.GetPtr()));
	}

	RETURN_NOERROR;
}


TActionStruct::ActionSub ObstacleDetection::GetCurrActionSub(){
	__synchronized_obj(criticalSection_ActionSubAccess);
	return actionSub;
}

tResult ObstacleDetection::SetCurrActionSub(TActionStruct::ActionSub newActionSub){
	__synchronized_obj(criticalSection_ActionSubAccess);
	actionSub = newActionSub;
	RETURN_NOERROR;
}

ObstacleDetection::operationalStatus ObstacleDetection::GetOperationalStatus(){
	__synchronized_obj(criticalSection_OperationalStatusAccess);
	return OACC_operational_status;
}

tResult ObstacleDetection::SetOperationalStatus(operationalStatus opStatus){
	__synchronized_obj(criticalSection_OperationalStatusAccess);
	OACC_operational_status = opStatus;
	RETURN_NOERROR;
}


tBool ObstacleDetection::GetRunningState(){
	__synchronized_obj(criticalSection_RunningStateAccess);
	return runningState;
}

tResult ObstacleDetection::SetRunningState(tBool state){
	__synchronized_obj(criticalSection_RunningStateAccess);
	runningState = state;
	RETURN_NOERROR;
}

TSignalValue::Data ObstacleDetection::GetSteeringAngle(){
	__synchronized_obj(criticalSection_SteeringAngleAccess);
	return curSteeringAngle;
}
tResult ObstacleDetection::SetSteeringAngle(TSignalValue::Data steeringAngle){
	__synchronized_obj(criticalSection_SteeringAngleAccess);
	curSteeringAngle = steeringAngle;
	//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC - steering angle set to: %f.",curSteeringAngle.f32_value));
	RETURN_NOERROR;
}

tFloat32 ObstacleDetection::GetTargetSpeedLimit(){
	__synchronized_obj(criticalSection_TargetSpeedLimitAccess);
	return curTargetSpeedLimit;
}

tResult ObstacleDetection::SetTargetSpeedLimit(tFloat32 targetSpeedLimit){
	__synchronized_obj(criticalSection_TargetSpeedLimitAccess);
	curTargetSpeedLimit = targetSpeedLimit;
	RETURN_NOERROR;
}


/* Method printing the dynamic calculated driving lane - check area */
tResult ObstacleDetection::CreateAndTransmitGCL_dynamicDrive(const TSignalValue::Data steeringAngle){
	// just draw gcl if the pin is connected and debug mode is enabled
	if (!m_oGCLOutput.IsConnected() || !showGCLDebug)
	{
		RETURN_NOERROR;
	}
	std::vector<cv::Point3f> referenceLine;
	cv::Point3f tmpPointLeft;
	cv::Point3f tmpPointRight;
	tFloat32 corr_steeringAngleAbs = ((fabsf(steeringAngle.f32_value - 90.0f) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
	tFloat32 interval = 0.0f;
	/* handle case that steringAngle would be smaller than zero due to obliqueAngle */
	if(corr_steeringAngleAbs < 0.0f){
		corr_steeringAngleAbs = 0.0f;
	}
	/* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
	if(corr_steeringAngleAbs < 0.005f){
		/* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
		tFloat32 x_oacc_rect_min = 0.0f;
		tFloat32 x_oacc_rect_max = OACC_circSegmentDistance - OACC_frontAxisToFrontBumper;
		if(x_oacc_rect_max < 0.0f){
			LOG_WARNING(cString::Format("ObstacleDetec: OACC - search-corridor in x-direction too small, received input target speed will be returned unmodified!"));
		}

		/* Creating 50 x-values in car-coordinate system as supporting points*/
		interval = (x_oacc_rect_max - x_oacc_rect_min)/50.0f;
		for(tUInt32 i = 0; i < 50; i++){
			tmpPointLeft.x = static_cast<tFloat32>(i) * interval;
			tmpPointLeft.y = 0.0f - OACC_ySearchTolerance;
			tmpPointLeft.z = 0.0f;
			tmpPointRight.x = static_cast<tFloat32>(i) * interval;
			tmpPointRight.y = 0.0f + OACC_ySearchTolerance;
			tmpPointRight.z = 0.0f;
			//LOG_WARNING(cString::Format("ObstacleDetec: calculated points straight-mode - x: %f, y: %f",tmpPoint.x,tmpPoint.y));
			referenceLine.push_back(tmpPointLeft);
			referenceLine.push_back(tmpPointRight);
		}

	}
	else{
		/* radius between momentum-pole and middle of front-axis */
		tFloat32 segmentRadius = OACC_wheelbase / sinf(corr_steeringAngleAbs);
		tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
		/* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
		if((segmentAngle + corr_steeringAngleAbs) > CV_PI/2){
			segmentAngle = CV_PI/2 - corr_steeringAngleAbs;
		}

		/* minimum and maximum values of the reference line-segment */
		tFloat32 x_oacc_ref_min = 0.0f;
		tFloat32 x_oacc_ref_max = segmentRadius * sinf(corr_steeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;

		/* Creating 50 x-values in car-coordinate system as supporting points*/
		interval = (x_oacc_ref_max - x_oacc_ref_min)/50.0f;
		for(tUInt32 i = 0; i < 50; i++){
			tmpPointLeft.x = static_cast<tFloat32>(i) * interval;
			tmpPointRight.x = static_cast<tFloat32>(i) * interval;
			/* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value - 90) < 0 */
			tmpPointLeft.y = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((tmpPointLeft.x + OACC_wheelbase)*(tmpPointLeft.x + OACC_wheelbase))) - OACC_ySearchTolerance;
			tmpPointRight.y = (OACC_wheelbase / tanf(corr_steeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((tmpPointRight.x + OACC_wheelbase)*(tmpPointRight.x + OACC_wheelbase))) + OACC_ySearchTolerance;
			if((steeringAngle.f32_value - 90) > 0){
				tmpPointLeft.y = - tmpPointLeft.y;
				tmpPointRight.y = -tmpPointRight.y;
			}
			//LOG_WARNING(cString::Format("ObstacleDetec: calculated points dyn-mode - x: %f, y: %f",tmpPoint.x,tmpPoint.y));
			tmpPointLeft.z = 0.0f;
			tmpPointRight.z = 0.0f;
			referenceLine.push_back(tmpPointLeft);
			referenceLine.push_back(tmpPointRight);
		}


	}


	/* Transformation matrix coefficients; rotation and translation ;
	 * z points straight in front of car , y points downwards to the ground, x points to the right hand side! */
	tFloat32 RT_xx = 1.0f;
	tFloat32 RT_xy = 0.0f;
	tFloat32 RT_xz = 0.0f;
	tFloat32 RT_xo = camera_y_offset + camera_DepthToRgbOffest; // 0.0f; property is in car-coordinates !
	tFloat32 RT_yx = 0.0f;
//	tFloat32 RT_yy = cos_m_pitch_rad;
	tFloat32 RT_yy = cos_m_dynRange_rad;
//	tFloat32 RT_yz = -sin_m_pitch_rad;
	tFloat32 RT_yz = -sin_m_dynRange_rad;
	tFloat32 RT_yo = camera_z_offset; // -0.22f; property is in car-coordinates !
	tFloat32 RT_zx = 0.0f;
//	tFloat32 RT_zy = sin_m_pitch_rad;
	tFloat32 RT_zy = sin_m_dynRange_rad;
//	tFloat32 RT_zz = cos_m_pitch_rad;
	tFloat32 RT_zz= cos_m_dynRange_rad;
	tFloat32 RT_zo = -camera_x_offset; //-0.2f; property is in car-coordinates !
//	const tFloat32 T_ox = 0.0f;
//	const tFloat32 T_oy = 0.0f;
//	const tFloat32 T_oz = 0.0f;
//	const tFloat32 T_oo = 1.0f;

	if(referenceLine.size() <= 0){
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("ObstDetec: GCL-dynamic: Invalid size of vector: %d",referenceLine.size()));
	}

	/* perform actual transformation */
	for(tUInt32 i = 0; i < referenceLine.size(); i++){


		/* Re-assign transformed point cloud values to original pointcloud values, change from car-coordinates to depth-image coordinates */
		tFloat32 tmp_z = referenceLine[i].x;
		tFloat32 tmp_x = - referenceLine[i].y;
		tFloat32 tmp_y = - referenceLine[i].z;

		/* translation backwards */
		tmp_x = tmp_x - (RT_xo * 1.0f);
		tmp_y = tmp_y - (RT_yo * 1.0f);
		tmp_z = tmp_z - (RT_zo * 1.0f);
//		LOG_WARNING(cString::Format("ObstacleDetec: After back-translation - x: %f, y: %f, z: %f",tmp_x,tmp_y,tmp_z));
		/* rotation backwards-- maybe first rotate pi/2, then translate, then rotate pitch */

		referenceLine[i].x = (RT_xx * tmp_x) +  (RT_xy * tmp_y) + (RT_xz * tmp_z);
		referenceLine[i].y = (RT_yx * tmp_x) +  (RT_yy * tmp_y) + (RT_yz * tmp_z);
		referenceLine[i].z = (RT_zx * tmp_x) +  (RT_zy * tmp_y) + (RT_zz * tmp_z);

//		LOG_WARNING(cString::Format("ObstacleDetec: After back-rotation- x: %f, y: %f, z: %f",referenceLine[i].x,referenceLine[i].y,referenceLine[i].z));
//		LOG_WARNING(cString::Format("ObstacleDetec: After back-rotation- sin: %f, cos: %f",sin_m_dynRange_rad, cos_m_dynRange_rad));

	}
	/* At this point, the point-cloud data is transformed back to original orientation from camera view */
	/* Now, point cloud needs to be transformed back to depth-image frame-resolution and format */

	for(tUInt32 i = 0; i < referenceLine.size(); i++){

		referenceLine[i].x =  ((referenceLine[i].x * OD_CAMERA_FX) / referenceLine[i].z) +  OD_CAMERA_CX;
		referenceLine[i].y = ((referenceLine[i].y * OD_CAMERA_FY) / referenceLine[i].z) + OD_CAMERA_CY;
		//referenceLine[i].z = referenceLine[i].z;
//		LOG_WARNING(cString::Format("ObstacleDetec: After depth-back-trafo- x: %f, y: %f, z: %f",referenceLine[i].x,referenceLine[i].y,referenceLine[i].z));
	}

	if(showGCLDebug_extendedLog){
		LOG_WARNING(cString::Format("ObstacleDetec: GCL -> should print %d elements.",referenceLine.size()));
	}


	// create a mediasample for GCL output
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

	RETURN_IF_FAILED(pSample->AllocBuffer(8192));

	pSample->SetTime(_clock->GetStreamTime());

	tUInt32* aGCLProc;
	RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

	tUInt32* pc = aGCLProc;

	/* Do the actual drawing -> create rectangles representing the search corridor */
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 70, 0).GetRGBA());
	/* Draw a rectangle to scale the GCL output to the video display */
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, 2*inputFormat.nWidth, 2*inputFormat.nHeight);

	if(showGCLDebug_extendedLog)LOG_WARNING(cString::Format("ObstDetec: GCL-dynamic: Point to plot: x:%d, y: %d",inputFormat.nWidth,inputFormat.nHeight));
//	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
	if((referenceLine.size()-1) * 24 < 8192){
		for(tUInt32 i = 0; i < referenceLine.size();i = i+2){
			/* Set Color */
			//cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, ((static_cast<tInt>(referenceLine[i].x) + (inputFormat.nWidth / 2)) * 2) - 2, ((static_cast<tInt>(referenceLine[i].y) + (inputFormat.nHeight / 2)) * 2) - 2,((static_cast<tInt>(referenceLine[i].x) + (inputFormat.nWidth / 2)) * 2) + 2, ((static_cast<tInt>(referenceLine[i].y) + (inputFormat.nHeight / 2)) * 2) + 2);
//			if(i < 40){
//				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,0,0).GetRGBA()); // red
//			}
//			else if((i >= 40) && (i < 80)){
//				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,238,0).GetRGBA()); // yellow
//			}
//			else{ // i>=80
//				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,238,0).GetRGBA()); // green
//			}
			if(interval > ((OACC_circSegmentDistance - OACC_frontAxisToFrontBumper)/50.0f)*0.92f){
				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,238,0).GetRGBA()); // green
			}
			else if((interval < ((OACC_circSegmentDistance - OACC_frontAxisToFrontBumper)/50.0f)*0.92f) && (interval > ((OACC_circSegmentDistance - OACC_frontAxisToFrontBumper)/50.0f)*0.35f)){
				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,238,0).GetRGBA()); // yellow
			}
			else{
				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,0,0).GetRGBA()); // red
			}
			/* Draw Rectangle */
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, (static_cast<tInt>(referenceLine[i].x))*2 - 2, (static_cast<tInt>(referenceLine[i].y)* 2) - 2,(static_cast<tInt>(referenceLine[i+1].x) * 2) + 2, (static_cast<tInt>(referenceLine[i+1].y)* 2) + 2);
			if(showGCLDebug_extendedLog){
				LOG_WARNING(cString::Format("ObstDetec: GCL-dynamic: Point to plot: x_left: %f, y_left: %f, x_right: %f, y_right: %f",referenceLine[i].x,referenceLine[i].y,referenceLine[i+1].x,referenceLine[i+1].y));
			}
		}
	}
	else{
		LOG_WARNING(cString::Format("ObstacleDetec: GCL -> too many points to draw, exit."));
	}
	cGCLWriter::StoreCommand(pc, GCL_CMD_END);

	pSample->Unlock(aGCLProc);
	RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));


	RETURN_NOERROR;
}



tResult ObstacleDetection::CreateAndTransmitGCL(const std::vector<cv::Point2i> &referenceSegmentLine){
	LOG_WARNING(cString::Format("ObstacleDetec: GCL -> should print %d circles.",referenceSegmentLine.size()));
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput.IsConnected() || !showGCLDebug)
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(8192));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;

    // draw rectangle to scale the video display correctly
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, 2*inputFormat.nWidth, 2*inputFormat.nHeight);

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
	if(referenceSegmentLine.size() * 24 < 8192){
		for(tUInt32 i = 0; i < referenceSegmentLine.size();i++){
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, referenceSegmentLine[i].x*2 - 2, referenceSegmentLine[i].y * 2 - 2,referenceSegmentLine[i].x * 2 + 2, referenceSegmentLine[i].y * 2 + 2);
		}
	}
	else{
		LOG_WARNING(cString::Format("ObstacleDetec: GCL -> too many points to draw, exit."));
	}
	//	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, inputFormat.nWidth - inputFormat.nWidth/2, inputFormat.nHeight - inputFormat.nHeight/2, inputFormat.nWidth + inputFormat.nWidth/2, inputFormat.nHeight + inputFormat.nHeight/2);

//    // draw the min and max lane width
//    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());
//    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, referenceSegmentLine[i].x - m_i16LaneWidthMaxFar/2, m_nFarLine, m_i16FarLaneCenter + m_i16LaneWidthMaxFar/2, m_nFarLine + 5);
//    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMinFar/2, m_nFarLine - 5, m_i16FarLaneCenter + m_i16LaneWidthMinFar/2, m_nFarLine);
//
//    // draw near and far line
//    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
//    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nCurrentNearLine, m_sInputFormat.nWidth, m_nCurrentNearLine);
//    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nFarLine, m_sInputFormat.nWidth, m_nFarLine);


    // draw the found near lines
//    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,50,100).GetRGBA());
//    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, referenceSegment[0].x*2 , referenceSegment[0].y*2,referenceSegment[0].x*2 + 10, referenceSegment[0].y*2 + 10);

//    // draw the found far lines
//    for (tInt nIdx = 0; nIdx < m_ui8FarPointsCount; ++nIdx)
//    {
//        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsFar[nIdx].x, m_nFarLine - 20, m_asAllpointsFar[nIdx].x, m_nFarLine + 20);
//    }

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

/* returns gridcell resolution based on property */
tUInt32 ObstacleDetection::GetResolutionFromIndicator(tUInt32 OACC_GridCellResolutionIndicator){
	tUInt32 gridCellResolution = 0;
	switch(OACC_GridCellResolutionIndicator){
		case 1:
			gridCellResolution = 2;
			break;
		case 2:
			gridCellResolution = 4;
			break;
		case 3:
			gridCellResolution = 5;
			break;
		case 4:
			gridCellResolution = 8;
			break;
		case 5:
			gridCellResolution = 10;
			break;
		case 6:
			gridCellResolution = 16;
			break;
		case 7:
			gridCellResolution = 20;
			break;
	}
	return gridCellResolution;
}

tResult ObstacleDetection::LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues) {

	// check if file exits
	if (m_fileConfig.IsEmpty()) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("ObstacleDetec: Configuration xml-file not found."));
	}
	// create absolute path
	ADTF_GET_CONFIG_FILENAME(m_fileConfig);
	m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

	//Load file, parse configuration, print the data

	if (cFileSystem::Exists(m_fileConfig)) {
		cDOM oDOM;
		oDOM.Load(m_fileConfig);
		//load supporting points
		cDOMElementRefList oElems;
		if (IS_OK(oDOM.FindNodes("calibrationOpticACC/supportingPoints/point", oElems))) {
			for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {
				cDOMElement* pConfigElement;
				if (IS_OK((*itElem)->FindNode("xValue", pConfigElement))) {
					m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
				}
				if (IS_OK((*itElem)->FindNode("yValue", pConfigElement))) {
					m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
				}
			}
		}
		if (oElems.size() > 0) {
			if (xml_PrintInitialtable) {
				for (tUInt i = 0; i < m_xValues.size(); i++) {
					if (i > m_yValues.size()){
						break;
					}
					else{
						LOG_WARNING(cString::Format("ObstacleDetec: supporting point #%d: (%lf / %lf)", i, m_xValues[i],  m_yValues[i]));
					}
				}
			}
		} else {
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"ObstacleDetec: no supporting points in given file found!");
		}
		//checks if data are valid
		RETURN_IF_FAILED(CheckConfigurationData(m_fileConfig,m_xValues));

	} else {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"ObstacleDetec: Configured configuration file not found");
	}

	RETURN_NOERROR;
}

tResult ObstacleDetection::CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues) {
	//checks if the xValues of the calibration table are increasing
	for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end(); it++) {
		vector<tFloat32>::iterator it2 = it;
		it2++;
		if (it2 != m_xValues.end()) {
			// next values is smaller than current value
			if ((tFloat32(*it) > tFloat32(*it2))) {
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("ObstacleDetec: The x-values in the file %s are not in increasing order. Please reorder the points!", m_fileConfig.GetPtr()));
			}
		}
	}

	RETURN_NOERROR;
}

tFloat32 ObstacleDetection::GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues) {
	// requested value is smaller than smallest value in table
	if (fl32InputValue < m_xValues.front()) {
		if (xml_BorderWarningModeEnabled){
			LOG_WARNING(cString::Format("ObstacleDetec: requested x-value %f is lower than smallest x-value in calibration table", fl32InputValue));
		}
		return m_yValues.front();
	}
	// requested value is bigger than biggest value in table
	else if (fl32InputValue > m_xValues.back()) {
		if (xml_BorderWarningModeEnabled){
			LOG_WARNING(cString::Format("ObstacleDetec: requested x-value %f is higher than highes x-value in calibration table", fl32InputValue));
		}
		return m_yValues.back();
	}
	// search in vector for corresponding index (smallest neighbor)
	tUInt iIndex;
	if (m_xValues.size() > 2) {
		for (iIndex = 0; iIndex < m_xValues.size(); iIndex++) {
			if (m_xValues[iIndex] >= fl32InputValue){
				break;
			}
		}
		// get smaller neighbor
		if (iIndex != 0){
			iIndex = iIndex - 1;
		}
	}
	else{
		iIndex = 0;
	}
	if ((m_xValues[iIndex + 1] - m_xValues[iIndex]) != 0) {
		// doing the linear interpolation
		tFloat32 f32Value = (fl32InputValue - m_xValues[iIndex]) * (m_yValues[iIndex + 1] - m_yValues[iIndex])
				/ (m_xValues[iIndex + 1] - m_xValues[iIndex]) + m_yValues[iIndex];

		//tFloat32 security check to send only minimum or maximum value of table
		if (f32Value > *max_element(m_yValues.begin(), m_yValues.end())){
			return *max_element(m_yValues.begin(), m_yValues.end());
		}
		else if (f32Value < *min_element(m_yValues.begin(), m_yValues.end())){
			return *min_element(m_yValues.begin(), m_yValues.end());
		}
		else{
			return f32Value;
		}
	}
	else {
		LOG_ERROR("ObstacleDetec: Error occured. Invalid table in xml-config-file!");
		return -1.0f;
	}
}


