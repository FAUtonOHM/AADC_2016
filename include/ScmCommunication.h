/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************************
* $Author:: mahill $  $Date:: 2016-01-30 13:49:07#$ $Rev:: 0.0.3  $Team FAUtonOHM $
**********************************************************************************/

#ifndef _SCMCOMMUNICATION_ENUMS
#define _SCMCOMMUNICATION_ENUMS

/* enum for communication between filter and StateControlManagement */
enum FilterIDs{
	/* F_ stands for Filter */
	F_LINE_SPECIFIER = 1,
	F_FOLLOW_PATH = 2,
	F_ROAD_SIGNS = 3,
	F_TIMER = 4,
	F_ULTRASONIC_CHECK = 5,
	F_OBSTACLE_DETECTION = 6,
	F_LIGHT_FILTER = 7,
	F_STOP_ACTION = 8,
	F_ULTRASONIC_ACC = 9,
	F_JURY_COMMUNICATION = 10,
	F_LIGHT_MESSAGE_RECEIVER = 11,
	F_LIGHT_MESSAGE_TRANSMITTER = 12,
	F_LIGHT_MESSAGE_LOGGER = 13,
};

enum FeedbackStatus{
	/** FB_ stands for FeedBack **/


	/* *****************
	 *  LineSpecifier  *
	 * *****************/
	FB_LS_STOPLINE = 1000,
	FB_LS_CROSSING_WAIT_POSITION = 1001,
	FB_LS_STOP_WITH_SIGN = 1002,

	/* Different Activation Modes of LineSpecifier*/
	FB_LS_STOPPED = 1011,
	FB_LS_SLOW_RIGHTLANE = 1021,
	FB_LS_SLOW_LEFTLANE = 1026,
	FB_LS_NORMAL_RIGHTLANE = 1031,
	FB_LS_NORMAL_LEFTLANE = 1036,
	FB_LS_NOSPEED = 1041,
	FB_LS_NOSPEED_BACKWARDS = 1051,
	/* parking maneuver feedback */
	/* Type of spot the car is currently parked in */
	FB_LS_IN_PARKSPOT_TRANS = 1101 , // by checking whether horizontal line is in sight
	FB_LS_IN_PARKSPOT_LONG = 1102,
	/* Detecting parallel parking spot before parking maneuver */
	FB_LS_PARKING_LONG = 1111,
	FB_LS_NO_PARKING_LONG = 1112,
	/* Detecting transversal parking spot before parking maneuver */
	FB_LS_PARKING_TRANS = 1121,
	FB_LS_NO_PARKING_TRANS = 1122,
	/* crossing maneuver feedback */
	/* detecting lines of crossing to distinguish type and direction */
	FB_LS_CROSSING_LINE_HORIZONTAL = 1201,
	FB_LS_CROSSING_LINE_VERTICAL= 1202,
	FB_LS_CROSSING_LINE_NONE = 1203,


	/* **************
	 *  FollowPath  *
	 * **************/
	/* Car reached certain point, given by xy-coordinates */
	FB_FP_REACHED_POINT = 2011,
	FB_FP_REACHED_POINT_NOSTOP = 2016,
	/* Car completed corresponding turn successfully*/
	FB_FP_LEFT_TURN = 2021,
	FB_FP_RIGHT_TURN = 2031,
	/* Car completed corresponding parking maneuver successfully*/
	FB_FP_PARKING_TRANS = 2041,
	FB_FP_PARKING_OUT_TRANS_RIGHT = 2051,
	FB_FP_PARKING_OUT_TRANS_RIGHT_EXTENDED = 2056,
	FB_FP_PARKING_OUT_TRANS_LEFT = 2061,
	FB_FP_PARKING_OUT_TRANS_LEFT_EXTENDED = 2066,
	FB_FP_PARKING_LONG = 2071,
	FB_FP_PARKING_LONG_BADYAW = 2072,
	/* Pull Out Right Parallel Parking */
	FB_FP_PARKING_OUT_LONG = 2081,
	FB_FP_PARKING_OUT_LONG_NORESET = 2086,
	/* Straight Paths for crossings and parking spot detection */
	FB_FP_GO_STRAIGHT = 2091,
	FB_FP_STRAIGHT_PARKING_LONG_FORW = 2101,
	FB_FP_STRAIGHT_PARKING_LONG_BACK = 2111,
	FB_FP_STRAIGHT_PARKING_LONG_ALLBACK = 2116, // return to first parking space
	FB_FP_STRAIGHT_PARKING_TRANS = 2121,
	FB_FP_STRAIGHT_PARKING_TRANS_BACK = 2131,
	FB_FP_STRAIGHT_PARKING_TRANS_ALLBACK = 2136, // return to first parking space
	/* Filter stopped/paused or was activated again*/
	FB_FP_STOPPED = 2141,
	FB_FP_CONTINUED = 2146,
	/* Get last completed parking maneuver */
	FB_FP_PARKING_STATUS_NO_PARKING = 2151,
	FB_FP_PARKING_STATUS_PARALLEL = 2152,
	FB_FP_PARKING_STATUS_TRANSVERSE = 2153,
	/*Overtaking*/
	FB_FP_GO_BACKWARDS_BEFORE_OVERTAKING = 2161,
	FB_FP_CHANGE_TO_ONCOMING_LANE = 2171,
	FB_FP_OVERTAKE_CONSTRUCTION_SITE = 2176,
	FB_FP_CHANGE_TO_ORIGINAL_LANE = 2181,
	/* Car should drive to certain point, given by xy-coordinates */
	FB_FP_GOTO_XY_NOSTEERING = 2191,
	/* FB_FP_GOTO_XY_NOSTEERING_BACKWARDS = 2196, */ // Not implemented yet
	FB_FP_GOTO_XY_NOSTEERING_NOSTOP = 2201,
	/* Parking out parallel (extended: go backwards after pull out) */
	FB_FP_PARKING_OUT_LONG_EXTENDED = 2221,
	FB_FP_PARKING_OUT_LONG_NORESET_EXTENDED = 2226,
	/* Correct parking parallel if YAW is too bad */
	FB_FP_PARKING_LONG_CORRECTION = 2241,

	/* *************
	 *  RoadSign:  *
	 * *************/
	FB_RS_UNMARKEDINTERSECTION = 3000,
	FB_RS_STOPANDGIVEWAY = 3001,
	FB_RS_PARKINGAREA = 3002,
	FB_RS_HAVEWAY = 3003,
	FB_RS_AHEADONLY = 3004,
	FB_RS_GIVEWAY = 3005,
	FB_RS_PEDESTRIANCROSSING = 3006,
	FB_RS_ROUNDABOUT = 3007,
	FB_RS_NOOVERTAKING = 3008,
	FB_RS_NOENTRYVEHICULARTRAFFIC = 3009,
	FB_RS_ONEWAYSTREET = 3011,

	FB_RS_CROSSING_SIGN_PRIORITY = 3111,
	FB_RS_CROSSING_SIGN_GIVEWAY = 3112,
	FB_RS_CROSSING_SIGN_NONE = 3113,

	FB_RS_CROSSING_SIGN_RESET = 3121,

	/* command for getting parking sign received status */
	FB_RS_PARKING_SIGN_RECEIVED = 3131,
	FB_RS_NO_PARKING_SIGN_RECEIVED = 3132,

	/* command for resetting parking sign status */
	FB_RS_PARKING_SIGN_RESET = 3141,


	/* ***************
	 *  TimerFilter  *
	 * ***************/
	FB_TF_FINISHED = 4001,


	/* ******************
	 *  UltrasonicCheck *
	 * ******************/
	FB_UC_NO_OBSTACLE = 5001,
	FB_UC_OBSTACLE = 5002,
	FB_UC_ERROR_CODE = 5999,


	/* *********************
	 *  ObstacleDetection  *
	 * *********************/
	FB_OD_NO_OBSTACLE = 6001,
	FB_OD_STATIC_OBSTACLE = 6011,


	/* ***************
	 *  LightFilter  *
	 * ***************/
	/* Status from LightFilter = sent Action Command */



	/* **************
	 *  StopAction  *
	 * **************/
	FB_SA_STOPPED = 8011,


	/* *******************
	 *   UltrasonicACC*  *
	 * *******************/
	FB_UA_NO_MOVEMENT = 9111,
	FB_UA_MOVING_AGAIN = 9121,


	/* *********************
	 *  JuryCommunication  *
	 * *********************/
	FB_JURY_ALIVE_AND_READY = 10991,
	/* Maneuver relevant feedback */
	FB_JURY_MAN_LEFT = 10011,
	FB_JURY_MAN_RIGHT = 10021,
	FB_JURY_MAN_STRAIGHT = 10031,
	FB_JURY_MAN_PARALLEL_PARKING = 10041,
	FB_JURY_MAN_CROSS_PARKING = 10051,
	FB_JURY_MAN_PULL_OUT_LEFT = 10061,
	FB_JURY_MAN_PULL_OUT_RIGHT = 10071,
	FB_JURY_NO_MAN_REMAINING = 10091,
	/* Maneuver feedback for previous maneuvers */
	/* Maneuver relevant feedback */
	FB_JURY_PREVIOUS_MAN_LEFT = 10211,
	FB_JURY_PREVIOUS_MAN_RIGHT = 10221,
	FB_JURY_PREVIOUS_MAN_STRAIGHT = 10231,
	FB_JURY_PREVIOUS_MAN_PARALLEL_PARKING = 10241,
	FB_JURY_PREVIOUS_MAN_CROSS_PARKING = 10251,
	FB_JURY_PREVIOUS_MAN_PULL_OUT_LEFT = 10261,
	FB_JURY_PREVIOUS_MAN_PULL_OUT_RIGHT = 10271,
	FB_JURY_PREVIOUS_NO_MAN_EXISTING = 10291,
	/* Communication & carState relevant feedback */
	//FB_JURY_CARSTATE_STARTUP = 10311,
	//FB_JURY_CARSTATE_READY = 10321,
	//FB_JURY_CARSTATE_RUNNING = 10331,
	//FB_JURY_CARSTATE_COMPLETE = 10341,
	FB_JURY_CARSTATE_ERROR_SECTION_RESET = 10351,
	/* Waiting for commands sent from jury */
	//FB_JURY_COMMAND_RECEIVED_GETREADY = 10510,
	FB_JURY_COMMAND_RECEIVED_START = 10521,
	FB_JURY_COMMAND_RECEIVED_STOP = 10531,


	/* *********************
	 *  LightMessageReceiver  *
	 * *********************/
	FB_LMR_RECEIVE_START = 11011,
	FB_LMR_RECEIVE_START_CONFIRMATION = 11021,
	/* Slave Direction */
	FB_LMR_RECEIVE_REQUEST_SLAVE_DIRECTION = 11031,
	FB_LMR_RECEIVE_SLAVE_TURN_LEFT = 11041,
	FB_LMR_RECEIVE_SLAVE_TURN_RIGHT = 11051,
	FB_LMR_RECEIVE_SLAVE_STRAIGHT = 11061,
	/* Master Direction */
	FB_LMR_RECEIVE_MASTER_TURN_LEFT = 11071,
	FB_LMR_RECEIVE_MASTER_TURN_RIGHT = 11081,
	FB_LMR_RECEIVE_MASTER_STRAIGHT = 11091,
	/* Conflict */
	FB_LMR_RECEIVE_CONFLICT_SITUATION = 11101,
	FB_LMR_RECEIVE_PRIORITY_QUESTION = 11111,
	FB_LMR_RECEIVE_PRIORITY_ANSWER = 11121,
	/* Farewell */
	FB_LMR_RECEIVE_MASTER_GOODBYE = 11131,
	FB_LMR_RECEIVE_SLAVE_GOODBYE = 11141,
	FB_LMR_RECEIVE_END_COMMUNICATION = 11151,

	/* *********************
	 *  LightMessageTransmitter  *
	 * *********************/
	FB_LMT_TRANSMIT_START = 12011,
	FB_LMT_TRANSMIT_START_CONFIRMATION = 12021,
	/* Slave Direction */
	FB_LMT_TRANSMIT_REQUEST_SLAVE_DIRECTION = 12031,
	FB_LMT_TRANSMIT_SLAVE_TURN_LEFT = 12041,
	FB_LMT_TRANSMIT_SLAVE_TURN_RIGHT = 12051,
	FB_LMT_TRANSMIT_SLAVE_STRAIGHT = 12061,
	/* Master Direction */
	FB_LMT_TRANSMIT_MASTER_TURN_LEFT = 12071,
	FB_LMT_TRANSMIT_MASTER_TURN_RIGHT = 12081,
	FB_LMT_TRANSMIT_MASTER_STRAIGHT = 12091,
	/* Conflict */
	FB_LMT_TRANSMIT_CONFLICT_SITUATION = 12101,
	FB_LMT_TRANSMIT_PRIORITY_QUESTION = 12111,
	FB_LMT_TRANSMIT_PRIORITY_ANSWER = 12121,
	/* Farewell */
	FB_LMT_TRANSMIT_MASTER_GOODBYE = 12131,
	FB_LMT_TRANSMIT_SLAVE_GOODBYE = 12141,
	FB_LMT_TRANSMIT_END_COMMUNICATION = 12151,

	/* *********************
	 *  LightMessageLogger  *
	 * *********************/
	FB_LML_LOG_START_SENDER = 13011,
	FB_LML_LOG_START_RECEIVER = 13016,
	FB_LML_LOG_STARTCONFIRMATION_SENDER = 13021,
	FB_LML_LOG_STARTCONFIRMATION_RECEIVER = 13026,
	/* Slave Direction */
	FB_LML_LOG_REQUEST_SLAVE_DIRECTION_SENDER = 13031,
	FB_LML_LOG_REQUEST_SLAVE_DIRECTION_RECEIVER = 13036,
	FB_LML_LOG_SLAVE_TURN_LEFT_SENDER = 13041,
	FB_LML_LOG_SLAVE_TURN_LEFT_RECEIVER = 13046,
	FB_LML_LOG_SLAVE_TURN_RIGHT_SENDER = 13051,
	FB_LML_LOG_SLAVE_TURN_RIGHT_RECEIVER = 13056,
	FB_LML_LOG_SLAVE_STRAIGHT_SENDER = 13061,
	FB_LML_LOG_SLAVE_STRAIGHT_RECEIVER = 13066,
	/* Master Direction */
	FB_LML_LOG_MASTER_TURN_LEFT_SENDER = 13071,
	FB_LML_LOG_MASTER_TURN_LEFT_RECEIVER = 13076,
	FB_LML_LOG_MASTER_TURN_RIGHT_SENDER = 13081,
	FB_LML_LOG_MASTER_TURN_RIGHT_RECEIVER = 13086,
	FB_LML_LOG_MASTER_STRAIGHT_SENDER = 13091,
	FB_LML_LOG_MASTER_STRAIGHT_RECEIVER = 13096,
	/* Conflict Situation */
	FB_LML_LOG_CONFLICT_SITUATION_SENDER = 13101,
	FB_LML_LOG_CONFLICT_SITUATION_RECEIVER = 13106,
	FB_LML_LOG_PRIORITY_QUESTION_SENDER = 13111,
	FB_LML_LOG_PRIORITY_QUESTION_RECEIVER = 13116,
	FB_LML_LOG_PRIORITY_ANSWER_SENDER = 13121,
	FB_LML_LOG_PRIORITY_ANSWER_RECEIVER = 13126,
	/* Farewell */
	FB_LML_LOG_MASTER_GOODBYE_SENDER = 13131,
	FB_LML_LOG_MASTER_GOODBYE_RECEIVER = 13136,
	FB_LML_LOG_SLAVE_GOODBYE_SENDER = 13141,
	FB_LML_LOG_SLAVE_GOODBYE_RECEIVER = 13146,
	FB_LML_LOG_END_COMMUNICATION_SENDER = 13151,
	FB_LML_LOG_END_COMMUNICATION_RECEIVER = 13156,
	/* Clear History */
	FB_LML_CLEAR_HISTORY = 13201
};

enum FeedbackCommand{
	/* Increase step by certain number */
	C_STEP_PLUS_COMMAND_BEGIN = 0,
	C_STEP_PLUS_ONE = 1,
	C_STEP_PLUS_TWO = 2,
	C_STEP_PLUS_THREE = 3,


	/* Jump to certain step */
	C_STEP_JUMP_COMMAND_BEGIN = 100,
	C_STEP_JUMP_TO_ZERO = 100,
	C_STEP_JUMP_TO_ONE = 101,
	C_STEP_JUMP_TO_TWO = 102,
	C_STEP_JUMP_TO_THREE = 103,
	C_STEP_JUMP_TO_FOUR = 104,
	C_STEP_JUMP_TO_FIVE = 105,
	C_STEP_JUMP_TO_SIX = 106,
	C_STEP_JUMP_TO_SEVEN = 107,
	C_STEP_JUMP_TO_EIGHT = 108,
	C_STEP_JUMP_TO_NINE = 109,
	C_STEP_JUMP_TO_TEN  = 110,
	C_STEP_JUMP_TO_ELEVEN = 111,
	C_STEP_JUMP_TO_TWELFE  = 112,
	C_STEP_JUMP_TO_THIRTEEN = 113,
	C_STEP_JUMP_TO_FOURTEEN = 114,
	C_STEP_JUMP_TO_FIFTEEN = 115,

	/* Jump to certain maneuver */
	C_MAN_JUMP_COMMAND_BEGIN = 200,
	C_MAN_JUMP_TO_ZERO = 200,
	C_MAN_JUMP_TO_ONE = 201,
	C_MAN_JUMP_TO_TWO = 202,
	C_MAN_JUMP_TO_THREE = 203,
	C_MAN_JUMP_TO_FOUR = 204,
	C_MAN_JUMP_TO_FIVE = 205,
	C_MAN_JUMP_TO_SIX = 206,
	C_MAN_JUMP_TO_SEVEN = 207,
	C_MAN_JUMP_TO_EIGHT = 208,
	C_MAN_JUMP_TO_NINE = 209,
	C_MAN_JUMP_TO_TEN  = 210,
	C_MAN_JUMP_TO_ELEVEN = 211,
	C_MAN_JUMP_TO_TWELFE  = 212,
	C_MAN_JUMP_TO_THIRTEEN = 213,
	C_MAN_JUMP_TO_FOURTEEN = 214,
	C_MAN_JUMP_TO_FIFTEEN = 215,

	/* Interval for maneuver jump commands ends at 300 */
	C_MAN_JUMP_COMMAND_END = 300
};

enum ActionCommand{

	/* *****************
	 *  LineSpecifier  *
	 * *****************/
	/* Deactivates output, lane detection still running */
	AC_LS_STOP = 1010,
	/* speed output slow */
	AC_LS_SLOW_RIGHTLANE = 1020,
	AC_LS_SLOW_LEFTLANE = 1025,
	/* speed output normal */
	AC_LS_NORMAL_RIGHTLANE = 1030,
	AC_LS_NORMAL_LEFTLANE = 1035,
	/* no speed output */
	AC_LS_NOSPEED = 1040,
	AC_LS_NOSPEED_BACKWARDS = 1050,

	/* parking maneuver commands */
	/* detect type of parking spot the car is currently parked in */
	AC_LS_GET_PARKSPOT_TYPE = 1100,
	/* detect parallel parking spot before parking maneuver */
	AC_LS_DETECT_LONG_PARKING_SPOT_SLOW = 1110,
	AC_LS_DETECT_LONG_PARKING_SPOT_NEXT = 1115,
	/* detect transversal parking spot before parking maneuver */
	AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW = 1120,
	AC_LS_DETECT_TRANS_PARKING_SPOT_NEXT = 1125,

	/* crossing commands */
	AC_LS_CHECK_CROSSING_LINE = 1200,


	/* **************
	 *  FollowPath  *
	 * **************/
	/* Car should drive to certain point, given by xy-coordinates */
	AC_FP_GOTO_XY = 2010, // with stop at xy-coordinate
	AC_FP_GOTO_XY_NOSTOP = 2015,
	/* Car should turn in given direction, activate corresponding turn-trajectory */
	AC_FP_LEFT_TURN = 2020,
	AC_FP_RIGHT_TURN = 2030,
	/* Car should execute given parking maneuver, activate corresponding park-trajectory*/
	AC_FP_PARKING_TRANS = 2040,
	AC_FP_PARKING_OUT_TRANS_RIGHT = 2050,
	AC_FP_PARKING_OUT_TRANS_RIGHT_EXTENDED = 2055,
	AC_FP_PARKING_OUT_TRANS_LEFT = 2060,
	AC_FP_PARKING_OUT_TRANS_LEFT_EXTENDED = 2065,
	AC_FP_PARKING_LONG = 2070,
	AC_FP_PARKING_OUT_LONG = 2080,
	AC_FP_PARKING_OUT_LONG_NORESET = 2085,
	AC_FP_GO_STRAIGHT = 2090,
	AC_FP_GO_STRAIGHT_PARKING_LONG_FORW = 2100,
	AC_FP_GO_STRAIGHT_PARKING_LONG_BACK = 2110,
	AC_FP_GO_STRAIGHT_PARKING_LONG_ALLBACK = 2115, // return to first parking space
	AC_FP_GO_STRAIGHT_PARKING_TRANS = 2120,
	AC_FP_GO_STRAIGHT_PARKING_TRANS_BACK = 2130,
	AC_FP_GO_STRAIGHT_PARKING_TRANS_ALLBACK = 2135, // return to first parking space
	/* Commands to stop/pause Filter and activate again*/
	AC_FP_STOP = 2140,
	AC_FP_CONTINUE = 2145,
	/* Get last completed parking maneuver */
	AC_FP_GET_PARKING_STATUS = 2150,
	AC_FP_RESET_PARKING_STATUS = 2155,
	/*Overtaking*/
	AC_FP_GO_BACKWARDS_BEFORE_OVERTAKING = 2160,
	AC_FP_CHANGE_TO_ONCOMING_LANE = 2170,
	AC_FP_OVERTAKE_CONSTRUCTION_SITE = 2175,
	AC_FP_CHANGE_TO_ORIGINAL_LANE = 2180,
	/* Car should drive certain distance, given by sqrt(x^2+y^2) (received from LS) */
	AC_FP_GOTO_XY_NOSTEERING = 2190,
	/* AC_FP_GOTO_XY_NOSTEERING_BACKWARDS = 2195, */ // Not implemented yet
	AC_FP_GOTO_XY_NOSTEERING_NOSTOP = 2200,
	/* Parking out parallel Extended (go backwards after pull out) */
	AC_FP_PARKING_OUT_LONG_EXTENDED = 2220,
	AC_FP_PARKING_OUT_LONG_NORESET_EXTENDED = 2225,
	/* Correct parking parallel if YAW is too bad */
	AC_FP_PARKING_LONG_CORRECTION = 2240,


	/* ************
	 *  RoadSign  *
	 * ************/
	AC_RS_START = 3100,
	AC_RS_START_PARKING_SIGN_LOGGING = 3105,
	AC_RS_START_PRIORITY_SIGN_LOGGING = 3108,
	/* returns feedback for signs influencing 'priority'/'give way' */
	AC_RS_GET_CROSSING_SIGN = 3110,
	/* command for reseting crossing sign status to 'none' */
	AC_RS_RESET_CROSSING_SIGN = 3120,
	/* command for getting parking sign received status */
	AC_RS_GET_PARKING_SIGN_RECEIVED = 3130,
	/* command for resetting parking sign status to false */
	AC_RS_RESET_PARKING_SIGN = 3140,


	/* ***************
	 *  TimerFilter  *
	 * ***************/
	/* Time to wait in milliseconds*/
	AC_TF_WAIT_ONEH_MSEC = 4001,
	AC_TF_WAIT_TWOH_MSEC = 4002,
	AC_TF_WAIT_THREEH_MSEC = 4003,
	AC_TF_WAIT_FOURH_MSEC = 4004,
	AC_TF_WAIT_FIVEH_MSEC = 4005,
	AC_TF_WAIT_SIXH_MSEC = 4006,
	AC_TF_WAIT_SEVENH_MSEC = 4007,
	AC_TF_WAIT_EIGHTH_MSEC = 4008,
	AC_TF_WAIT_NINEH_MSEC = 4009,

	/* Time to wait in seconds*/
	AC_TF_WAIT_ONE_SEC = 4010,
	AC_TF_WAIT_TWO_SEC = 4020,
	AC_TF_WAIT_THREE_SEC = 4030,
	AC_TF_WAIT_FOUR_SEC = 4040,
	AC_TF_WAIT_FIVE_SEC = 4050,
	AC_TF_WAIT_SIX_SEC = 4060,
	AC_TF_WAIT_SEVEN_SEC = 4070,
	AC_TF_WAIT_EIGHT_SEC = 4080,
	AC_TF_WAIT_NINE_SEC = 4090,
	AC_TF_WAIT_TEN_SEC = 4100,
	AC_TF_WAIT_ELEVEN_SEC = 4110,
	AC_TF_WAIT_TWELVE_SEC = 4120,
	AC_TF_WAIT_THIRTEEN_SEC = 4130,
	AC_TF_WAIT_FOURTEEN_SEC = 4140,
	AC_TF_WAIT_FIFTEEN_SEC = 4150,
	AC_TF_WAIT_SIXTEEN_SEC = 4160,
	AC_TF_WAIT_SEVENTEEN_SEC = 4170,
	AC_TF_WAIT_EIGHTEEN_SEC = 4180,
	AC_TF_WAIT_NINETEEN_SEC = 4190,
	AC_TF_WAIT_TWENTY_SEC = 4200,

	/* NO_RESET mode for timer */
	/* Time to wait in seconds*/
	AC_TF_NORESET_WAIT_ONE_SEC = 4510,
	AC_TF_NORESET_WAIT_TWO_SEC = 4520,
	AC_TF_NORESET_WAIT_THREE_SEC = 4530,
	AC_TF_NORESET_WAIT_FOUR_SEC = 4540,
	AC_TF_NORESET_WAIT_FIVE_SEC = 4550,
	AC_TF_NORESET_WAIT_SIX_SEC = 4560,
	AC_TF_NORESET_WAIT_SEVEN_SEC = 4570,
	AC_TF_NORESET_WAIT_EIGHT_SEC = 4580,
	AC_TF_NORESET_WAIT_NINE_SEC = 4590,
	AC_TF_NORESET_WAIT_TEN_SEC = 4600,
	AC_TF_NORESET_WAIT_ELEVEN_SEC = 4610,
	AC_TF_NORESET_WAIT_TWELVE_SEC = 4620,
	AC_TF_NORESET_WAIT_THIRTEEN_SEC = 4630,
	AC_TF_NORESET_WAIT_FOURTEEN_SEC = 4640,
	AC_TF_NORESET_WAIT_FIFTEEN_SEC = 4650,
	AC_TF_NORESET_WAIT_SIXTEEN_SEC = 4660,
	AC_TF_NORESET_WAIT_SEVENTEEN_SEC = 4670,
	AC_TF_NORESET_WAIT_EIGHTEEN_SEC = 4680,
	AC_TF_NORESET_WAIT_NINETEEN_SEC = 4690,
	AC_TF_NORESET_WAIT_TWENTY_SEC = 4700,

	/* *******************
	 *  UltrasonicCheck  *
	 * *******************/
	AC_UC_CHECK_PARKING_SPACE_TRANS = 5010,
	AC_UC_CHECK_PARKING_SPACE_LONG = 5020,
	AC_UC_CHECK_PULL_OUT_TRANS= 5030,
	AC_UC_CHECK_PULL_OUT_LONG = 5040,
	AC_UC_CHECK_GIVEWAY_LEFT = 5050,
	AC_UC_CHECK_PARKING_TRANS_FRONT = 5060,
	AC_UC_CHECK_OVERTAKING_OBSTACLE = 5070,


	/* *********************
	 *  ObstacleDetetion  *
	 * *********************/
	/* modes for intersection checking */
	AC_OD_INTERSECTION_CHECK_ONCOMING_TRAFFIC = 6010,
	AC_OD_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT = 6020,
	AC_OD_INTERSECTION_CHECK_ALL = 6030,

	/* commands for driving mode */
	AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT = 6110,
	AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF = 6115,
	/* before and during overturn */
	AC_OD_OVERTAKE_CHECK_ONCOMING_TRAFFIC  = 6120,
	AC_OD_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE = 6130,
	/* commands for parking mode, before parking maneuver and before pull-out*/
	AC_OD_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC = 6210,
	AC_OD_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC = 6220,
	/* commands for opatical acc functionality */
	AC_OD_OPTICAL_ACC_ENABLE = 6310, // default value, set at startup
	AC_OD_OPTICAL_ACC_DISABLE = 6330,

	/* ****************
	 *  LightControl  *
	 * ****************/
	/* Light Commands: 7000 - 7999
	 * 7 a b c
	 * a	Head Light 0: no change, 1: disable, 2: enable
	 * b	Reverse Light: 0: no change, 1: disable, 2: enable
	 * c	Turn Lights: 0: no change, 1: disable, 2: left turn, 3: right turn, 4: hazard */

	AC_LC_HEAD_LIGHT_ON = 7200,
	AC_LC_HEAD_LIGHT_OFF = 7100,
	AC_LC_RERVERSE_LIGHT_ON = 7020,
	AC_LC_RERVERSE_LIGHT_OFF = 7010,
	AC_LC_TURN_LEFT = 7002,
	AC_LC_TURN_RIGHT = 7003,
	AC_LC_TURN_DISABLE = 7001,
	AC_LC_HAZARD_LIGHT_ON = 7004,
	AC_LC_HAZARD_LIGHT_OFF = 7001,



	/* **************
	 *  StopAction  *
	 * **************/
	AC_SA_STOP_CAR = 8010,


	/* *****************
	 *  UltrasonicACC  *
	 * *****************/
	AC_UA_DRIVING_MODE = 9010,
	AC_UA_PARKING_MODE = 9020,
	AC_UA_CHECK_NO_MOVEMENT = 9110,
	AC_UA_CHECK_MOVING_AGAIN = 9120,


	/* *********************
	 *  JuryCommunication  *
	 * *********************/
	AC_JURY_ALIVE_AND_READY = 10990,
	/* Maneuver relevant actions */
	AC_JURY_MAN_GET_FIRST = 10100,
	AC_JURY_MAN_FINISHED_GET_NEXT = 10120,
	/* get last maneuver on the maneuver-list, only executable if list was actually completed! */
	AC_JURY_MAN_GET_FINISHING = 10140,
	/* get previous maneuver during normal operational procedure */
	AC_JURY_MAN_GET_PREVIOUS = 10200,
	/* Communication & carState relevant actions */
	//AC_JURY_SET_CARSTATE_STARTUP = 10310,
	//AC_JURY_SET_CARSTATE_READY = 10320,
	//AC_JURY_SET_CARSTATE_RUNNING = 10330,
	//AC_JURY_SET_CARSTATE_COMPLETE = 10340,
	AC_JURY_SET_CARSTATE_ERROR_AND_RESET = 10350,
	/* Waiting for commands sent from jury */
	AC_JURY_WAIT_FOR_COMMAND_START = 10520,
	AC_JURY_WAIT_FOR_COMMAND_STOP = 10530,


	/* *********************
	 *  LightMessageReceiver  *
	 * *********************/
	AC_LMR_RECEIVE_START = 11010, //E
	AC_LMR_RECEIVE_START_CONFIRMATION = 11020, // T
	/* Slave Direction */
	AC_LMR_RECEIVE_REQUEST_SLAVE_DIRECTION = 11030, // W
	AC_LMR_RECEIVE_SLAVE_DIRECTION = 11040, // M, H, A
	/* Master Direction */
	AC_LMR_RECEIVE_MASTER_DIRECTION = 11070, // N, O, I
	/* Conflict */
	AC_LMR_RECEIVE_CONFLICT_SITUATION = 11100, // G
	AC_LMR_RECEIVE_PRIORITY_QUESTION = 11110, // K
	AC_LMR_RECEIVE_PRIORITY_ANSWER = 11120, // D
	/* Farewell */
	AC_LMR_RECEIVE_MASTER_GOODBYE = 11130, // R
	AC_LMR_RECEIVE_SLAVE_GOODBYE = 11140, // U
	AC_LMR_RECEIVE_END_COMMUNICATION = 11150, // S

	/* *********************
	 *  LightMessageTransmitter  *
	 * *********************/
	AC_LMT_TRANSMIT_START = 12010, //E
	AC_LMT_TRANSMIT_START_CONFIRMATION = 12020, // T
	/* Slave Direction */
	AC_LMT_TRANSMIT_REQUEST_SLAVE_DIRECTION = 12030, // W
	AC_LMT_TRANSMIT_SLAVE_TURN_LEFT = 12040, // M
	AC_LMT_TRANSMIT_SLAVE_TURN_RIGHT = 12050, // H
	AC_LMT_TRANSMIT_SLAVE_STRAIGHT = 12060, // A
	/* Master Direction */
	AC_LMT_TRANSMIT_MASTER_TURN_LEFT = 12070, // N
	AC_LMT_TRANSMIT_MASTER_TURN_RIGHT = 12080, // O
	AC_LMT_TRANSMIT_MASTER_STRAIGHT = 12090, // I
	/* Conflict */
	AC_LMT_TRANSMIT_CONFLICT_SITUATION = 12100, // G
	AC_LMT_TRANSMIT_PRIORITY_QUESTION = 12110, // K
	AC_LMT_TRANSMIT_PRIORITY_ANSWER = 12120, // D
	/* Farewell */
	AC_LMT_TRANSMIT_MASTER_GOODBYE = 12130, // R
	AC_LMT_TRANSMIT_SLAVE_GOODBYE = 12140, // U
	AC_LMT_TRANSMIT_END_COMMUNICATION = 12150, // S

	/* *********************
	 *  LightMessageLogger  *
	 * *********************/
	AC_LML_LOG_START_SENDER = 13010,
	AC_LML_LOG_START_RECEIVER = 13015,
	AC_LML_LOG_STARTCONFIRMATION_SENDER = 13020,
	AC_LML_LOG_STARTCONFIRMATION_RECEIVER = 13025,
	/* Slave Direction */
	AC_LML_LOG_REQUEST_SLAVE_DIRECTION_SENDER = 13030,
	AC_LML_LOG_REQUEST_SLAVE_DIRECTION_RECEIVER = 13035,
	AC_LML_LOG_SLAVE_TURN_LEFT_SENDER = 13040,
	AC_LML_LOG_SLAVE_TURN_LEFT_RECEIVER = 13045,
	AC_LML_LOG_SLAVE_TURN_RIGHT_SENDER = 13050,
	AC_LML_LOG_SLAVE_TURN_RIGHT_RECEIVER = 13055,
	AC_LML_LOG_SLAVE_STRAIGHT_SENDER = 13060,
	AC_LML_LOG_SLAVE_STRAIGHT_RECEIVER = 13065,
	/* Master Direction */
	AC_LML_LOG_MASTER_TURN_LEFT_SENDER = 13070,
	AC_LML_LOG_MASTER_TURN_LEFT_RECEIVER = 13075,
	AC_LML_LOG_MASTER_TURN_RIGHT_SENDER = 13080,
	AC_LML_LOG_MASTER_TURN_RIGHT_RECEIVER = 13085,
	AC_LML_LOG_MASTER_STRAIGHT_SENDER = 13090,
	AC_LML_LOG_MASTER_STRAIGHT_RECEIVER = 13095,
	/* Conflict Situation */
	AC_LML_LOG_CONFLICT_SITUATION_SENDER = 13100,
	AC_LML_LOG_CONFLICT_SITUATION_RECEIVER = 13105,
	AC_LML_LOG_PRIORITY_QUESTION_SENDER = 13110,
	AC_LML_LOG_PRIORITY_QUESTION_RECEIVER = 13115,
	AC_LML_LOG_PRIORITY_ANSWER_SENDER = 13120,
	AC_LML_LOG_PRIORITY_ANSWER_RECEIVER = 13125,
	/* Farewell */
	AC_LML_LOG_MASTER_GOODBYE_SENDER = 13130,
	AC_LML_LOG_MASTER_GOODBYE_RECEIVER = 13135,
	AC_LML_LOG_SLAVE_GOODBYE_SENDER = 13140,
	AC_LML_LOG_SLAVE_GOODBYE_RECEIVER = 13145,
	AC_LML_LOG_END_COMMUNICATION_SENDER = 13150,
	AC_LML_LOG_END_COMMUNICATION_RECEIVER = 13155,
	/* Clear History */
	AC_LML_CLEAR_HISTORY = 13200
	
};


#endif // _SCMCOMMUNICATION_ENUMS
