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
 * $Author:: schoen $   $Date:: 2015-12-03 #$
 **********************************************************************/

#include "stdafx.h"
#include "LineFilter.h"
#include "ImageProcessingUtils.h"
#include "CVMath.h"

#include <iostream>

using namespace cv;

const float LineFilter::SIZE_X = 1.5;
const float LineFilter::SIZE_Y = 1.5;

LineFilter::LineFilter() {
	//Some default parameters, usually they are overwritten by Init
	this->near = 0.7;
	this->far = 2.2;

	this->fov_h = 58;
	this->fov_v = 45;

	this->pitch = 0;
	this->camera_x_offset = -0.015;
	this->camera_z_offset = 0.22;

	this->camera_width = 640;
	this->camera_height = 480;

	enableMedianFiltering = false;
	is_initialized = false;

	meanBrightness = 0;

	CalculatePerspectiveTransformationMatrix();
}

LineFilter::~LineFilter() {
}

void LineFilter::Init(float near, float fovHorizontal, float fovVertical, float pitch, float cameraPositionY,
		float cameraPositionZ, float adaptiveThresholdConstant, int cameraWidth, int cameraHeight) {
	this->near = near;
	this->far = near + SIZE_Y;

	this->fov_h = fovHorizontal;
	this->fov_v = fovVertical;

	this->pitch = pitch;
	this->camera_x_offset = - cameraPositionY;
	this->camera_z_offset = cameraPositionZ;

	this->camera_width = cameraWidth;
	this->camera_height = cameraHeight;

	this->adaptive_threshold_constant = adaptiveThresholdConstant;

	meanBrightness = 0;

	maskWorkRegion = 0;

	gray = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
	validPixels = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
	warp = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC3, Scalar(0, 0, 0));
	affineMask = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));

	is_initialized = true;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::EnableMedianFiltering(bool enable) {
	this->enableMedianFiltering = enable;
}

void LineFilter::SetNearFar(float near) {
	this->near = near;
	this->far = near + SIZE_Y;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetFOV(float fovHorizontal, float fovVertical) {
	this->fov_h = fovHorizontal;
	this->fov_v = fovVertical;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetPitch(float pitch) {
	this->pitch = pitch;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetCameraPosition(float cameraPositionX, float cameraPositionZ) {
	this->camera_x_offset = cameraPositionX;
	this->camera_z_offset = cameraPositionZ;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetCameraSize(int cameraWidth, int cameraHeight) {
	this->camera_width = cameraWidth;
	this->camera_height = cameraHeight;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetAdaptiveThresholdConstant(float adaptiveThresholdConstant) {
	this->adaptive_threshold_constant = adaptiveThresholdConstant;
}

void LineFilter::SetValidPixels(cv::Mat &mask, Size s, bool enable) {
	if(enable) {
		int cropWidth = 4;
		int maskWidth = mask.cols-cropWidth;
		int outHeight = (s.height >> 1) - maskWorkRegion;
		//int outWidth = 0.95 * maskWidth;//measured 0.95
		int outWidth = (s.width >> 1);

		mask(Rect(0, maskWorkRegion, mask.cols, mask.rows - maskWorkRegion)).copyTo(validPixels);

		if(!lastMask.empty()) {
			bitwise_or(validPixels, lastMask, buffer);
			if(!lastlastMask.empty()) {
				bitwise_or(buffer, lastlastMask, buffer);
			}

			//imwrite("/tmp/buffer.jpg", buffer);
			resize(buffer(Rect(0, 0, maskWidth, buffer.rows)), buffer, Size(outWidth, outHeight));

			lastMask.copyTo(lastlastMask);
		} else {
			resize(validPixels(Rect(0, 0, maskWidth, validPixels.rows)), buffer, Size(outWidth, outHeight));
		}

		validPixels.copyTo(lastMask);

		if(outWidth != (s.width >> 1)) {
			copyMakeBorder(buffer, buffer, 0, 0, 0, (s.width >> 1) - outWidth, BORDER_REPLICATE);
		}

		if(1) {
			//Mat element = getStructuringElement(MORPH_RECT, Size( 12, 2));
			//cv::erode(buffer, validPixels, element, Point(-1,-1), 1);

			blur(buffer, validPixels, Size(15, 15));
			//imwrite("/tmp/maske.jpg", validPixels);
		} else {
			buffer.copyTo(validPixels);
		}

		if(!lastValidPixels.empty()){
			multiply(validPixels, lastValidPixels, validPixels, 1.0/255);
		}

		if(1) {
			Mat element = getStructuringElement( MORPH_RECT, Size( 4, 3));
			cv::dilate(validPixels, lastValidPixels, element, Point(-1,-1), 1);
		}

		copyMakeBorder(validPixels, validPixels, maskWorkRegion, 0, 0, 0, BORDER_CONSTANT);
		resize(validPixels, validPixels, s);

		//imwrite("/tmp/maske1.jpg", validPixels);
	} else {
		validPixels = Mat::ones(s, CV_8UC1) * 255;
	}
}

tResult LineFilter::GetTopDownView(Mat &in, Mat& out) {
	if(!is_initialized) {
		RETURN_AND_LOG_ERROR(0);
	}
	//imwrite("/tmp/in.jpg", in);
	cvtColor(in,in,COLOR_RGB2GRAY);

	multiply(in, validPixels, buffer, 1.0/255);

	if(1) {
		warpPerspective(buffer, warp, topDownMatrix, warp.size(), INTER_LINEAR, BORDER_REPLICATE);

		adaptiveThreshold(warp, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 21, adaptive_threshold_constant);
		gray.copyTo(out, affineMask);
	} else {
		warpPerspective(validPixels, warp, topDownMatrix, warp.size(), INTER_LINEAR, BORDER_REPLICATE);
		warp.copyTo(out, affineMask);
	}

	if(enableMedianFiltering) {
		medianBlur(out, out, 3);
	}

	RETURN_NOERROR;
}

Point2f LineFilter::GetTopLeftWorldCoordinates() {
	return Point2f(far, SIZE_X * 0.5);
}

void LineFilter::CalculatePerspectiveTransformationMatrix() {
	Point2f sourcePoints[4] = { Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
	CalculateSourcePoints(sourcePoints);

	maskWorkRegion = (sourcePoints[0].y - 10) / 2;
	if(maskWorkRegion < 0) {
		maskWorkRegion = 0;
	}

	Point2f destPoints[4] = { Point2f(100, 0), Point2f(200, 0), Point2f(200, 300), Point2f(100, 300) };
	topDownMatrix = getPerspectiveTransform(sourcePoints, destPoints);

	try {
		Mat white = Mat(480, 640, CV_8UC1, Scalar(255));
		warpPerspective(white, affineMask, topDownMatrix, affineMask.size(), INTER_LINEAR, BORDER_CONSTANT, 0);

	} catch (cv::Exception &e) {
		std::cout << e.msg << std::endl;
	}
}

void LineFilter::CalculateSourcePoints(cv::Point2f* buffer) {
	if(!is_initialized) {
		return;
	}

	float vertical = tan(fov_v / 180 * CV_PI / 2);
	float horizontal = tan(fov_h / 180 * CV_PI / 2);

	Point3f vectorFarLeft(-0.25 - camera_x_offset, camera_z_offset, far);
	Point3f vectorFarRight(0.25 - camera_x_offset, camera_z_offset, far);
	Point3f vectorNearRight(0.25 - camera_x_offset, camera_z_offset, near);
	Point3f vectorNearLeft(-0.25 - camera_x_offset, camera_z_offset, near);

	std::vector<Point2f> vec;
	vec.push_back(Point2f(vectorNearRight.y, vectorNearRight.z));
	vec.push_back(Point2f(vectorFarRight.y, vectorFarRight.z));

	//clockwise rotation with pitch angle
	Mat r = getRotationMatrix2D(Point2f(0, 0), pitch, 1.0);
	transform(vec, vec, r);

	Point2f vectorNear = vec.at(0);
	Point2f vectorFar = vec.at(1);
	float nearY = (camera_height / 2) + vectorNear.x / (vectorNear.y * vertical) * (camera_height / 2);
	float farY = (camera_height / 2) + vectorFar.x / (vectorFar.y * vertical) * (camera_height / 2);

	float farLeftX = (camera_width / 2) + vectorFarLeft.x / (vectorFarLeft.z * horizontal) * (camera_width / 2);
	float farRightX = (camera_width / 2) + vectorFarRight.x / (vectorFarRight.z * horizontal) * (camera_width / 2);
	float nearRightX = (camera_width / 2) + vectorNearRight.x / (vectorNearRight.z * horizontal) * (camera_width / 2);
	float nearLeftX = (camera_width / 2) + vectorNearLeft.x / (vectorNearLeft.z * horizontal) * (camera_width / 2);

	buffer[0] = Point2f(farLeftX, farY);
	buffer[1] = Point2f(farRightX, farY);
	buffer[2] = Point2f(nearRightX, nearY);
	buffer[3] = Point2f(nearLeftX, nearY);
}
