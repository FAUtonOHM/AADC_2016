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

#ifndef LINEFILTER_H_
#define LINEFILTER_H_

/**
 *
 */
class LineFilter {
protected:
	static const float SIZE_X;
	static const float SIZE_Y;

	float near;
	float far;

	float fov_h;
	float fov_v;

	int camera_width;
	int camera_height;

	float pitch;

	float camera_x_offset;
	float camera_z_offset;

	cv::Mat topDownMatrix;

	/**
	  * indicates if we use a median filter on the binary image
	  */
	bool enableMedianFiltering;
	float adaptive_threshold_constant;

	bool is_initialized;

	cv::Scalar meanBrightness;

	/**
	 * some temporaries
	 */
	Mat buffer;
	Mat gray;
	Mat validPixels;
	Mat lastValidPixels;

	int maskWorkRegion;
	Mat lastMask;
	Mat lastlastMask;
	Mat warp;
	Mat affineMask;

public:


	LineFilter();
	virtual ~LineFilter();

	/**
	 *
	 * @param near
	 * @param fovHorizontal
	 * @param fovVertical
	 * @param pitch
	 * @param cameraPositionX
	 * @param cameraPositionY
	 * @param cameraWidth
	 * @param cameraHeight
	 */
	void Init(float near, float fovHorizontal, float fovVertical, float pitch, float cameraPositionY,
			float cameraPositionZ, float adaptiveThresholdConstant, int cameraWidth, int cameraHeight);

	void EnableMedianFiltering(bool enable);
	void SetNearFar(float near);
	void SetFOV(float fovHorizontal, float fovVertical);
	void SetPitch(float pitch);
	float GetPitch() {
		return pitch;
	}
	void SetCameraPosition(float cameraPositionX, float cameraPositionY);
	void SetCameraSize(int cameraWidth, int cameraHeight);
	void SetAdaptiveThresholdConstant(float adaptiveThresholdConstant);

	void SetValidPixels(cv::Mat &mask, Size s, bool enable);

	virtual tResult GetTopDownView(cv::Mat &in, cv::Mat& out);

	cv::Point2f GetTopLeftWorldCoordinates();

private:
	void CalculatePerspectiveTransformationMatrix();
	void CalculateSourcePoints(cv::Point2f * buffer);
};

#endif /* LINEFILTER_H_ */
