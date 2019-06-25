/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <urbste at googlemail.com>
* For more information see <https://github.com/urbste/MultiCol-SLAM>
*
* MultiCol-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MultiCol-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
*/

/*
* MultiCol-SLAM is based on ORB-SLAM2 which was also released under GPLv3
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Ra鷏 Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "cMultiFrame.h"

#include <opencv2/opencv.hpp>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>

namespace MultiColSLAM
{
	class cCamModelGeneral_;
	class cMultiCamSys_;

	class cMultiInitializer
	{
		typedef std::pair<int, int> Match;

	public:

		// Fix the reference frame 
		//SHINAN: and set mvKeys1,mvKeysRays1,referenceFrame values from reference frame
		cMultiInitializer(const cMultiFrame &ReferenceFrame,
			double sigma = 1.0, int iterations = 200);

		// Computes in parallel a fundamental matrix and a homography
		// Selects a model and tries to recover the motion and the structure from motion
		/* 利用RANSAC 从Refrence Frames和current Frames 中挑选出最匹配的相机，并输出其相关参数
		* currentFrame： 当前帧
		* vMatches12： 在Img2中的匹配点的index
		* R21，t21 由ransac计算出的相机RT， 选取三角重建最靠谱的RT输出
		* vP3D： 从最靠谱的相机中恢复出的三维点坐标
		* vbTriangulated： 
		*/
		bool Initialize(cMultiFrame &currentFrame,
			const std::vector<int> &vMatches12,
			cv::Matx33d &R21, cv::Vec3d &t21,
			std::vector<cv::Vec3d> &vP3D,
			std::vector<bool> &vbTriangulated,
			int& bestCam);

	private:

		bool ReconstructE(
			std::vector<bool> &vbMatchesInliers,
			cv::Matx33d &F21,
			cv::Matx33d &R21,
			cv::Vec3d &t21,
			std::vector<cv::Vec3d> &vP3D,
			std::vector<bool> &vbTriangulated,
			double minParallax, int
			minTriangulated);

		/*SHINAN
		*@param: currentFrame: 当前帧
		*@param: R,t : Rotation and translation computed by RANSAC
		*@param:  vKeys1, vKeys2  : keypoints on image
		*vKeysRays1，vKeysRays2： keypoints in normalized plane
		*vMatches12: 两张图像匹配到的点[index_in_cam1, index_in_cam2]
		*th2： 单个点重投影误差阈值
		*vbGood： Reference Frame中个点的匹配情况，如果三角重建出的三维点满足条件，则其对应的vbgood为true
		*vP3D： Reference Frame中的特征点重建后的三维坐标
		* ret: 好的匹配点
		*/
		int CheckRT(const cMultiFrame &currentFrame,
			const cv::Matx33d &R,
			const cv::Vec3d &t,
			const std::vector<cv::KeyPoint> &vKeys1,
			const std::vector<cv::KeyPoint> &vKeys2,
			const std::vector<cv::Vec3d> &vKeysRays1,
			const std::vector<cv::Vec3d> &vKeysRays2,
			const std::vector<Match> &vMatches12,
			std::vector<cv::Vec3d> &vP3D,
			double th2,
			std::vector<bool> &vbGood,
			double parallax,
			int currCam);

		// Keypoints from Reference Frame (Frame 1)
		std::vector<cv::KeyPoint> mvKeys1;
		std::vector<cv::Vec3d> mvKeysRays1;
		// Keypoints from Current Frame (Frame 2)
		std::vector<cv::KeyPoint> mvKeys2;
		std::vector<cv::Vec3d> mvKeysRays2;

		// Current Matches from Reference to Current
		std::vector<Match> mvMatches12;
		std::vector<bool> mvbMatched1;

		// Calibration
		cMultiCamSys_ camSystem;
		cMultiFrame referenceFrame;

		// Standard Deviation and Variance
		double mSigma, mSigma2;

		// Ransac max iterations
		int mMaxIterations;

		// Ransac sets
		std::vector<std::vector<size_t> > mvSets;

	};
}
#endif // INITIALIZER_H
