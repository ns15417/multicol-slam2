#include <iostream>
#include <fstream> 
#include <iomanip>
#include <thread>
#include <mutex>
#include <misc.h>
#include <opencv2/core/core.hpp>

#include "cTracking.h"
#include "cConverter.h"
#include "cam_model_omni.h"
#include "cSystem.h"
#include "stdlib.h"

using namespace std;

void LoadImagesAndTimestamps(
	const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps);

void util_transfer()
{
	// --------------------------HERE I AM TRYING TO TRANSFORM MY RT to Caylay--------------------------------
	cv::Matx<double, 4, 4> C1(1, 0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1);
	cv::Matx <double, 6, 1> new_C1 = MultiColSLAM::hom2cayley(C1);
	cout << "new_C1" << std::endl << new_C1;

	cv::Matx<double, 4, 4> C2(0.998907, - 0.046551, 0.00422947, - 0.0551771,
		0.0466267, 0.998711, - 0.0200511, - 0.0105182,
		- 0.00329063, 0.0202264, 0.99979, 0.00119695,
	0,0,0,1);

	cv::Matx <double, 6, 1> new_C2 = MultiColSLAM::hom2cayley(C2);
	cout << "new_C2hahahah" << std::endl << new_C2;

	cv::Matx<double, 4, 4> C3(-0.147873, - 0.519001 ,- 0.841885, 0.484182,
		0.0710469, 0.84347, - 0.532457 ,- 0.614021,
		0.986451, - 0.13855, - 0.0878534, - 0.332528,
		0, 0, 0, 1);
	cv::Matx <double, 6, 1> new_C3 = MultiColSLAM::hom2cayley(C3);
	cout << "new_C3hahahah" << std::endl << new_C3;


	// -------------------------------------------END--------------------------------------
}
int main(int argc, char **argv)
{
	util_transfer();
	if (argc != 7)
	{
		cerr << endl << "Usage: ./MultiCol_Slam_Lafida vocabulary_file slam_settings_file path_to_settings path_to_img_sequence cam1id cam2id" << endl;
		return 1;
	}
	
	string path2voc = string(argv[1]);
	string path2settings = string(argv[2]);
	string path2calibrations = string(argv[3]);
	string path2imgs = string(argv[4]);
	int camid_for_cap1 = atoi(argv[5]);
	int camid_for_cap2 = atoi(argv[6]);

	cout << "Get cam id: " << camid_for_cap1 << " and " << camid_for_cap2 << std::endl;
	cout << endl << "MultiCol-SLAM Copyright (C) 2016 Steffen Urban" << endl << endl;
	// --------------
	// 1. Tracking settings
	// --------------
	cv::FileStorage frameSettings(path2settings, cv::FileStorage::READ);

	int traj = (int)frameSettings["traj2Eval"];
	string trajs = to_string(traj);
	const int endFrame = (int)frameSettings["traj.EndFrame"];
	const int startFrame = (int)frameSettings["traj.StartFrame"];

	// --------------
	// 4. Load image paths and timestamps
	// --------------
	cv::VideoCapture cap1(camid_for_cap1);
	cv::VideoCapture cap2(camid_for_cap2);
	//cv::VideoCapture cap3(3);

	cv::Mat src_img1;
	cv::Mat src_img2;
	//cv::Mat src_img3;

	int init_images = 0;
	MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);
	while (init_images < 200)
	{
		init_images++;
	}

	while (cap1.read(src_img1) && cap2.read(src_img2))
	{
		char c = cv::waitKey(1);
		cout << endl << "-------" << endl;
		cout << "Start processing sequence ..." << endl;

		int nrCams = 2;
		std::vector<cv::Mat> imgs;
		cv::Mat gray_img1, gray_img2,gray_img3;
		cv::cvtColor(src_img1, gray_img1, cv::COLOR_BGR2GRAY);
		cv::cvtColor(src_img2, gray_img2, cv::COLOR_BGR2GRAY);
		//cv::cvtColor(src_img3, gray_img3, cv::COLOR_BGR2GRAY);
		imgs.push_back(gray_img1);
		imgs.push_back(gray_img2);
		//imgs.push_back(gray_img3);

		std::vector<double> timestamps;
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// Pass the image to the SLAM system
		double tframe = 2221156;
		std::cout << "bedore TrackMultiColSLAM : imgs size: " << imgs.size() << std::endl;
		MultiSLAM.TrackMultiColSLAM(imgs, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
		
		double T = 0.025;  //ttrack以秒为单位
		if (ttrack < T)
			std::this_thread::sleep_for(std::chrono::milliseconds(
				static_cast<long>((T - ttrack))));

		
		if (c == 'q' || c == 'Q')
		{
			break;
		}
	}
	/*vector<vector<string>> imgFilenames;
	vector<double> timestamps;
	LoadImagesAndTimestamps(startFrame, endFrame, path2imgs, imgFilenames, timestamps);
	
	int nImages = imgFilenames[0].size();

	MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	const int nrCams = static_cast<int>(imgFilenames.size());
	std::vector<cv::Mat> imgs(nrCams);
	for (int ni = 0; ni < nImages; ni++)
	{
		// Read image from file
		std::vector<bool> loaded(nrCams);
		for (int c = 0; c < nrCams; ++c)
		{
			std::cout << "loading image :" << imgFilenames[c][ni] << std::endl;
			imgs[c] = cv::imread(imgFilenames[c][ni], CV_LOAD_IMAGE_GRAYSCALE);
			if (imgs[c].empty())
			{
				cerr << endl << "Failed to load image at: " << imgFilenames[c][ni] << endl;
				return 1;
			}
		}
		double tframe = timestamps[ni];
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// Pass the image to the SLAM system
		MultiSLAM.TrackMultiColSLAM(imgs, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;

		// Wait to load the next frame
		double T = 0;
		if (ni < nImages - 1)
			T = timestamps[ni + 1] - tframe;
		else if (ni > 0)
			T = tframe - timestamps[ni - 1];
		//std::this_thread::sleep_for(std::chrono::milliseconds(30));

		if (ttrack < T)
			std::this_thread::sleep_for(std::chrono::milliseconds(
			static_cast<long>((T - ttrack))));
	}*/

	// Stop all threads
	MultiSLAM.Shutdown();

	// Tracking time statistics
	/*sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	MultiSLAM.SaveMKFTrajectoryLAFIDA("MKFTrajectory.txt");*/

	return 0;
}


void LoadImagesAndTimestamps(const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps)
{
	vstrImageFilenames.resize(2);
	ifstream fTimes;
	string strPathTimeFile = path2imgs + "/images_and_timestamps.txt";

	fTimes.open(strPathTimeFile.c_str());
	string line;


	int cnt = 1;
	while (std::getline(fTimes, line))
	{
		if (cnt >= startFrame && cnt < endFrame) // skip until startframe
		{
			std::istringstream iss(line);
			double timestamp;
			string pathimg1, pathimg2, pathimg3;
			if (!(iss >> timestamp >> pathimg1 >> pathimg2))
				break;
			vTimestamps.push_back(timestamp);
			vstrImageFilenames[0].push_back(path2imgs + '/' + pathimg1);
			vstrImageFilenames[1].push_back(path2imgs + '/' + pathimg2);
		}
		++cnt;

	}
}