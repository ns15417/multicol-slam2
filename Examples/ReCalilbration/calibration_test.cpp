#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "misc.h"

int CalibrationResultTest();
int ResultTransform();
int SaveCameraIntrinsic(const Eigen::VectorXd new_distort_coeff,
	const Eigen::VectorXd new_inverse_polynomial,
	const Eigen::Matrix3d camera_matrix,
	const int Iw, const int Ih, const std::string &filename);
void util_transfer();
int WriteCameraIntrinsic(const Eigen::VectorXd new_distort_coeff,
	const Eigen::VectorXd new_inverse_polynomial,
	const Eigen::Matrix3d camera_matrix,
	const int Iw, const int Ih);

int main(int argc,char** argv)
{
	std::cout << "argc: " << argc << std::endl;
	if (argc == 1)
	{
		std::cout << "Attempting to execute Calibration result test!!" << std::endl;
		CalibrationResultTest();
		util_transfer();
		ResultTransform();
	}
	else if (argc != 3)
	{
		std::cerr << "Usage: calibration_test.exe path_of_mcptam_file path_of_MultiCol_Slam_file " << std::endl;
	}
	//std::string mcptam_path = std::string(argv[1]);
	//std::string multicol_path = std::string(argv[2]);
	//ResultTransform(mcptam_path, multicol_path);
}


/*
*  @brief:This function is used to test Rotation Matrix in pose.data
*  RotationMatrix: Rotation Matrix in Fil pose.data
*/
int CalibrationResultTest()
{
	Eigen::Matrix3d RotationMatrix1, RotationMatrix2;
	RotationMatrix1 << -0.606798, 0.200102, 0.769256,
		              - 0.155266 ,0.91931 ,- 0.36161, 
		              - 0.779544, - 0.338864, - 0.526766;

	RotationMatrix2 << -0.160924, - 0.210418, - 0.964276, 
		0.20669, 0.94816, - 0.241395, 
		0.965082, - 0.238153, - 0.109091;

	Eigen::Vector3d euler_angles1 = RotationMatrix1.eulerAngles(2, 1, 0);
	Eigen::Vector3d euler_angles2 = RotationMatrix2.eulerAngles(2, 1, 0);

	std::cout << "for RotationMatrix1 : " << std::endl <<" yaw(Z) pitch(Y) roll(X) = \n" << euler_angles1.transpose() << std::endl;
	std::cout << "actural rotation around Z : " << euler_angles1[0] / M_PI * 180 << std::endl
		      << "actural rotation around Y : " << euler_angles1[1] / M_PI * 180 << std::endl
		      << "actural rotation around X : " << euler_angles1[2] / M_PI * 180 << std::endl;
	
    std::cout << "for RotationMatrix2 : " << std::endl << "yaw(Z) pitch(Y) roll(X)=\n" << euler_angles2.transpose() << std::endl;
	std::cout << "actural rotation around Z : " << euler_angles2[0] / M_PI * 180 << std::endl
		<< "actural rotation around Y : " << euler_angles2[1] / M_PI * 180 << std::endl
		<< "actural rotation around X : " << euler_angles2[2] / M_PI * 180 << std::endl;
	
	return 0;
}

void util_transfer()
{
	// --------------------------HERE I AM TRYING TO TRANSFORM MY RT to Caylay--------------------------------
	cv::Matx<double, 4, 4> C1(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	cv::Matx <double, 6, 1> new_C1 = MultiColSLAM::hom2cayley(C1);
	std::cout << "new_C1" << std::endl << new_C1 << std::endl;

	cv::Matx<double, 4, 4> C2(-0.556082, 0.0123927, 0.831035, 0.0619053,
		0.0116095, 0.999907, - 0.00714255, 0.00242012,
		- 0.831046, 0.00567604, - 0.556174, - 0.107676,
		0, 0, 0, 1);

	cv::Matx <double, 6, 1> new_C2 = MultiColSLAM::hom2cayley(C2);
	std::cout << "new_C2hahahah" << std::endl << new_C2 << std::endl;

	cv::Matx<double, 4, 4> C3(-0.503322, - 0.00728889, - 0.864068, - 0.0670418,
		- 0.0234716, 0.999711, 0.00523913, 0.00348552,
		0.86378 ,0.022918, - 0.503347, - 0.11071,
		0, 0, 0, 1);
	cv::Matx <double, 6, 1> new_C3 = MultiColSLAM::hom2cayley(C3);
	std::cout << "new_C3hahahah" << std::endl << new_C3 << std::endl;


}


int ReadFromFile(Eigen::Vector4d &distort_coeff, Eigen::Matrix3d &camera_matrix, std::string mcptamfilepath)
{
	cv::FileStorage InstrinsicReading(mcptamfilepath, cv::FileStorage::READ);
	if (!InstrinsicReading.isOpened())
	{
		std::cout << "Failed to open " << mcptamfilepath << std::endl;
		return 0;
	}

	std::string tmp_distort = InstrinsicReading["distortion_coefficients"]["data"];
	std::cout << "distort_coeff: " << tmp_distort << std::endl;
	distort_coeff[0] = (double)tmp_distort[0];
	distort_coeff[1] = (double)tmp_distort[1];
	distort_coeff[2] = (double)tmp_distort[2];
	distort_coeff[3] = (double)tmp_distort[3];


	std::string tmp_instrinsic = InstrinsicReading["camera_matrix"]["data"];
	camera_matrix(0, 0) = (double)tmp_instrinsic[0];
	camera_matrix(0, 1) = (double)tmp_instrinsic[1];
	camera_matrix(0, 2) = (double)tmp_instrinsic[2];
	camera_matrix(1, 0) = (double)tmp_instrinsic[3];
	camera_matrix(1, 1) = (double)tmp_instrinsic[4];
	camera_matrix(1, 2) = (double)tmp_instrinsic[5];
	camera_matrix(2, 0) = (double)tmp_instrinsic[6];
	camera_matrix(2, 1) = (double)tmp_instrinsic[7];
	camera_matrix(2, 2) = (double)tmp_instrinsic[8];
}
/*
*  @brief:This function is to transform the intrinsic Matrix got from MCPTAM 
           to format asked by MultiCol-SLAM
*  RotationMatrix: Rotation Matrix in Fil pose.data
*/
int ResultTransform()
{
	std::cout << "Transforming" << std::endl;
	int img_width, img_height;
	img_width = 640;
	img_height = 480;

	// 需要根据每个相机进行修改
	Eigen::Vector4d distort_coeff;
	Eigen::Matrix3d camera_matrix;
	distort_coeff << 319.543314011052, -0.0015144390163834, 1.64012124087578e-06, -6.29922060212327e-09;
	camera_matrix << 0.997103876401539, 0.000824430307240755, 671.277625446551, -0.000650217723157717, 1, 350.761338052098, 0, 0, 0;
	//记得修改维数11
	cv::Matx<double, 1, 11> inverse_polynomial;
	cv::Matx<double, 1, 11> new_inverse_polynomial;
	inverse_polynomial<<274.09, - 126.007 ,- 6.80859 ,- 27.5604 ,3.31105 ,- 4.754, 6.00577, 2.66313, - 0.47732,- 2.54251, 0.934194,0.0;

	std::cout << "Camera.Iw: " << img_width << std::endl;
	std::cout << "Camera.Ih: " << img_height << std::endl;

	/*Eigen::VectorXd new_distort_coeff(5);
	new_distort_coeff(0, 0) = -distort_coeff[0];
	new_distort_coeff(0, 1) = (double)0.0;
	new_distort_coeff(0, 2) = -distort_coeff[1];
	new_distort_coeff(0, 3) = -distort_coeff[2];
	new_distort_coeff(0, 4) = -distort_coeff[3];*/
	cv::Matx<double,1,5 > new_distort_coeff;
	new_distort_coeff(0, 0) = -distort_coeff[0];
	new_distort_coeff(0, 1) = (double)0.0;
	new_distort_coeff(0, 2) = -distort_coeff[1];
	new_distort_coeff(0, 3) = -distort_coeff[2];
	new_distort_coeff(0, 4) = -distort_coeff[3];


	for (int i = 0; i < inverse_polynomial.cols; i++)
	{
		if(i%2 == 1)
			new_inverse_polynomial(0,i) = -inverse_polynomial(i);
		else
			new_inverse_polynomial(0,i) = inverse_polynomial(i);
	}
	

	std::cout << "Camera.nrpol: " << int(5) << std::endl;
	std::cout << "Camera.nrinvpol: " << int(new_inverse_polynomial.cols) << std::endl;

	std::cout << "Camera.a0: " << new_distort_coeff(0, 0) << std::endl;
	std::cout << "Camera.a1: " << new_distort_coeff(0, 1) << std::endl;
	std::cout << "Camera.a2: " << new_distort_coeff(0, 2) << std::endl;
	std::cout << "Camera.a3: " << new_distort_coeff(0, 3) << std::endl;
	std::cout << "Camera.a4: " << new_distort_coeff(0, 4) << std::endl;

	for (int i = 0; i < new_inverse_polynomial.cols; i++)
	{
		std::string tmpstr = "Camera.pol" + std::to_string(i)+": ";
		std::cout << tmpstr << new_inverse_polynomial(0,i) << std::endl;
	}

	std::cout << "Camera.c: " << camera_matrix(0, 0) << std::endl;
	std::cout << "Camera.d: " << camera_matrix(0, 1) << std::endl;
	std::cout << "Camera.e: " << camera_matrix(1, 0) << std::endl;

	std::cout << "Camera.u0: " << camera_matrix(0, 2) << std::endl;
	std::cout << "Camera.v0: " << camera_matrix(1, 2) << std::endl;
	//WriteCameraIntrinsic(new_distort_coeff,new_inverse_polynomial,camera_matrix,img_width,img_height);
	return 0;
}

int WriteCameraIntrinsic(const Eigen::VectorXd new_distort_coeff,
	const Eigen::VectorXd new_inverse_polynomial,
	const Eigen::Matrix3d camera_matrix,
	const int Iw, const int Ih)
{
	
	std::cout << "Camera.Iw:" << Iw << std::endl;
	std::cout << "Camera.Ih" << Ih << std::endl;

	std::cout << "Camera.nrpol" << int(5) << std::endl;
	std::cout << "Camera.nrinvpol" << int(new_inverse_polynomial.cols()) << std::endl;

	std::cout << "Camera.a0" << new_distort_coeff[0] << std::endl;
	std::cout << "Camera.a1" << new_distort_coeff[1] << std::endl;
	std::cout << "Camera.a2" << new_distort_coeff[2] << std::endl;
	std::cout << "Camera.a3" << new_distort_coeff[3] << std::endl;
	std::cout << "Camera.a4" << new_distort_coeff[4] << std::endl;

	for (int i = 0; i < new_inverse_polynomial.cols(); i++)
	{
		std::string tmpstr = "Camera.pol" + std::to_string(i);
		std::cout << tmpstr << new_inverse_polynomial[i] << std::endl;
	}

	std::cout << "Camera.c" << camera_matrix(0, 0) << std::endl;
	std::cout << "Camera.d" << camera_matrix(0, 1) << std::endl;
	std::cout << "Camera.e" << camera_matrix(1, 0) << std::endl;

	std::cout << "Camera.u0:" << camera_matrix(0, 2) << std::endl;
	std::cout << "Camera.v0:" << camera_matrix(1, 2) << std::endl;

	return 0;
}


int SaveCameraIntrinsic(const Eigen::VectorXd new_distort_coeff,
	const Eigen::VectorXd new_inverse_polynomial,
	const Eigen::Matrix3d camera_matrix,
	const int Iw, const int Ih, const std::string &filename)
{
	cv::FileStorage new_cam_intrinsic(filename,cv::FileStorage::WRITE);
	new_cam_intrinsic << "Camera.Iw" << Iw;
	new_cam_intrinsic << "Camera.Ih" << Ih;
	
	new_cam_intrinsic << "Camera.nrpol" << int(5);
	new_cam_intrinsic << "Camera.nrinvpol" << int(new_inverse_polynomial.cols());

	new_cam_intrinsic << "Camera.a0" << new_distort_coeff[0];
	new_cam_intrinsic << "Camera.a1" << new_distort_coeff[1];
	new_cam_intrinsic << "Camera.a2" << new_distort_coeff[2];
	new_cam_intrinsic << "Camera.a3" << new_distort_coeff[3];
	new_cam_intrinsic << "Camera.a4" << new_distort_coeff[4];

	for (int i = 0; i < new_inverse_polynomial.cols(); i++)
	{
		std::string tmpstr = "Camera.pol" + std::to_string(i);
		new_cam_intrinsic << tmpstr << new_inverse_polynomial[i];
	}

	new_cam_intrinsic << "Camera.c" << camera_matrix(0, 0);
	new_cam_intrinsic << "Camera.d" << camera_matrix(0, 1);
	new_cam_intrinsic << "Camera.e" << camera_matrix(1, 0);

	new_cam_intrinsic << "Camera.u0" << camera_matrix(0, 2);
	new_cam_intrinsic << "Camera.v0" << camera_matrix(1, 2);

	return 0;
}