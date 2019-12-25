
#include "stdafx.h"
#include <time.h>
#include <Windows.h>  // GetTickCount's head file

//================= 0.caribrate camera ==================//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

#include <filesystem>
using namespace std::tr2::sys;

#include <chrono> 
using namespace std;
using namespace chrono; 

using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using namespace std;
using namespace cv;

int n_boards = 20;
float image_sf = 1.f;
float delay = 200.f;
int board_w = 6;
int board_h = 4;

Size ResImgSiz = Size(1296, 972);


int main() {
	vector<Mat> images;
	Mat im;
	directory_iterator it("chessborad-images\\");

	while (it != directory_iterator()) {
		String tmp = it->path().string();
		im = imread(tmp, 1);
		images.push_back(im);
		++it;
	}

	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);
												   // ALLOCATE STORAGE
												   //
	vector<vector<cv::Point2f> > image_points;
	vector<vector<cv::Point3f> > object_points;

											   // Capture corner views: loop until we've got n_boards successful
											   // captures (all corners on the board are found).
											   //
	double last_captured_timestamp = 0;
	cv::Size image_size;//图像尺寸
	while (image_points.size() < (size_t)n_boards) {
		cv::Mat image0, image;

		//	capture >> image0; // for video calibration

		image0 = images[image_points.size()];

		image_size = image0.size();
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);


																					// Find the board
																					//
		vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_sz, corners);

																		 // Draw it
																		 //
		drawChessboardCorners(image, board_sz, corners, found);//found


															   // If we got a good board, add it to our data
															   //
		double timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;

		if (found && timestamp - last_captured_timestamp > 1) {
			last_captured_timestamp = timestamp;
			image ^= cv::Scalar::all(255);
			cv::Mat mcorners(corners);


			// do not copy the data
			mcorners *= (1.0 / image_sf);


										 // scale the corner coordinates
			image_points.push_back(corners);
			object_points.push_back(vector<cv::Point3f>());
			vector<cv::Point3f> &opts = object_points.back();


			opts.resize(board_n);
			for (int j = 0; j < board_n; j++) {
				opts[j] = cv::Point3f(static_cast<float>(j / board_w),
					static_cast<float>(j % board_w), 0.0f);
			}
			cout << "Collected our " << static_cast<uint>(image_points.size())
				<< " of " << n_boards << " needed chessboard images\n" << endl;
		}

		resize(image, image, ResImgSiz);
		cv::imshow("Calibration", image);


		// show in color if we did collect the image
		if ((cv::waitKey(30) & 255) == 27)
			return -1;
	}


	// END COLLECTION WHILE LOOP.
	cv::destroyWindow("Calibration");
	cout << "\n\n*** CALIBRATING THE CAMERA...\n" << endl;


	// CALIBRATE THE CAMERA!
	cv::Mat intrinsic_matrix, distortion_coeffs;
	double err = cv::calibrateCamera(
		object_points, image_points, image_size, intrinsic_matrix,
		distortion_coeffs, cv::noArray(), cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);


	// SAVE THE INTRINSICS AND DISTORTIONS
	cout << " *** DONE!\n\nReprojection error is " << err
		<< "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("chessboard.yaml", cv::FileStorage::WRITE);
	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
		<< distortion_coeffs;
	fs.release();


	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	fs.open("chessboard.yaml", cv::FileStorage::READ);
	cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
	cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
	cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;


	// Build the undistort map which we will use for all
	// subsequent frames.
	// 
	cv::Mat map1, map2;
	auto start = system_clock::now();


	cv::initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded,
		cv::Mat(), intrinsic_matrix_loaded, image_size,
		CV_16SC2, map1, map2);


	// Just run the camera to the screen, now showing the raw and
	// the undistorted image.

	for (int i = 0; i < n_boards; ++i) {
		cv::Mat image_calib, image0;
		image0 = images[i];


		//if (image0.empty()) {
		//	break;
		//}
		cv::remap(image0, image_calib, map1, map2, cv::INTER_LINEAR,
			cv::BORDER_CONSTANT, cv::Scalar());

		//resize(image_calib, image_calib, ResImgSiz);
		//cv::imshow("Undistorted", image_calib);
		//if ((cv::waitKey(delay) & 255) == 27) {
		//	break;
		//}
	}
	auto end = system_clock::now();

	auto duration = duration_cast<microseconds>(end - start);
	printf("(CPU)  cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);

	waitKey(0);
	return 0;
}


//================= 1.image reading ==================//

//#include <iostream>
//#include <string>
//#include <opencv2/opencv.hpp>
//
//#include <chrono> 
//#include <tbb/tbb.h>
//
//using namespace std;
//using namespace chrono; // compute the running time
//
//using namespace cv;
//using namespace tbb;
//
//Mat camera_matrix = Mat::zeros(3, 3, CV_64FC1);
//Mat distortion_coefficients = Mat::zeros(1, 5, CV_64FC1);
//
//vector<Mat> src_images(15);
//vector<Mat> dst_images(15);
//
//
//spin_mutex mutexObj; 
//
//string imagePath;
//string imageFilePath = "data file irectory\\";
//
//void ReadImage(int a)
//{
//	//read image from 0.jpg to 14.jpg
//
//	Mat tmp2;
//	imagePath = imageFilePath + std::to_string(a) + ".jpg";
//	tmp2 = imread(imagePath);
//	mutexObj.lock(); 
//	src_images[a] = tmp2;
//	mutexObj.unlock(); 
//	cout << a << endl;
//}
//
//void undistortOperate(int a)
//{
//	Mat tmp3;
//	undistort(src_images[a], tmp3, camera_matrix, distortion_coefficients);
//	mutexObj.lock(); 
//	dst_images[a] = tmp3;
//	mutexObj.unlock(); 
//}
//
//void ReadImage_no_tbb(int a)
//{
//	Mat tmp2;
//	imagePath = imageFilePath + std::to_string(a) + ".jpg";
//	Mat image = imread(imagePath);
//	src_images[a] = image;
//}
//
//void undistortOperate_no_tbb(int a)
//{
//	Mat tmp;
//	undistort(src_images[a], tmp, camera_matrix, distortion_coefficients);
//	dst_images[a] = tmp;
//}
//
//
//
//int main()
//{
//	camera_matrix.at<double>(0, 0) = 3.0126077468510184e+03;
//	camera_matrix.at<double>(0, 2) = 1296;
//	camera_matrix.at<double>(1, 1) = 3.0094117205252010e+03;
//	camera_matrix.at<double>(1, 2) = 972;
//	camera_matrix.at<double>(2, 2) = 1;
//	//std::cout << camera_matrix << std::endl;
//
//	distortion_coefficients.at<double>(0, 0) = -1.2974312863621433;
//	distortion_coefficients.at<double>(0, 1) = 2.11178864438862;
//	distortion_coefficients.at<double>(0, 2) = 0;
//	distortion_coefficients.at<double>(0, 3) = 0;
//	distortion_coefficients.at<double>(0, 4) = -2.2622338952366023;
//
//	auto start = system_clock::now();
//
//	parallel_for(0, 15, 1, ReadImage); 
////	parallel_for(0, 10, 1, undistortOperate); 
//
//	auto end = system_clock::now();
//	auto duration = duration_cast<microseconds>(end - start);
//	printf("tbb undistort cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//	imshow("undistort1", dst_images[3]);
//
//	start = system_clock::now();
//
//	for (int i = 0; i < 15; i++)
//	{
//		ReadImage_no_tbb(i);
////		undistortOperate_no_tbb(i);
//	}
//
//	end = system_clock::now();
//	duration = duration_cast<microseconds>(end - start);
//	printf("no tbb undistort cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//	imshow("undistort2", dst_images[3]);
//
//	waitKey(0);
//
//	return 0;
//}


//===================== 2. CPU undistort 15 chessbord images===================//
//#include <iostream>
//#include <string>
//#include <opencv2/opencv.hpp>
//#include <chrono> 
//
//using namespace std;
//// 必要，放在前
//using namespace chrono; 
//using namespace cv::cuda;
//
//using namespace cv;
//
//Mat camera_matrix = Mat::zeros(3, 3, CV_64FC1);
//Mat distortion_coefficients = Mat::zeros(1, 5, CV_64FC1);
//
//vector <Mat> h_src_images(15);
//vector <Mat> h_dst_images(15);
//vector <Mat> src_images(15);
//vector <Mat> undistort_images(15);
//Mat tmp;
//
//void ReadImage(int a)
//{
//	stringstream stringstr;
//	stringstr << a;
//	string imagePath;
//	stringstr >> imagePath;
//	imagePath += ".jpg";
//	tmp = imread(imagePath);
//	h_src_images[a].push_back(tmp);
//}
//
//
//int main()
//{
//	
//	camera_matrix.at<double>(0, 0) = 3.0126077468510184e+03;
//	camera_matrix.at<double>(0, 2) = 1296;
//	camera_matrix.at<double>(1, 1) = 3.0094117205252010e+03;
//	camera_matrix.at<double>(1, 2) = 972;
//	camera_matrix.at<double>(2, 2) = 1;
//	//std::cout << camera_matrix << std::endl;
//
//	distortion_coefficients.at<double>(0, 0) = -1.2974312863621433;
//	distortion_coefficients.at<double>(0, 1) = 2.11178864438862;
//	distortion_coefficients.at<double>(0, 2) = 0;
//	distortion_coefficients.at<double>(0, 3) = 0;
//	distortion_coefficients.at<double>(0, 4) = -2.2622338952366023;
//	
//	auto start = system_clock::now(); 
//
//	for (int i = 0; i < 15; i++)
//		ReadImage(i);
//	auto end = system_clock::now();
//	auto duration = duration_cast<microseconds>(end - start);
//	printf("CPU read 15 iamges cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//
//	
//	start = system_clock::now();
//	cv::Size imageSize(2592,1944);
//	//矫正
//	for (int i = 0; i<15; i++)
//		undistort(h_src_images[i], h_dst_images[i], camera_matrix, distortion_coefficients); 
//	end = system_clock::now();
//
//	//start = system_clock::now();
//	
//	//cv::Mat map1, map2;
//	//initUndistortRectifyMap(
//	//	camera_matrix, distortion_coefficients, Mat(),
//	//	camera_matrix, imageSize,
//	//	CV_32FC1, map1, map2);
//	//
//	//for(int i=0;i<10;i++)
//	//	cv::remap(h_src_images[i], h_dst_images[i], map1, map2, INTER_LINEAR); 
//
//	//end = system_clock::now();
//
//	duration = duration_cast<microseconds>(end - start);
//	printf("CPU undistort fun for 15 images cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//
//	waitKey(0);
//	return 0;
//}



//===================== 2. CPU+GPU undistort chessbord images ===================//
//#include <iostream>
//#include <string>
//#include <opencv2/opencv.hpp>
//#include <chrono> 
//
//using namespace std;
//using namespace chrono;
//using namespace cv::cuda;
//
//using namespace cv;
//
//Mat camera_matrix = Mat::zeros(3, 3, CV_64FC1);
//Mat distortion_coefficients = Mat::zeros(1, 5, CV_64FC1);
//
//vector <GpuMat> d_src_images(15);
//vector <GpuMat> d_dst_images(15);
//vector <Mat> src_images(15);
//vector <Mat> undistort_images(15);
//Mat tmp;
//
//void ReadImage(int a)
//{
//	stringstream stringstr;
//	stringstr << a;
//	string imagePath;
//	stringstr >> imagePath;
//	imagePath += ".jpg";
//	src_images[a] = imread(imagePath);
//}
//
//
//int main()
//{
//	camera_matrix.at<double>(0, 0) = 3.0126077468510184e+03;
//	camera_matrix.at<double>(0, 2) = 1296;
//	camera_matrix.at<double>(1, 1) = 3.0094117205252010e+03;
//	camera_matrix.at<double>(1, 2) = 972;
//	camera_matrix.at<double>(2, 2) = 1;
//	//std::cout << camera_matrix << std::endl;
//
//	distortion_coefficients.at<double>(0, 0) = -1.2974312863621433;
//	distortion_coefficients.at<double>(0, 1) = 2.11178864438862;
//	distortion_coefficients.at<double>(0, 2) = 0;
//	distortion_coefficients.at<double>(0, 3) = 0;
//	distortion_coefficients.at<double>(0, 4) = -2.2622338952366023;
//	
//
//	for (int i = 0; i < 15; i++)
//		ReadImage(i);
//
//
//	auto start = system_clock::now();
//	for (int j = 0; j < 15; j++)
//		d_src_images[j].upload(src_images[j]);;
//	
//	cv::Size imageSize(2592,1944);
//
//	//矫正
//	//undistort(src, distortion, camera_matrix, distortion_coefficients);
//
//	cv::Mat map1, map2;
//	initUndistortRectifyMap(
//		camera_matrix, distortion_coefficients, Mat(),
//		camera_matrix, imageSize,
//		CV_32FC1, map1, map2);
//
//	
//	cv::cuda::GpuMat m_mapx;
//	cv::cuda::GpuMat m_mapy;
//	m_mapx = ::cv::cuda::GpuMat(map1);
//	m_mapy = ::cv::cuda::GpuMat(map2);
//
//	
//	for(int i=0;i<15;i++)
//	cv::cuda::remap(d_src_images[i], d_dst_images[i], m_mapx, m_mapy, INTER_LINEAR); // 10 images cost 30ms
//	
//	
//	for (int j = 0; j<15; j++)
//	d_dst_images[j].download(undistort_images[j]);
//
//	auto end = system_clock::now();
//
//	auto duration = duration_cast<microseconds>(end - start);
//	printf("(GPU)  cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//
//	waitKey(0);
//	return 0;
//}


//===================== 3. SURF and ORB descriptor caculation ===================//
//============ GPU ORB =============//
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
//#include "opencv2/cudabgsegm.hpp"
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/cuda_stream_accessor.hpp"
//#include "opencv2/cudafeatures2d.hpp"
//#include "opencv2/cudaimgproc.hpp"
//#include "opencv2/cudaarithm.hpp"
//#include "opencv2/cudafilters.hpp"
//#include "opencv2/cudawarping.hpp"
//
//#include "opencv2/features2d.hpp"
//#include <vector>
//#include <chrono> 
//
//
//using namespace cv;
//using namespace cuda;
//using namespace std;
//using namespace chrono; 
//
//int real_num = 15;
//
//
//vector<GpuMat> d_full_images(real_num);
//vector<GpuMat> d_full_grays(real_num);
//vector<GpuMat> d_keypts(real_num);
//vector<GpuMat> d_descripts(real_num);
//
//vector<Mat> h_keypts(real_num);
//vector<Mat> h_descripts(real_num);
//
//vector<GpuMat> d_descripts_32F(real_num);
//vector<Mat> h_full_images(real_num);
//
//vector<vector<DMatch>>v_matches(real_num - 1);
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//vector<GpuMat> d_descriptors(real_num);
//Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher(NORM_L1); 
//
//
//void best2nearestmatch(GpuMat &d_query_des, GpuMat &d_train_des, vector<vector<DMatch>> &matches)
//{
//	MatchesSet mset;
//	//	Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher(NORM_L1); 
//	matches.clear();
//	// Find 1->2 matches
//	matcher->knnMatch(d_query_des, d_train_des, matches, 2);
//	for (size_t i = 1; i < matches.size(); ++i)
//	{
//		if (matches[i].size() < 2)
//			continue;
//		const DMatch& m0 = matches[i][0];
//		const DMatch& m1 = matches[i][1];
//		if (m0.distance < (1.f - 0.25) * m1.distance)
//		{
//			mset.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
//		}
//	}
//	//cout << mset.size() << endl;
//}
//
//
//int main()
//{
//	vector<String> files;
//
//	glob("data file directroy\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	for (int j = 0; j < num_images; j++)
//		d_full_images[j].upload(h_full_images[j]);
//
//	auto start = system_clock::now();
//
//	cuda::cvtColor(d_full_images[0], d_full_grays[0], COLOR_BGR2GRAY);
//	Ptr<cuda::ORB> d_orb = cuda::ORB::create(1500, 1.2f, 6, 31, 0, 2, 0, 31, 20, true);
//	d_orb->detectAndComputeAsync(d_full_grays[0], cuda::GpuMat(), d_keypts[0], d_descripts[0]);
//	d_descripts[0].convertTo(d_descripts_32F[0], CV_32F);
//
//	d_keypts[0].download(h_keypts[0]); 
//	d_descripts[0].download(h_descripts[0]); 
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cuda::cvtColor(d_full_images[i], d_full_grays[i], COLOR_BGR2GRAY);
//		d_orb->detectAndComputeAsync(d_full_grays[i], cuda::GpuMat(), d_keypts[i], d_descripts[i]);
//		d_descripts[i].convertTo(d_descripts_32F[i], CV_32F);
//
//		d_keypts[i].download(h_keypts[i]); 
//		d_descripts[i].download(h_descripts[i]); 
//
//		best2nearestmatch(d_descripts_32F[i], d_descripts_32F[i - 1], v_matches);
//
//	}
//
//	auto end = system_clock::now();
//
//	auto duration = duration_cast<microseconds>(end - start);
//	printf("(GPU) uploading cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);
//	
//	waitKey(0);
//	return 0;
//}

//============ GPU SURF =============//
//#include <iostream>
//
//#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
//#include "opencv2/cudabgsegm.hpp"
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/cuda_stream_accessor.hpp"
//#include "opencv2/cudafeatures2d.hpp"
//#include "opencv2/cudaimgproc.hpp"
//#include "opencv2/cudaarithm.hpp"
//#include "opencv2/cudafilters.hpp"
//#include "opencv2/cudawarping.hpp"
//#include "opencv2/features2d.hpp"
//#include <vector>
//
//using namespace cv;
//using namespace cuda;
//using namespace std;
//
//
//int real_num = 15;
//
//vector<GpuMat> d_full_images(real_num);
//vector<GpuMat> d_full_grays(real_num);
//vector<GpuMat> d_keypts(real_num);
//vector<GpuMat> d_descripts(real_num);
//
//vector<GpuMat> d_descripts_32F(real_num);
//vector<Mat> h_full_images(real_num);
//
//vector<vector<DMatch>>v_matches(real_num-1);
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//	start = getTickCount();
//
//	d_full_images[0].upload(h_full_images[0]);
//	cuda::cvtColor(d_full_images[0], d_full_grays[0], COLOR_BGR2GRAY);
//	cv::cuda::SURF_CUDA surf(1500, 4, 3);
//
//	surf(d_full_grays[0], cuda::GpuMat(), d_keypts[0], d_descripts[0]);
//	d_descripts[0].convertTo(d_descripts_32F[0], CV_32F);
//
//	Ptr<cv::cuda::DescriptorMatcher> d_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_L2);
//
//	for (int i = 1; i < num_images; i++)
//	{
//		d_full_images[i].upload(h_full_images[i]);
//		cuda::cvtColor(d_full_images[i], d_full_grays[i], COLOR_BGR2GRAY);
//		surf(d_full_grays[i], cuda::GpuMat(), d_keypts[i], d_descripts[i]);
//		d_descripts[0].convertTo(d_descripts_32F[i], CV_32F);
//
////		d_matcher->match(d_descripts_32F[i], d_descripts_32F[i - 1], v_matches[i - 1]);
//
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//	waitKey(0);
//	return 0;
//}

//============ CPU ORB =============//
//#include <iostream>
//#include <signal.h>
//#include <vector>
//
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches(real_num-1);
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//	start = getTickCount();
//
//	cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//
//	Ptr<ORB> d_orb = ORB::create(500, 1.2f, 6, 31, 0, 2);
//	d_orb->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//	Ptr<DescriptorMatcher> d_matcher = DescriptorMatcher::create(NORM_L2);
//	
//	for (int i = 1; i < num_images; i++)
//	{
//		cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		d_orb->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
////		d_matcher->match(h_descripts[i], h_descripts[i-1], v_matches[i-1]);
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//	waitKey(0);
//	return 0;
//}

//============ CPU SURF =============//
//#include <iostream>
//#include <vector>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches;
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//	start = getTickCount();
//
//	cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//	Ptr<SURF> surf = SURF::create(1500);
//	surf->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		surf->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
//		//d_matcher->match(h_descripts[i], h_descripts[i-1], v_matches[i-1]);
//	}
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//	waitKey(0);
//	return 0;
//}


//================= 4.low ratio ======================//
//============ CPU ORB for experiments with low ratio =============//
//#include <iostream>
//#include <vector>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches;
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//cv::FlannBasedMatcher matcher;
////cv::BFMatcher matcher(NORM_L2);
////Ptr<cv::DescriptorMatcher> matcher = makePtr<BFMatcher>((int)NORM_L2);
//
////---------------------- code from opencv2 stitcher detail module ---------------------------//
//void best2nearestmatch(Mat &query_des, Mat &train_des, vector<vector<DMatch>> &matches)
//{
//	MatchesSet mset;
//
//	// Find 1->2 matches
//	matcher.knnMatch(query_des, train_des, matches, 2);
//	for (size_t i = 1; i < matches.size(); ++i)
//	{
//		if (matches[i].size() < 2)
//			continue;
//		const DMatch& m0 = matches[i][0];
//		const DMatch& m1 = matches[i][1];
//		if (m0.distance < (1.f - 0.15) * m1.distance)
//		{
//			mset.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
//		}
//	}
////	cout << mset.size() << endl;
//}
//
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//
//	start = getTickCount();
//
//	cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//	cv::Ptr<cv::Feature2D> d_orb = cv::ORB::create(500);
//
//	d_orb->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//	h_descripts[0].convertTo(h_descripts[0], CV_32F);
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		d_orb->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
//		//d_matcher->match(h_descripts[i], h_descripts[i-1], v_matches[i-1]);
//		//best2nearestmatch(h_descripts[i], h_descripts[i - 1], v_matches);
//
//		h_descripts[i].convertTo(h_descripts[i], CV_32F);
//	}
//
//	for (int i = 1; i < num_images; i++)
//	{
//		best2nearestmatch(h_descripts[i], h_descripts[i - 1], v_matches);
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//	waitKey(0);
//	return 0;
//}

//============ GPU ORB for experiments with low ratio =============//
//#include <iostream>
//#include <vector>
//#include <opencv2/xfeatures2d.hpp>
//#include "opencv2/core/cuda.hpp"
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;
//using namespace cv::cuda;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches;
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//vector<GpuMat> d_descriptors(real_num);
//Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher(NORM_L1);
//
//
//---------------------- code from opencv2 stitcher detail module ---------------------------//
//void best2nearestmatch(GpuMat &d_query_des, GpuMat &d_train_des, vector<vector<DMatch>> &matches)
//{
//	MatchesSet mset;
//	matches.clear();
//	// Find 1->2 matches
//	matcher->knnMatch(d_query_des, d_train_des, matches, 2);
//	for (size_t i = 1; i < matches.size(); ++i)
//	{
//		if (matches[i].size() < 2)
//			continue;
//		const DMatch& m0 = matches[i][0];
//		const DMatch& m1 = matches[i][1];
//		if (m0.distance < (1.f - 0.25) * m1.distance)
//		{
//			mset.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
//		}
//	}
//	//cout << mset.size() << endl;
//}
//
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//
//
//	cv::cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//	cv::Ptr<cv::Feature2D> d_orb = cv::ORB::create(1500);
//
//	d_orb->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//	h_descripts[0].convertTo(h_descripts[0], CV_32F); 
//	
//	d_descriptors[0].upload(h_descripts[0]);
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cv::cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		d_orb->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
//		h_descripts[i].convertTo(h_descripts[i], CV_32F); 
//
//		d_descriptors[i].upload(h_descripts[i]);
//	}
//
//	start = getTickCount();
//
//	for (int i = 1; i < num_images; i++)
//	{
//		best2nearestmatch(d_descriptors[i], d_descriptors[i - 1], v_matches);
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//  waitKey(0);
//	return 0;
//}


//============ CPU SURF for experiments with low ratio =============//
//#include <iostream>
//#include <signal.h>
//#include <vector>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches;
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//cv::FlannBasedMatcher matcher;
////cv::BFMatcher matcher(NORM_L2);
////Ptr<cv::DescriptorMatcher> matcher = makePtr<BFMatcher>((int)NORM_L2);
//
//---------------------- code from opencv2 stitcher detail module ---------------------------//
//void best2nearestmatch(Mat &query_des,Mat &train_des, vector<vector<DMatch>> &matches)
//{
//	MatchesSet mset;
//
//	// Find 1->2 matches
//	matcher.knnMatch(query_des, train_des, matches, 2);
//	for (size_t i = 1; i < matches.size(); ++i)
//	{
//		if (matches[i].size() < 2)
//			continue;
//		const DMatch& m0 = matches[i][0];
//		const DMatch& m1 = matches[i][1];
//		if (m0.distance < (1.f - 0.25) * m1.distance)
//		{
//			mset.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
//		}
//	}	
//}
//
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//
//
//	cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//	Ptr<SURF> surf = SURF::create(1500);
//	surf->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//	
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		surf->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
//		//d_matcher->match(h_descripts[i], h_descripts[i-1], v_matches[i-1]);
//		//best2nearestmatch(h_descripts[i], h_descripts[i - 1], v_matches);
//	}
//
//	start = getTickCount();
//
//	for (int i = 1; i < num_images; i++)
//	{
//		best2nearestmatch(h_descripts[i], h_descripts[i - 1], v_matches);
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//  waitKey(0);
//	return 0;
//}

//============ GPU SURF for experiments with low ratio =============//
//#include <iostream>
//#include <vector>
//#include <opencv2/xfeatures2d.hpp>
////#include "opencv2/cudabgsegm.hpp"
//#include "opencv2/core/cuda.hpp"
////#include "opencv2/core/cuda_stream_accessor.hpp"
////#include "opencv2/cudafeatures2d.hpp"
////#include "opencv2/cudaimgproc.hpp"
////#include "opencv2/cudaarithm.hpp"
////#include "opencv2/cudafilters.hpp"
////#include "opencv2/cudawarping.hpp"
//
//#include <opencv2/opencv.hpp>
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;
//using namespace cv::cuda;
//
//int real_num = 15;
//
//
//vector<Mat> h_descripts(real_num);
//vector<Mat> h_full_images(real_num);
//vector<Mat> h_full_grays(real_num);
//vector<vector<DMatch>> v_matches;
//vector<vector<KeyPoint>> v_h_keypts(real_num);
//
//typedef std::set<std::pair<int, int> > MatchesSet;
//
//vector<GpuMat> d_descriptors(real_num);
//Ptr<cuda::DescriptorMatcher> matcher = cuda::DescriptorMatcher::createBFMatcher(NORM_L1);
//
//---------------------- code from opencv2 stitcher detail module ---------------------------//
//void best2nearestmatch(GpuMat &d_query_des,GpuMat &d_train_des, vector<vector<DMatch>> &matches)
//{
//	MatchesSet mset;
//	matches.clear();
//	// Find 1->2 matches
//	matcher->knnMatch(d_query_des, d_train_des, matches, 2);
//	for (size_t i = 1; i < matches.size(); ++i)
//	{
//		if (matches[i].size() < 2)
//			continue;
//		const DMatch& m0 = matches[i][0];
//		const DMatch& m1 = matches[i][1];
//		if (m0.distance < (1.f - 0.25) * m1.distance)
//		{
//			mset.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
//		}
//	}	
//}
//
//
//int main()
//{
//	vector<String> files;
//	glob("data file directory\\", files);
//
//	int num_images = files.size();
//
//	for (int i = 0; i < num_images; i++)
//	{
//		h_full_images[i] = imread(files[i]);
//	}
//
//	if (!h_full_images[0].data)
//	{
//		cout << "error reading images " << endl;
//		return -1;
//	}
//
//
//	int64 start, end;
//	double time;
//
//
//	cv::cvtColor(h_full_images[0], h_full_grays[0], COLOR_BGR2GRAY);
//	Ptr<SURF> surf = SURF::create(1500);
//	surf->detectAndCompute(h_full_grays[0], Mat(), v_h_keypts[0], h_descripts[0]);
//
//	d_descriptors[0].upload(h_descripts[0]);
//
//
//	for (int i = 1; i < num_images; i++)
//	{
//		cv::cvtColor(h_full_images[i], h_full_grays[i], COLOR_BGR2GRAY);
//		surf->detectAndCompute(h_full_grays[i], Mat(), v_h_keypts[i], h_descripts[i]);
//		d_descriptors[i].upload(h_descripts[i]);
//	}
//
//	start = getTickCount();
//
//	for (int i = 1; i < num_images; i++)
//	{
//		best2nearestmatch(d_descriptors[i], d_descriptors[i - 1], v_matches);
//	}
//
//	end = getTickCount();
//	time = (double)(end - start) * 1000 / getTickFrequency();
//	cout << "Total time : " << time << " ms" << endl;
//
//  waitKey(0);
//	return 0;
//}
