
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
using namespace openni;
pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer");
int threshold_value1 = 0;
int threshold_value2 = 0;
char* trackbar_value1 = "H_low Value";
char* trackbar_value2 = "H_high Value";
void Threshold_Demo(int, void*)
{}
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	Mat pic(480,640,CV_64FC1);

	if (!viewer.wasStopped())
	{

		/*
		Eigen::Quaternionf ori1(1,0,0,0);
		//cloud->sensor_orientation_ = ori1;
		Eigen::Quaternionf ori=cloud->sensor_orientation_;
		ori=ori1;

		*/
		for(int i=0;i<480;i++)
			for(int j=0;j<640;j++)
			{
				//cout<< cloud->points[i*640+j].x<<" "<<cloud->points[i*640+j].y<<" "<<cloud->points[i*640+j].z<<endl;
				pic.at<double>(i,j)=cloud->points[i*640+j].y;
			}
			double m1,m2;
			minMaxIdx(pic, &m1, &m2);
			pic=pic-m1;
			pic.convertTo(pic,CV_8U,255/(m2-m1));
			flip(pic,pic,1);
			Mat pic1,pic2;
			inRange(pic, threshold_value1, threshold_value2,pic1);
			//inRange(pic, 60, 240,pic1);
			//inRange(pic, 0, 60,pic2);
			//pic=pic1+pic2;


			cv::imshow("y",pic1);
			char key = cv::waitKey(20);		
			/*
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (0.1);

			seg.setInputCloud (cloud);
			seg.segment (*inliers, *coefficients);

			if (inliers->indices.size () == 0)
			{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			return ;
			}

			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " " 
			<< coefficients->values[3] << std::endl;

			std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
			for (size_t i = 0; i < inliers->indices.size (); ++i)
			std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
			<< cloud->points[inliers->indices[i]].y << " "
			<< cloud->points[inliers->indices[i]].z << std::endl;
			*/
			viewer.showCloud(cloud);
	}
}


void CheckOpenNIError(Status result, string status)
{
	if (result != STATUS_OK)
		cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}
string num2str(int i){
	stringstream s;
	s << i;
	return s.str();
}
int main(int argc, char** argv)
{
	//pcl test
	cv::namedWindow("BarValueThres");
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
	Eigen::Quaternionf ori1(0,0,0,0);






	//OpenNI2 image
	Status result = STATUS_OK;
	VideoFrameRef oniDepthImg;
	VideoFrameRef oniColorImg;

	//OpenCV image
	cv::Mat cvDepthImg;
	cv::Mat cvBGRImg;
	cv::Mat cvFusionImg;

	cv::namedWindow("depth");
	cv::namedWindow("image");
	cv::namedWindow("fusion");
	char key = 0;
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		boost::bind(&cloud_cb_, _1);
	interface->registerCallback(f);
	interface->start();

	//¡¾1¡¿
	// initialize OpenNI2
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	// open device  
	Device device;
	result = device.open(openni::ANY_DEVICE);

	//¡¾2¡¿
	// create depth stream 
	VideoStream oniDepthStream;
	result = oniDepthStream.create(device, openni::SENSOR_DEPTH);

	//¡¾3¡¿
	// set depth video mode
	VideoMode modeDepth;
	modeDepth.setResolution(640, 480);
	modeDepth.setFps(30);
	modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode(modeDepth);
	// start depth stream
	result = oniDepthStream.start();

	// create color stream
	VideoStream oniColorStream;
	result = oniColorStream.create(device, openni::SENSOR_COLOR);
	// set color video mode
	VideoMode modeColor;
	modeColor.setResolution(640, 480);
	modeColor.setFps(30);
	modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
	oniColorStream.setVideoMode(modeColor);

	//¡¾4¡¿
	// set depth and color imge registration mode
	if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	// start color stream
	result = oniColorStream.start();
	int index = 0;
	while (key != 27&&!viewer.wasStopped())
	{
		// read frame
		createTrackbar(trackbar_value1,
			"BarValueThres", &threshold_value1,
			255, Threshold_Demo);

		createTrackbar(trackbar_value2,
			"BarValueThres", &threshold_value2,
			255, Threshold_Demo);
		if (oniColorStream.readFrame(&oniColorImg) == STATUS_OK)
		{
			// convert data into OpenCV type
			cv::Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());
			cv::cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
			flip(cvBGRImg, cvBGRImg, 1);
			cv::imshow("image", cvBGRImg);
		}

		if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK)
		{
			cv::Mat cvRawImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
			double m1, m2;
			minMaxIdx(cvRawImg16U, &m1, &m2);
			//cout << m2<< endl;
			//cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
			flip(cvRawImg16U, cvRawImg16U, 1);
			cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / 6000);
			if (key == 32)
			{
				FileStorage fs((num2str(index++) + ".xml").c_str(), FileStorage::WRITE);
				fs << "data" << cvRawImg16U;
				//fs << "depth" << cvDepthImg;

				fs.release();
				//imwrite((num2str(index++) + ".jpg").c_str(), cvRawImg16U);
			}
			//¡¾5¡¿
			// convert depth image GRAY to BGR

			cv::cvtColor(cvDepthImg, cvFusionImg, CV_GRAY2BGR);

			cv::imshow("depth", cvDepthImg);
			//imwrite((num2str(index++) + ".jpg").c_str(), cvDepthImg);
		}
		//¡¾6¡¿
		cv::addWeighted(cvBGRImg, 0.5, cvFusionImg, 0.5, 0, cvFusionImg);
		cv::imshow("fusion", cvFusionImg);
		key = cv::waitKey(20);
	}
	interface->stop();
	//cv destroy
	cv::destroyWindow("depth");
	cv::destroyWindow("image");
	cv::destroyWindow("fusion");

	//OpenNI2 destroy
	oniDepthStream.destroy();
	oniColorStream.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}



