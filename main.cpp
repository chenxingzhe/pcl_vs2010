
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
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
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	if (!viewer.wasStopped())
	{
		
		Eigen::Quaternionf ori;
		Eigen::Quaternionf ori1(1,100,100,100);
		ori=cloud->sensor_orientation_;
		ori=ori1;
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
	Status result = STATUS_OK;

	//OpenNI2 image
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
	
	//【1】
	// initialize OpenNI2
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	// open device  
	Device device;
	result = device.open(openni::ANY_DEVICE);

	//【2】
	// create depth stream 
	VideoStream oniDepthStream;
	result = oniDepthStream.create(device, openni::SENSOR_DEPTH);

	//【3】
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

	//【4】
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
			cout << m2<< endl;
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
			//【5】
			// convert depth image GRAY to BGR

			cv::cvtColor(cvDepthImg, cvFusionImg, CV_GRAY2BGR);

			cv::imshow("depth", cvDepthImg);
			//imwrite((num2str(index++) + ".jpg").c_str(), cvDepthImg);
		}
		//【6】
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



//使用PCLVisualizer似乎只能这样
//修改自Bastian Steder的openni_narf_keypoint_extraction.cpp
/*
#include <pcl/io/openni_grabber.h>
#include "pcl/range_image/range_image_planar.h"
#include "pcl/common/common_headers.h"
#include "pcl/visualization/range_image_visualizer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/parse.h"

using namespace pcl;
using namespace pcl::visualization;
boost::mutex depth_image_mutex;
boost::shared_ptr<openni_wrapper::DepthImage> depth_image_ptr;
bool received_new_depth_data = false;
struct EventHelper
{
  void
  depth_image_cb (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image)
  {
    if (depth_image_mutex.try_lock ())
    {
      depth_image_ptr = depth_image;
      depth_image_mutex.unlock ();
      received_new_depth_data = true;
    }
  }
}event_helper;
int main ()
{
  PCLVisualizer viewer ("Viewer");
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  pcl::Grabber* interface = new pcl::OpenNIGrabber ("#1");
  boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&) > f_depth_image =
    boost::bind (&EventHelper::depth_image_cb, &event_helper, _1);
  boost::signals2::connection c_depth_image = interface->registerCallback (f_depth_image);
  interface->start ();
  boost::shared_ptr<RangeImagePlanar> range_image_planar_ptr (new RangeImagePlanar);
  RangeImagePlanar& range_image_planar = *range_image_planar_ptr;
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    bool got_new_range_image = false;
    if (received_new_depth_data && depth_image_mutex.try_lock ())
    {
      received_new_depth_data = false;
      const unsigned short* depth_map = depth_image_ptr->getDepthMetaData().Data();
      range_image_planar.setDepthImage (depth_map, depth_image_ptr->getWidth (), depth_image_ptr->getHeight (),
 depth_image_ptr->getWidth ()/2,
 depth_image_ptr->getHeight ()/2,
 depth_image_ptr->getFocalLength(), depth_image_ptr->getFocalLength(),
 asinf (0.5f*float(depth_image_ptr->getWidth ())/float(depth_image_ptr->getFocalLength())) / (0.5f*float(depth_image_ptr->getWidth ())));
      depth_image_mutex.unlock (); 
      got_new_range_image = !range_image_planar.points.empty();
    }
    
    if (!got_new_range_image)
      continue;
viewer.removePointCloud();pcl::PointXYZ tt;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
for(int i=0,j=range_image_planar.size();i<j;++i)
tt.x=range_image_planar.points[i].x,
tt.y=range_image_planar.points[i].y,
tt.z=-range_image_planar.points[i].z,
cloud->points.push_back(tt);//vector插入太慢了,怎么修改?
cloud->width=range_image_planar.width;
cloud->height=range_image_planar.height;
    viewer.addPointCloud(cloud);
  }

  interface->stop ();
}
*/