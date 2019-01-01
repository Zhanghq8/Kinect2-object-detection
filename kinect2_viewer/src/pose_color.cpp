/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/legacy/legacy.hpp>//no need for opencv 3.0

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>


using namespace cv;
using namespace std;

ros::Publisher pose_arm_pub,pose_car_pub;

geometry_msgs::Vector3 pose_arm,pose_car;

float pose_max_x,pose_max_y,pose_max_z=0;
float prepos_x=0,prepos_y=0,prepos_z=0;


/////////
volatile int poseX;
volatile int poseY;


void TimerCallback(const ros::TimerEvent&)
{
	pose_arm_pub.publish(pose_arm);

}

void TimerCallback1(const ros::TimerEvent&)
{
	pose_car_pub.publish(pose_car);

}

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

void run(const Mode mode)
{
    start(mode);
    stop();
}

private:
  void start(const Mode mode)
  {

      this->mode = mode;
      running = true;

      std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
      std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

      image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
      subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
      subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
      subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
      subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

      if(useExact)
      {

          syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
          syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
      }
      else
      {
          syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
          syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
      }

      spinner.start();

      std::chrono::milliseconds duration(1);
      while(!updateImage || !updateCloud)
      {
          if(!ros::ok())
              {
              return;
              }
          std::this_thread::sleep_for(duration);
      }
      cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
      cloud->height = color.rows;
      cloud->width = color.cols;
      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);
      createLookup(this->color.cols, this->color.rows);

      switch(mode)
      {
      case CLOUD:
        cloudViewer();
      break;
      case IMAGE:
    	  imageViewer();
      break;
      case BOTH:
    	  imageViewerThread = std::thread(&Receiver::imageViewer, this);
    	  cloudViewer();
      break;
    }
  }

  void stop()
  {
      spinner.stop();

      if(useExact)
      {
    	  delete syncExact;
      }
      else
      {
    	  delete syncApproximate;
      }
      delete subImageColor;
      delete subImageDepth;
      delete subCameraInfoColor;
      delete subCameraInfoDepth;

      running = false;
      if(mode == BOTH)
      {
    	  imageViewerThread.join();
      }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
      cv::Mat color, depth;

      readCameraInfo(cameraInfoColor, cameraMatrixColor);
      readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
      readImage(imageColor, color);
      readImage(imageDepth, depth);

      // IR image input
      if(color.type() == CV_16U)

          {
    	  cv::Mat tmp;
          color.convertTo(tmp, CV_8U, 0.02);
          cv::cvtColor(tmp, color, CV_GRAY2BGR);
          }

      lock.lock();
      this->color = color;
      this->depth = depth;
      updateImage = true;
      updateCloud = true;
      lock.unlock();
  }

  void imageViewer()
  {
	  cv::Mat color, depth, depthDisp, combined;
      std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
      double fps = 0;
      size_t frameCount = 0;
      std::ostringstream oss;
      const cv::Point pos(5, 15);
      const cv::Scalar colorText = CV_RGB(255, 255, 255);
      const double sizeText = 0.5;
      const int lineText = 1;
      const int font = cv::FONT_HERSHEY_SIMPLEX;

      cv::namedWindow("Image Viewer");
      oss << "starting...";

      start = std::chrono::high_resolution_clock::now();
      for(; running && ros::ok();)

    	  {
          if(updateImage)
              {
              lock.lock();
              color = this->color;
              depth = this->depth;
              updateImage = false;
              lock.unlock();


              ++frameCount;
              now = std::chrono::high_resolution_clock::now();
              double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
              if(elapsed >= 1.0)

            	  {
                  fps = frameCount / elapsed;
                  oss.str("");
                  oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                  start = now;
                  frameCount = 0;
                  }

              dispDepth(depth, depthDisp, 12000.0f);
              combine(color, depthDisp, combined);

              //combined = color;

              cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
              cv::imshow("Image Viewer", combined);
              }

              int key = cv::waitKey(1);
              switch(key & 0xFF)
              {
              case 27:
              case 'q':
                  running = false;
                  break;
              case ' ':
              case 's':

            	  if(mode == IMAGE)
                  	  {
            		  createCloud(depth, color, cloud);
            		  saveCloudAndImages(cloud, color, depth, depthDisp);
                  	  }
            	  else
            	  {
            		  save = true;
            	  }
            	  break;
              }
    	  }
      cv::destroyAllWindows();
      cv::waitKey(100);
  }

  void cloudViewer()
  {
	  cv::Mat color, depth;
	  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
	  double fps = 0;
	  size_t frameCount = 0;
	  std::ostringstream oss;
	  std::ostringstream ossXYZ;
	  const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 0, 0);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

	  int img_x = poseX;
    int img_y = poseY;

    geometry_msgs::PointStamped ptMsg;
    ptMsg.header.frame_id = "kinect_link";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();
    pose_get(color);//pixel
    const std::string window_name = "color viewer";
    const std::string window_name1 = "depth viewer";

	  createCloud(depth, color, cloud);


	  for(; running && ros::ok();)
	  {
	      if(updateCloud)
	      {
	          lock.lock();
	          color = this->color;
	          depth = this->depth;
	          updateCloud = false;
	          lock.unlock();
	          pose_get(color);
	          createCloud(depth, color, cloud);

	          ////////////////////
	          img_x = poseX;
	          img_y = poseY;

	          const pcl::PointXYZRGBA& pt = cloud->points[img_y * depth.cols + img_x];
	          ptMsg.point.x = pt.x;
	          ptMsg.point.y = pt.y;
	          ptMsg.point.z = pt.z;
	          ptMsg.header.stamp = ros::Time::now();

            // cal fps
	          ++frameCount;
	          now = std::chrono::high_resolution_clock::now();
	          double elapsed =
	          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;

	          if(elapsed >= 1.0)
	          {
	        	  fps = frameCount / elapsed;
	        	  oss.str("");
	        	  oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
	        	  start = now;
	        	  frameCount = 0;
	          }
	          cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
	          ossXYZ.str("");
	          ossXYZ << "( " << ptMsg.point.x << ", " << ptMsg.point.y
	                                    << ", " << ptMsg.point.z << " )";
	          cout<<"x= "<<ptMsg.point.x<<"****"<<"y= "<<ptMsg.point.y<<"****"<<"z= "<<ptMsg.point.z<<endl;

	          pose_arm.x=ptMsg.point.x;
	          pose_arm.y=ptMsg.point.y;
	          pose_arm.z=ptMsg.point.z;

            if((ptMsg.point.x-prepos_x<0.20)&&(ptMsg.point.y-prepos_y<0.20)&&(ptMsg.point.z-prepos_z<0.20))
              {
	              if (ptMsg.point.z>pose_max_z)
		              {	 
		                pose_max_z=ptMsg.point.z;
		                pose_car.x=ptMsg.point.x;
	                  pose_car.y=ptMsg.point.y;
		                pose_car.z=pose_max_z;	          
	                }
              }

	          prepos_x=ptMsg.point.x;
	          prepos_y=ptMsg.point.y;
	          prepos_z=ptMsg.point.z;

	          cv::putText(color, ossXYZ.str(), cv::Point(img_x, img_y), font, 0.5, colorText, 0.3, CV_AA);
	          cv::circle(color, cv::Point(poseX, poseY), 5, cv::Scalar(0, 0, 255), -1);
	          cv::imshow(window_name, color);
	          cv::imshow(window_name1, depth);
	          cv::waitKey(1);
	      }
	  }

	  cv::destroyAllWindows();
	  cv::waitKey(100);
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
    	  *itC = cameraInfo->K[i];
      }
  }

  void pose_get(const cv::Mat &color)
  {

	  cv::Mat colorImg, imgHSV, hsvImage, image;

	  /////////////
	  colorImg=color;
	  //imshow("result image", colorImg);

      //yellow
      int iLowH = 23;
      int iHighH = 60;
      int iLowS = 106;
      int iHighS = 255;
      int iLowV = 90;
      int iHighV = 255;
      
     //  //green
	    // int iLowH = 30;
  	  // int iHighH = 82;
  	  // int iLowS = 111;
  	  // int iHighS = 255;
  	  // int iLowV = 90;
  	  // int iHighV = 255;

  	  int thresh = 30;//////////
  	  vector<Mat> hsvSplit;

  	  cvtColor(colorImg, hsvImage, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  	  split(hsvImage, hsvSplit);
  	  equalizeHist(hsvSplit[2], hsvSplit[2]);
  	  merge(hsvSplit, hsvImage);

  	  Mat imgThresholded;
  	  inRange(hsvImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
  	  //imshow("imgThresholded", imgThresholded);

  	  Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

  	  morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
  	  morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

  	  GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
  	  blur(imgThresholded, imgThresholded, Size(3, 3));

  	  Mat canny_output;
  	  vector<vector<Point> > contours;
  	  vector<Vec4i> hierarchy;
  	  Canny(imgThresholded, canny_output, thresh, thresh * 3, 3);

  	  findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  	  //********************
  	  if (contours.size() == 0)
  	  {
  		  cout << "target object missing" << endl;
  	  }
  	  else
  	  {
  		  Mat result(imgThresholded.size(), CV_8U, Scalar(0));
  		  Mat area(1, contours.size(), CV_32FC1);
  		  float maxArea = area.at<float>(0);
  		  int max = 0;
  		  for (int i = 0; i < (int)contours.size(); i++)
  		  {
  			  area.at<float>(i) = contourArea(contours[i]);
  			  if (maxArea < area.at<float>(i))
  			  {
  				  maxArea = area.at<float>(i);
  				  max = i;
  			  }
  		  }

  		  Rect r = boundingRect(Mat(contours[max]));
  		  rectangle(image, r, Scalar(255), 1);

  		  vector<Moments> mu(contours.size());
  		  mu[max] = moments(contours[max], false);

  		  vector<Point2f> mc(contours.size());
  		  mc[max] = Point2d(mu[max].m10 / mu[max].m00, mu[max].m01 / mu[max].m00);
  		  //circle(image, mc[max], 5, Scalar(0, 0, 255), -1, 8, 0);

  		  //imshow("result image", image);
  		  cout << "x= " << (int)mc[max].x << "****" << "y= " << (int)mc[max].y << endl;

  		  poseX=(int)mc[max].x;
  		  poseY=(int)mc[max].y;
  	  }
  }


  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
	  cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
	  const uint32_t maxInt = 255;

      #pragma omp parallel for
	  for(int r = 0; r < in.rows; ++r)
	  {
		  const uint16_t *itI = in.ptr<uint16_t>(r);
		  uint8_t *itO = tmp.ptr<uint8_t>(r);

		  for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
		  {
			  *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
		  }
	  }

	  cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
	  out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

      #pragma omp parallel for
	  for(int r = 0; r < inC.rows; ++r)
	  {
		  const cv::Vec3b
		  *itC = inC.ptr<cv::Vec3b>(r),
		  *itD = inD.ptr<cv::Vec3b>(r);
		  cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

		  for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
		  {
			  itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
			  itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
			  itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
		  }
	  }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
	  const float badPoint = std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for
	  for(int r = 0; r < depth.rows; ++r)
	  {
		  pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
		  const uint16_t *itD = depth.ptr<uint16_t>(r);
		  const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
		  const float y = lookupY.at<float>(0, r);
		  const float *itX = lookupX.ptr<float>();

		  for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
		  {
			  register const float depthValue = *itD / 1000.0f;
			  // Check for invalid measurements
			  if(*itD == 0)
			  {
				  // not valid
				  itP->x = itP->y = itP->z = badPoint;
				  itP->rgba = 0;
				  continue;
			  }
			  itP->z = depthValue;
			  itP->x = *itX * depthValue;
			  itP->y = y * depthValue;
			  itP->b = itC->val[0];
			  itP->g = itC->val[1];
			  itP->r = itC->val[2];
			  itP->a = 255;
		  }
	  }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
	  oss.str("");
	  oss << "./" << std::setfill('0') << std::setw(4) << frame;
	  const std::string baseName = oss.str();
	  const std::string cloudName = baseName + "_cloud.pcd";
	  const std::string colorName = baseName + "_color.jpg";
	  const std::string depthName = baseName + "_depth.png";
	  const std::string depthColoredName = baseName + "_depth_colored.png";

	  OUT_INFO("saving cloud: " << cloudName);
	  writer.writeBinary(cloudName, *cloud);
	  OUT_INFO("saving color: " << colorName);
	  cv::imwrite(colorName, color, params);
	  OUT_INFO("saving depth: " << depthName);
	  cv::imwrite(depthName, depth, params);
	  OUT_INFO("saving depth: " << depthColoredName);
	  cv::imwrite(depthColoredName, depthColored, params);
      OUT_INFO("saving complete!");
      ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
	  const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
	  const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
	  const float cx = cameraMatrixColor.at<double>(0, 2);
	  const float cy = cameraMatrixColor.at<double>(1, 2);
	  float *it;

	  lookupY = cv::Mat(1, height, CV_32F);
	  it = lookupY.ptr<float>();
	  for(size_t r = 0; r < height; ++r, ++it)
	  {
		  *it = (r - cy) * fy;
	  }

	  lookupX = cv::Mat(1, width, CV_32F);
	  it = lookupX.ptr<float>();
	  for(size_t c = 0; c < width; ++c, ++it)
	  {
		  *it = (c - cx) * fx;
	  }
  }
};



int main(int argc, char **argv)
{
	#if EXTENDED_OUTPUT
	ROSCONSOLE_AUTOINIT;
	if(!getenv("ROSCONSOLE_FORMAT"))
	{
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
	}
	#endif

	ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::NodeHandle nh;
	ros::Timer timer = nh.createTimer(ros::Duration(0.1),TimerCallback);
  pose_arm_pub= nh.advertise<geometry_msgs::Vector3>("obj_vel", 1000);
	ros::NodeHandle n;
	ros::Timer timer1 = n.createTimer(ros::Duration(0.1),TimerCallback1);
  pose_car_pub= n.advertise<geometry_msgs::Vector3>("/pose_3d/pose_car_pub", 1000);

	if(!ros::ok())
	{
		return 0;
	}

	std::string ns = K2_DEFAULT_NS;
	std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
	std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	bool useExact = true;
	bool useCompressed = false;
	Receiver::Mode mode = Receiver::CLOUD;
	topicColor = "/" + ns + topicColor;
	topicDepth = "/" + ns + topicDepth;
	OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
	OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

	Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

	OUT_INFO("starting receiver...");
	receiver.run(mode);

	ros::shutdown();
	return 0;
}

