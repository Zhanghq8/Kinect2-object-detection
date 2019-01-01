#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sensor_msgs/Image.h>
using namespace cv;
using namespace std;

int iLowH = 70;
int iHighH = 130;
int iLowS = 90;
int iHighS = 255;
int iLowV = 90;
int iHighV = 255;
int number=3;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  public:
  ImageConverter(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &ImageConverter::imageCb, this);
    cv::namedWindow("Thresholded Image");
	cv::namedWindow("colorImg");

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 255);
	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);
	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
  }

  ~ImageConverter()
  {
	cv::destroyWindow("Thresholded Image");
	cv::destroyWindow("colorImg");
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;
	Mat colorImg;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  colorImg = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(colorImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2],hsvSplit[2]);
	merge(hsvSplit,imgHSV);
	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

	imshow("Thresholded Image", imgThresholded); 
	imshow("colorImg", colorImg); 
	cv::waitKey(3);

  }
 };

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;
	ImageConverter ic;
	ros::spin();
	return 0;
}
