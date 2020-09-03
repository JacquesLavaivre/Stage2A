
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <stdlib.h> 
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;

static const string OPENCV_WINDOW = "Image window";

cv::Mat cv_ptr_output1;
cv::Mat cv_ptr_output2;
	
cv::Mat gray1; 

cv::Mat canny_output1;
cv::Mat canny_output2;

cv::Mat	 cv_ptr_blurred;
cv::Mat hsv;

int thresh = 100;

cv::RNG rng(12345);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("im_traite", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    cv::GaussianBlur(cv_ptr->image, cv_ptr_blurred, cv::Size(5, 5), 0);
	
	cvtColor(cv_ptr_blurred, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, cv::Scalar(36, 0, 0), cv::Scalar(86, 255, 255), cv_ptr_output1); // détection de la couleur verte

    inRange(hsv, cv::Scalar(10, 100, 20), cv::Scalar(25, 255, 255), cv_ptr_output2); // détection de la couleur orange


  

    //détection de contours

    cvtColor(cv_ptr_blurred, gray1, cv::COLOR_BGR2HSV);
    Canny(gray1, canny_output1, thresh, thresh*2);
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    findContours( canny_output1, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    cv::Mat drawing = cv::Mat::zeros( canny_output1.size(), CV_8UC3 );

	for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2,cv::LINE_8, hierarchy, 0 );
    }
   

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow("Image_traite_vert", cv_ptr_output1);
    cv::imshow("Image_traite_orange", cv_ptr_output2);
    cv::imshow("Contours", drawing);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

