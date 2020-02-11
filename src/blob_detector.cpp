#include <ros/ros.h>
#include <boost/thread/mutex.hpp>


#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <calibration_msgs/CalibrationPoints.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class blob_detetctor
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher blob_pub_;

public:
blob_detetctor()
  : it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/image_publisher_1576171610643837239/image_raw", 1, &ImageConverter::imageCb, this);
  blob_pub_ = it_.advertise("/image_converter/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

~blob_detetctor()
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
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob_detetctor");

  ros::NodeHandle nh;

  blob_detetctor detector;

  ros::spin();
  return 0;
}
