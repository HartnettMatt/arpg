#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class edge_detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher edge_pub_;

public:
edge_detector()
  : it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("image_pub/image", 1, &edge_detector::imageCb, this);
  edge_pub_ = it_.advertise("/edge_detector/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

~edge_detector()
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

    // cv::Mat im = cv::imread("/home/matt/Pictures/IRTestImages/my_photo-1.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat im = cv_ptr->image;

    //
    // // cv::SimpleBlobDetector detector;
    // cv::SimpleBlobDetector::Params params;
    //
    // // Change thresholds
    // params.minThreshold = 100;
    // params.maxThreshold = 255;
    //
    // // Filter by Area.
    // params.filterByArea = true;
    // params.minArea = 200;
    //
    // // Filter by Circularity
    // params.filterByCircularity = false;
    // params.minCircularity = 0.1;
    //
    // // Filter by Convexity
    // params.filterByConvexity = false;
    // params.minConvexity = 0.87;
    //
    // // Filter by Inertia
    // params.filterByInertia = false;
    // params.minInertiaRatio = 0.01;
    //
    // params.thresholdStep = 10;
    //
    // cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    // std::vector<cv::KeyPoint> keypoints;
    // detector->detect( im, keypoints);
    //
    // cv::Mat im_with_keypoints;
    // drawKeypoints( im, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // // imshow("keypoints", im_with_keypoints );
    // cv::imshow(OPENCV_WINDOW, im_with_keypoints);

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    edge_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detector");

  ros::NodeHandle nh;

  edge_detector detector;

  ros::spin();
  return 0;
}
