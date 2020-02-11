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

class blob_detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher blob_pub_;

public:
blob_detector()
  : it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("image_pub/image", 1, &blob_detector::imageCb, this);
  blob_pub_ = it_.advertise("/blob_detector/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

~blob_detector()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Grab the image
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

    cv::Mat im = cv_ptr->image;
    cv::Mat new_im = cv_ptr->image;
    cv::Mat dst;
    // Increase the contrast of the image
    float alpha = 1.3;
    for( int y = 0; y < im.rows; y++ ) {
        for( int x = 0; x < im.cols; x++ ) {
            for( int c = 0; c < im.channels(); c++ ) {
                new_im.at<cv::Vec3b>(y,x)[c] =
                  cv::saturate_cast<uchar>( alpha*im.at<cv::Vec3b>(y,x)[c]);
            }
        }
    }
    im = new_im;
    // Apply a GaussianBlur to the image
    int MAX_KERNEL_LENGTH = 19;
    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ) {
       cv::GaussianBlur( im, dst, cv::Size( i, i ), 0, 0 );
     }
    im = dst;
    // Detector for the torso
    cv::SimpleBlobDetector::Params torsoParams;
    if(true){
      // Change thresholds
      torsoParams.minThreshold = 200;
      torsoParams.maxThreshold = 255;

      // Filter by Area.
      torsoParams.filterByArea = true;
      torsoParams.minArea = 10001;
      torsoParams.maxArea = 100000;

      // Filter by Circularity
      torsoParams.filterByCircularity = false;
      torsoParams.minCircularity = 0.1;

      // Filter by Convexity
      torsoParams.filterByConvexity = false;
      torsoParams.minConvexity = 0.01;

      // Filter by Inertia
      torsoParams.filterByInertia = false;
      torsoParams.minInertiaRatio = 0.01;

      torsoParams.thresholdStep = 1;

    }

    // Detector for the legs
    cv::SimpleBlobDetector::Params legParams;
    if(true){
      // Change thresholds
      legParams.minThreshold = 200;
      legParams.maxThreshold = 255;

      // Filter by Area.
      legParams.filterByArea = true;
      legParams.minArea = 100;
      legParams.maxArea = 1000000;

      // Filter by Circularity
      legParams.filterByCircularity = true;
      legParams.minCircularity = 0.01;
      legParams.maxCircularity = .7;

      // Filter by Convexity
      legParams.filterByConvexity = false;
      legParams.minConvexity = 0.5;
      legParams.maxConvexity = 1;

      // Filter by Inertia
      legParams.filterByInertia = true;
      legParams.minInertiaRatio = 0.1;
      legParams.maxInertiaRatio = .3;

      legParams.thresholdStep = 1;

    }

    // Detector for the head
    cv::SimpleBlobDetector::Params headParams;
    if(true){
      // Change thresholds
      headParams.minThreshold = 210;
      headParams.maxThreshold = 255;

      // Filter by Area.
      headParams.filterByArea = true;
      headParams.minArea = 1000;
      headParams.maxArea = 100000;

      // Filter by Circularity
      headParams.filterByCircularity = false;
      headParams.minCircularity = 0.7;
      headParams.maxCircularity = 1;

      // Filter by Convexity
      headParams.filterByConvexity = false;
      headParams.minConvexity = 0.5;
      headParams.maxConvexity = 1;

      // Filter by Inertia
      headParams.filterByInertia = false;
      headParams.minInertiaRatio = .7;
      headParams.maxInertiaRatio = 1;

      headParams.thresholdStep = 1;

    }

    cv::Ptr<cv::SimpleBlobDetector> torsoDetector = cv::SimpleBlobDetector::create(torsoParams);
    cv::Ptr<cv::SimpleBlobDetector> legDetector = cv::SimpleBlobDetector::create(legParams);
    cv::Ptr<cv::SimpleBlobDetector> headDetector = cv::SimpleBlobDetector::create(headParams);
    std::vector<cv::KeyPoint> keypoints;
    torsoDetector->detect( im, keypoints);
    // headDetector->detect(im, keypoints);
    // legDetector->detect(im, keypoints);

    cv::Mat im_with_keypoints;
    drawKeypoints( im, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // Fill in keypoints with black
    // for (int x = 0; x < keypoints.size(); x++){
    //   cv::circle(im_with_keypoints, (keypoints[x].pt), keypoints[x].size, (0), -1, 8, 0);
    // }

    // imshow("keypoints", im_with_keypoints );

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, im_with_keypoints);
    cv::waitKey(3);

    // Output modified video stream
    blob_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob_detector");

  ros::NodeHandle nh;

  blob_detector detector;

  ros::spin();
  return 0;
}
