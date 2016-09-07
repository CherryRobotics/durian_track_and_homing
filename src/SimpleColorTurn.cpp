// TODO: Test latest rendition. Added in Bounding Rect Capability.
// Include Ros Libraries
#include <ros/ros.h>
// Image transport library for subscribing I think...
#include <image_transport/image_transport.h>
// cv_bridge to go from ROSMSG image to CV Image
#include <cv_bridge/cv_bridge.h>
// Sensor_msgs for encoding constants
#include <sensor_msgs/image_encodings.h>
// Geometry_msgs for cmd_vel message to Drone.
#include <geometry_msgs/Twist.h>
// Include custom hsv parameter message
#include <tutorial_ros_opencv/hsvparam.h>
//OpenCV stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

// Store encodings in enc namespace
namespace enc = sensor_msgs::image_encodings;
// yea I know im lazy..
using namespace cv;

// Window name string
static const std::string window = "Processed View";
// Parameter Values
// Hue
int iLowH = 0;
int iHighH = 179;
// Saturation
int iLowS = 0;
int iHighS = 255;
// Value(?)
int iLowV = 0;
int iHighV = 255;

vector<vector<Point> > contours;
vector<Point> points;
//vector<Vec4i> hierarchy;

// Callback for image recieve AKA this is processing algorithm for
// each individual image recieved over ros.
void imageCallback(const sensor_msgs::ImageConstPtr &original_image) {
  // initialize openCV ptr variable
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // What what it seems, it's always good to copy, and return mutatable
    // image... Also, OpenCV expects color images to use BGR8 Encoding..
    cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  }catch (cv_bridge::Exception &e) {
    // If there's an error throw it on ROS_ERROR stream
    ROS_ERROR("tutorial_ros_opencv::SimpleColorTurn.cpp::cv_bridge exception : %s",
              e.what());
  }
  // Since we're looking for a specific color object, we're going to filter
  // the image so only the objects with that color is show.
  Mat imgHSV;
  cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);

  Mat imgThreshold;
  // Threshold the image!
  inRange(imgHSV,
              Scalar(iLowH, iLowS, iLowV),
              Scalar(iHighH, iHighS, iHighV),
              imgThreshold);
  // morphological Opening, This removes small objects and elements from the
  // foreground AKA noise reduction O:
  erode(imgThreshold,
            imgThreshold,
            getStructuringElement(MORPH_ELLIPSE, Size(7,7)));
  dilate(imgThreshold,
             imgThreshold,
             getStructuringElement(MORPH_ELLIPSE, Size(7,7)));

  // Morphological closing, This fills in caps in foreground.
  // "negative space noise" reduction if you will.
  dilate(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(7,7)));
  erode(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(7,7)));

  // After thresholding we should have our isolated color!!!

  // We're not necessarily publishing images back to ROS, instead
  // we're publishing AR.Drone commands.

  // find contours of object.
  findContours(imgThreshold, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // approximate contours to polygons + get bounding rects.
    for (int t = 0; t < contours[0].size(); t++) { // FIX THIS
      for (int i = 0; i < contours.size(); i++) {
      Point p = contours[i][t];
      points.push_back(p);
    }
  }
  if (points.size() > 0) {
    Rect brect = boundingRect(Mat(points).reshape(2));
    rectangle(imgThreshold, brect.tl(), brect.br(), Scalar(100, 100, 100), 2); // TODO: TEST THIS
  }
  // Show image of modified picture
  imshow("SimpleTurn", imgThreshold);
  waitKey(3);
}

void RecieveHSVParameters(const tutorial_ros_opencv::hsvparam &params) {
  // Whenever you recieve an HSV param change, change the filter params
  // on this node. This will only happen during color selection phase at
  // the beginning and shouldn't necessarily happen any more.
  iLowH = params.lowHue;
  iHighH = params.highHue;
  iLowS = params.lowSaturation;
  iHighS = params.highSaturation;
  iLowV = params.lowValue;
  iHighV = params.highValue;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "simplecolorturn");
  ros::NodeHandle nh;
  // Setup image transport to recieve img msgs from ros.
  image_transport::ImageTransport it(nh);
  // subscribe to the images passed by camera.
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, &imageCallback);
  ros::Subscriber subparams = nh.subscribe("mk0/hsvparams", 1000, &RecieveHSVParameters);
  while (ros::ok() ) {
    ros::spinOnce();
  }
}






/// Publish
// linear.x
// linear.y
// linear.z
// angular.x
// angular.y
// angular.z
/// #Publish
