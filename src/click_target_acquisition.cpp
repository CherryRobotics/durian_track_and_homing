/**
 *            Click Target Acquisition
 *  This node is for picking a target for durian.
 *  You will click on the image and it will grab
 *  the HSV ranges for that specific location on the
 *  image and advertise so durian tracking can retrieve it.
 */

// Include ROS Libraries
#include <ros/ros.h>
// Image transport for recieving and sending ROS images.
#include <image_transport/image_transport.h>
// CV_bridge to convert between ROSMSG and CV IMG
#include <cv_bridge/cv_bridge.h>
// Sensor_msgs for encoding constants
#include <sensor_msgs/image_encodings.h>
// and CV related includes.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// custom HSV Param msg to send said parameters over ROS
#include <durian_track_and_homing/hsvparam.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

// Keep these around for local filtering to see target
int iLowH = 0;
int iHighH = 179;
// Saturation
int iLowS = 0;
int iHighS = 255;
// Value(?)
int iLowV = 0;
int iHighV = 255;

Mat imgHSV;
Mat imgHSVcallback;
Mat raw;

// Initialize Publisher in global so all callbacks have access.
ros::Publisher pub;

// Sub to camera/image_raw topic so we can run our
// filter parameters on. We'll need a callback for everytime we recieve
// a new img from stream.
void imageCallback(const sensor_msgs::ImageConstPtr &original_image) {
  // intialize cv_bridge ptr variable
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error in transforming ROSIMG to CVImage, Details: %s", e.what());
  }
  // We have the picture so now we can process the image using click info;
  raw = cv_ptr->image;
  imshow("raw", raw);
  // Convert image to HSV and perform filtering on that with click params.
  Mat imgHSV;
  cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV);


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
            getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
  dilate(imgThreshold,
             imgThreshold,
             getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

  // Morphological closing, This fills in caps in foreground.
  // "negative space noise" reduction if you will.
  dilate(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
  erode(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

  // After thresholding we should have our isolated color!!!


  imshow("Target Isolation", imgThreshold);
  waitKey(3);
  // Since there's no publish we can stop at this.
}

// Click callback
// Everytime a click is made, publish a new msg containing HSV
// data of certain point.
void clickCallback(int event, int x, int y, int, void*) {
  durian_track_and_homing::hsvparam msg;
  int loop;
  Scalar hsvlow(0,0,0), hsvhigh(180,255,255);
  float change;
  if (event == EVENT_LBUTTONDOWN) {
    cvtColor(raw, imgHSVcallback, CV_BGR2HSV);
    Vec3b p = imgHSVcallback.at<Vec3b>(y,x);
    for(loop = 0; loop < 3; loop++) {
      // this sets the accuracy
      change = p[loop]*0.2;
      hsvlow[loop] = p[loop] - change;
      hsvhigh[loop] = p[loop] + change;
    }
    // 100 value for lighting differences?
    // TODO: Look into making sure this is true.
    // hsvlow[2] = 0;
    // hsvhigh[2] = 255;

    iLowH = hsvlow[0];
    iLowS = hsvlow[1];
    iLowV = hsvlow[2];

    iHighH = hsvhigh[0];
    iHighS = hsvhigh[1];
    iHighV = hsvhigh[2];
    // Prep msg for transport.
    msg.lowHue = iLowH;
    msg.highHue = iHighH;
    msg.lowSaturation = iLowS;
    msg.highSaturation = iHighS;
    msg.lowValue = iLowV;
    msg.highValue = iHighV;
    pub.publish(msg);
  }
}

int main(int argc, char* argv[]) {
  // initialize ros and nodehandles
  ros::init(argc, argv, "click_target");
  ros::NodeHandle nh;
  // our publisher is used in a callback with highgui
  // So initialize now to make sure it's ready to use.
  // publish hsvparams to durian/hsvparams
  pub = nh.advertise<durian_track_and_homing::hsvparam>("durian/hsvparams", 1000);
  ros::Rate rate(2);
  // set up image transport so we can recieve images from ROS
  image_transport::ImageTransport it(nh);

  // Subscribe to image raw to grab images.
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);

  // set up OpenCV Windows
  namedWindow("raw", CV_WINDOW_AUTOSIZE);
  namedWindow("Target Isolation", CV_WINDOW_AUTOSIZE);
  // Set the OpenCV mouse callback on the raw window.
  setMouseCallback("raw", clickCallback, 0);
  // We can now run processing on incoming ros MSGS
  // and handle everything using callbacks/CVGui.
  // Take it away Ros!
  while(ros::ok()) {
    if (waitKey(30) == 27) // wait for 'esc' key press for 30ms
    {

    }
    ros::spinOnce();
  }
}
