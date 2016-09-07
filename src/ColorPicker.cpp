/**
 *  This node is for picking the color filters to be applied in
 *  The other nodes, It's for isolating the color for processing.
 *
 */
// Include Ros Libraries
#include <ros/ros.h>
// Image transport for for recieving and sending ROS imgs
#include <image_transport/image_transport.h>
// cv_bridge to convert between ROSMSG and CV Img.
#include <cv_bridge/cv_bridge.h>
// Sensor_msgs for encoding constants
#include <sensor_msgs/image_encodings.h>
// and finally some open CV stuff.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Include our hsvparam msg to send said parameters over ros.
#include <tutorial_ros_opencv/hsvparam.h>
// store image_encodings in a namespace cause it's easier.
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

// I plan to make this a class anyway so these will probably be
// local private variables, since it feels icky make them global..
// TODO: That^
// Hue...huehuehuehuheuhuehuehueh
int iLowH = 0;
int iHighH = 179;
// Saturation
int iLowS = 0;
int iHighS = 255;
// Value(?)
int iLowV = 0;
int iHighV = 255;
int iEllipseSize = 5;
// Initialize the publisher here so all callbacks have acces to pub.
ros::Publisher pub;

// We'll be subscribing to camera/image_raw topic so we can run our
// filter parameters on, so we'll need a callback for everytime we
// grab a new img from the stream.
void imageCallback(const sensor_msgs::ImageConstPtr &original_image) {
  // Initialize cv_bridge ptr variable
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error in transforming RosIMG to CVImage, Details: %s", e.what());
  }

  // We got the picture now we can start doing processing on the image
  // using the parameters we're grabbing from the ControlView Sliders!

  // Since sensor_msgs doesn't have a conversion to HSV we'll have to use CV to
  // complete it.
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
            getStructuringElement(MORPH_ELLIPSE, Size(iEllipseSize,iEllipseSize)));
  dilate(imgThreshold,
             imgThreshold,
             getStructuringElement(MORPH_ELLIPSE, Size(iEllipseSize,iEllipseSize)));

  // Morphological closing, This fills in caps in foreground.
  // "negative space noise" reduction if you will.
  dilate(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(iEllipseSize,iEllipseSize)));
  erode(imgThreshold,
     imgThreshold,
     getStructuringElement(MORPH_ELLIPSE, Size(iEllipseSize,iEllipseSize)));

  // After thresholding we should have our isolated color!!!

  // Show image of modified picture
  imshow("ControlView", imgThreshold);
  waitKey(3);
  // Since we're not publish anything we can just leave it like this! :D
}

// Callback for Slider changes, Each time a slider is changed Send a ROS message
// including the new parameters...
// I feel like this could bloat the buffer... Trying it anyway. ¯\_(ツ)_/¯
static void trackbarChange(int) {
  // To make it easier for me I just throw all of them into a msg so I don't
  // need to make multiple callbacks.
  tutorial_ros_opencv::hsvparam msg;
  msg.lowHue = iLowH;
  msg.highHue = iHighH;
  msg.lowSaturation = iLowS;
  msg.highSaturation = iHighS;
  msg.lowValue = iLowV;
  msg.highValue = iHighV;
  pub.publish(msg);
}

int main(int argc, char* argv[]) {
  // Initialize ros and nodehandles
  ros::init(argc, argv, "colorpicker");
  ros::NodeHandle nh;
  // Since our publisher is used in callbacks, we want to make sure
  // that's up and ready to go. Publish hsvparams to mk0/hsvparams
  pub = nh.advertise<tutorial_ros_opencv::hsvparam>("mk0/hsvparams", 1000);
  ros::Rate rate(2);
  // Set up image transport so we can recieve images from ROS
  image_transport::ImageTransport it(nh);
  // Setup CV window stuffs
  // Set up sliders for parameter filters
  // Everytime we change a slider, we publish to "cv_encoding/slider/[PARAM]"
  namedWindow("Control", CV_WINDOW_AUTOSIZE);
  namedWindow("ControlView", CV_WINDOW_AUTOSIZE);

  // Push trackbars into namedWindow "Control"
  cvCreateTrackbar("LowH", "Control", &iLowH, 179, trackbarChange); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179, trackbarChange);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255, trackbarChange); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255, trackbarChange);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255, trackbarChange); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255, trackbarChange);
  cvCreateTrackbar("EllipseSize", "Control", &iEllipseSize, 100);

  // Subscribe to image raw to grab images.
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);

  // Highgui call to clean up window on shutdown
  destroyWindow("ControlView");
  //destroyWindow("ControlView");
  // now we just run processing from the above on the incomming ros messages
  // using callbacks.
  // Take it away ros!
  while(ros::ok()) {
    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
   {

   }
    ros::spinOnce();
  }
}
