// Ros library
#include <ros/ros.h>
// Image_transport library
#include <image_transport/image_transport.h>
// cv_bridge to go from ros imgs to OpenCV imgs
#include <cv_bridge/cv_bridge.h>
// sensor_msgs for sensor constants
#include <sensor_msgs/image_encodings.h>
// headers for OpenCV2 image processing
#include <opencv2/imgproc/imgproc.hpp>
// headers for openCv2 GUI
#include <opencv2/highgui/highgui.hpp>

// store all constants for image encodings in the enc namespace to be used later
namespace enc = sensor_msgs::image_encodings;

// String for window for OpenCV
static const char WINDOW[] = "Image Processed";

// use method of image_transport to create image publisher
image_transport::Publisher pub;

// Callback everytime an image is published.
void imageCallback(const sensor_msgs::ImageConstPtr& original_image) {
  // convert from ros image message to a CvImage suitable for working with
  // OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Always copy, returning a mutatable CvImage
    //OpenCV expects, color images to use BGR channel order
    cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  } catch (cv_bridge::Exception &e) {
    // if there is an error during conversion display it on ros_error
    ROS_ERROR("tutorial_ros_opencv::main.cpp::cv_bridge exception: %s", e.what());
  }
  // Invert the image.
  // Go through all the rows.
  for (int i = 0; i < cv_ptr->image.rows; i++) {

    // Go through all the columns
    for (int j = 0; j < cv_ptr->image.cols; j++) {

      // go through all the channels
      for (int k = 0; k < cv_ptr->image.channels(); k++) {

        // Invert the image by subtracting image data from 255
        cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
      }
    }
  }

  // Display the image using OpenCV
  cv::imshow(WINDOW, cv_ptr->image);
  // Add some delay in miliseconds. this function only works if there is at
  // least one highgui window created and the window is active.
  // if there are several highgui windows, any of them can be active.
  cv::waitKey(3);
  /**
   * The publish() function below is how you send messages. The parameter
   * is the message object. The type of this object must agree with the
   * type given as a template parameter to the advertise<>() call, as was
   * done in the constructor in main().
   */
   // Convert the CvImage to a ROS Image message and publish it on
   // "camera/image_processed" topic
   pub.publish(cv_ptr->toImageMsg());
}
// Demonstrates a simple image conversion from ros image message to openCV
// formats and image processing.
int main(int argc, char* argv[]) {
  // ros init to start the ros to add to ros system.
  ros::init(argc, argv, "image_processor");
  // ros nodehandle to handle comms between rosframework and current class.
  ros::NodeHandle nh;
  // create an ImageTransport instance initializing it through nh.
  image_transport::ImageTransport it(nh);
  // OpenCV Highgui call to open a window on startup.
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  /**
   * Subscribe to camera/image_raw topic. Actual ros topic depends on publisher
   * but I'm sure I can figure that out later.
   * note: normally raw transport is just camera/image_raw with type
   * sensor_msgs/Image.
   * ROS will call the imageCallback function every time a new image arrives.
   * second arguement is queue size, this is like how many messages can be pushed
   * but not necessarily popped yet.. kinda.
   * Subscribe() returns an image_transport::Subscriber object. you must hold
   * onto this until you wish to unsubscribe. When the Subscriber object is
   * destroyed, it will automatically unsubscribe from camera/image_raw topic.
   * but this seems like it's bad ROS ettique.
   */
   image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
   // OpenCV Highgui call to destroy a display window on shutdown.
   cv::destroyWindow(WINDOW);
   /**
    * Alternatively to subscribe, is advertise.
    * This is how you publish on a given topic.
    * The first argument is topic you wish to advertise/publish on
    * second argument is message queue size.
    * In other words how many messages can be placed in the buffer to send before
    * throwing some away incase we publish faster than we can send.
    */
    pub = it.advertise("camera/image_processed", 1);
    // ros.spin() to have ros take over!
    ros::spin();
}
