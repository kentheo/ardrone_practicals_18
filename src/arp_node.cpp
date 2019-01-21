#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone EDIT ====" << std::endl;

  double vel_forward = 1.;
  double vel_left = 1.;
  double vel_up = 1.;
  double vel_rotateLeft = 1.;

  double forward = 0.0;
  double left = 0.0;
  double up = 0.0;
  double rotateLeft = 0.0;

  // ros::Rate rate(10);

  while (ros::ok()) {

    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    cv::Mat image;
    if (subscriber.getLastImage(image)) {

      // TODO: add overlays to the cv::Mat image, e.g. text
      cv::putText(image, 
      "forward/backward: UP-arrow/DOWN-arrow\n \
      left/right: LEFT-arrow/RIGHT-arrow\n \
      up/down: W/S\n \
      yaw left/right A/D\n ",
      cvPoint(30,30), 
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
      // http://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      //Convert to SDL_Surface
      IplImage opencvimg2 = (IplImage) image;
      IplImage* opencvimg = &opencvimg2;
      auto frameSurface = SDL_CreateRGBSurfaceFrom(
          (void*) opencvimg->imageData, opencvimg->width, opencvimg->height,
          opencvimg->depth * opencvimg->nChannels, opencvimg->widthStep,
          0xff0000, 0x00ff00, 0x0000ff, 0);
      if (frameSurface == NULL) {
        std::cout << "Couldn't convert Mat to Surface." << std::endl;
      } else {
        texture = SDL_CreateTextureFromSurface(renderer, frameSurface);
        SDL_FreeSurface(frameSurface);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
      }
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // reset velocities to 0 if nothing is pressed
    forward = 0;
    left = 0;
    up = 0;
    rotateLeft = 0;

    // check states!
    auto droneStatus = autopilot.droneStatus();
    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_UP]) {
      std::cout << "Forward..." << std::endl;
      forward = vel_forward;
    }
    if (state[SDL_SCANCODE_DOWN]) {
      std::cout << "Backward..." << std::endl;
      forward = -vel_forward;
    }
    if (state[SDL_SCANCODE_LEFT]) {
      std::cout << "Left..." << std::endl;
      left = vel_left;
    }
    if (state[SDL_SCANCODE_RIGHT]) {
      std::cout << "Right..." << std::endl;
      left = -vel_left;
    }
    if (state[SDL_SCANCODE_W]) {
      std::cout << "Up..." << std::endl;
      up = vel_up;
    }
    if (state[SDL_SCANCODE_S]) {
      std::cout << "Down..." << std::endl;
      up = -vel_up;
    }
    if (state[SDL_SCANCODE_A]) {
      std::cout << "Yaw left..." << std::endl;
      rotateLeft = vel_rotateLeft;
    }
    if (state[SDL_SCANCODE_D]) {
      std::cout << "Yaw right..." << std::endl;
      rotateLeft = -vel_rotateLeft;
    }

    // TODO: process moving commands when in state 3,4, or 7

    // send commands:
    std::cout << "Sending " 
              << " Forward: " << forward 
              << " Left: " << left
              << " Up: " << up
              << " RotateLeft: " << rotateLeft;
    bool success = autopilot.manualMove(forward,left,up,rotateLeft);
    if (success) {
        std::cout << " [ OK ]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
    // bool success = autopilot.manualMove(0,1,0,0);
    std::cout << " ";
  }

  // 2>&1 | tee SomeFile.txt

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
