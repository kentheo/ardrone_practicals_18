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
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/CameraBase.hpp>
#include <arp/cameras/DistortionBase.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <arp/ViEkf.hpp>
#include <arp/StatePublisher.hpp>
//#include <arp/InteractiveMarkerServer.hpp>




// notes:
// cd ~/ardrone_ws
// catkin_make -DCMAKE_BUILD_TYPE=Release
// roslaunch ardrone_practicals arp.launch
// roslaunch ardrone_practicals arp_rviz.launch
// roslaunch ardrone_practicals arp_check_control.launch
// 2>&1 | tee SomeFile.txt

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  arp::VisualInertialTracker *visualTrackerPtr;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;

    visualTrackerPtr->addImage(timeMicroseconds, lastImage_);
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
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;

    Eigen::Vector3d omega_S;
    omega_S[0] = msg-> angular_velocity.x;
    omega_S[1] = msg-> angular_velocity.y;
    omega_S[2] = msg-> angular_velocity.z;
    Eigen::Vector3d acc_S;
    acc_S[0] = msg-> linear_acceleration.x;
    acc_S[1] = msg-> linear_acceleration.y;
    acc_S[2] = msg-> linear_acceleration.z;

    visualTrackerPtr->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
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

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup inputs
  Subscriber subscriber;

  // Set up State publisher
  arp::StatePublisher posePublisher(nh);

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

  // should not be greater than 1:
  const double vel_forward = 1.;
  const double vel_left = 1.;
  const double vel_up = 1.;
  const double vel_rotateLeft = 1.;

  double forward = 0.0;
  double left = 0.0;
  double up = 0.0;
  double rotateLeft = 0.0;

  // Retreive the values directly from the arp_rviz lauch file.
  double k1; double k2; double p1; double p2;
  double fu; double fv; double cu; double cv;
  const double tagSize = 0.1675;
  const int imageWidth = 640;
  const int imageHeight = 360;

  // Boolean for Challenge
  bool flyChallenge = false;

  // Boolean to check if Challenge is done
  bool finishedChallenge = false;
  bool wayback = false;

  nh.getParam("/arp_node/k1", k1);
  nh.getParam("/arp_node/k2", k2);
  nh.getParam("/arp_node/p1", p1);
  nh.getParam("/arp_node/p2", p2);
  nh.getParam("/arp_node/fu", fu);
  nh.getParam("/arp_node/fv", fv);
  nh.getParam("/arp_node/cu", cu);
  nh.getParam("/arp_node/cv", cv);

  printf("Params received:\nk1: %f, k2: %f, p1: %f, p2: %f, fu: %f, fv: %f, cu: %f, cv: %f\n",
  k1,k2,p1,p2,fu,fv,cu,cv);

  // const float k1 = -0.541596;
  // const float k2 = 0.307486;
  // const float p1 = -0.000014;
  // const float p2 = 0.001816;
  // const float tagSize =16.75;
  // const float fu = 569.46;
  // const float fv = 572.26;
  // const float cu = 320.00;
  // const float cv = 149.25;

  arp::cameras::RadialTangentialDistortion radDist(k1, k2, p1, p2);

  // imageWidth,imageHeight,focalLengthU,focalLengthV,imageCenterU,imageCenterV
  //TODO: find the correct parameters
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinCam(imageWidth, imageHeight, fu, fv, cu, cv, radDist);
  //without this image is not rendered
  pinCam.initialiseUndistortMaps();

  // ros::Rate rate(10);

  // CW 4 InteractiveMarkerServer
  //commented for challange task
  //arp::InteractiveMarkerServer marker_server(autopilot);
  //double x, y, z, yaw;
  //autopilot.getPoseReference(x, y, z, yaw);
  //marker_server.activate(x, y, z, yaw);

  arp::Frontend frontend;
  frontend.setCameraParameters(imageWidth,imageHeight,fu,fv,cu,cv,k1,k2,p1,p2);
  // set frontend targets
  for(size_t i=0; i<=8; ++i) {
    frontend.setTarget(i,0.158);
  }

  for(size_t i=10; i<=18; ++i) {
    frontend.setTarget(i,0.158);
  }

  //commented for final Challenge
  // frontend.setTarget(0, tagSize);

  //Visual Inertial Tracker Integration
  arp::VisualInertialTracker visualTracker; //= arp::VisualInertialTracker();
  arp::ViEkf viefk;

  //pass the Frontend as well as a ViEkf to the vis-inert tracker
  visualTracker.setFrontend(frontend);
  visualTracker.setEstimator(viefk);
  subscriber.visualTrackerPtr = &visualTracker;

  //below removed for Challenge task
  // // register the AprilTag
  // Eigen::Matrix<double,4,4> trs = Eigen::Matrix<double,4,4>::Identity();
  // trs << 1, 0, 0 ,0,
  //        0, 0, -1, 0,
  //        0, 1, 0, 0,
  //        0, 0, 0, 1;
  //
  // arp::kinematics::Transformation transform(trs);
  // viefk.setTarget(0,transform,tagSize);

  // init estimator targets
    Eigen::Matrix3d C_WT;
    C_WT << 0.000000000000000, 0.000000000000000, 1.000000000000000,
            1.000000000000000, 0.000000000000000, 0.000000000000000,
            0.000000000000000, 1.000000000000000, 0.000000000000000;
    const double target_size = 0.1725;
    Eigen::Quaterniond q_WT(C_WT);
    typedef arp::kinematics::Transformation tf_t;
    typedef Eigen::Vector3d vec3_t;
    tf_t T_WT0{vec3_t{0.0, 2.221, 1.653}, q_WT};
    viefk.setTarget(0, T_WT0, target_size);
    tf_t T_WT1{vec3_t{0.0, 2.994, 1.648}, q_WT};
    viefk.setTarget(1, T_WT1, target_size);
    tf_t T_WT2{vec3_t{0.0, 3.224, 1.649}, q_WT};
    viefk.setTarget(2, T_WT2, target_size);
    tf_t T_WT3{vec3_t{0.0, 3.997, 1.654}, q_WT};
    viefk.setTarget(3, T_WT3, target_size);
    tf_t T_WT4{vec3_t{0.0, 4.225, 1.657}, q_WT};
    viefk.setTarget(4, T_WT4, target_size);
    tf_t T_WT5{vec3_t{0.0, 5.000, 1.654}, q_WT};
    viefk.setTarget(5, T_WT5, target_size);
    tf_t T_WT6{vec3_t{0.0, 5.658, 1.650}, q_WT};
    viefk.setTarget(6, T_WT6, target_size);
    tf_t T_WT7{vec3_t{0.0, 6.418, 1.650}, q_WT};
    viefk.setTarget(7, T_WT7, target_size);
    tf_t T_WT8{vec3_t{0.0, 7.194, 1.649}, q_WT};
    viefk.setTarget(8, T_WT8, target_size);
    tf_t T_WT10{vec3_t{0.0, 2.218, 0.952}, q_WT};
    viefk.setTarget(10, T_WT10, target_size);
    tf_t T_WT11{vec3_t{0.0, 2.993, 0.952}, q_WT};
    viefk.setTarget(11, T_WT11, target_size);
    tf_t T_WT12{vec3_t{0.0, 3.222, 0.952}, q_WT};
    viefk.setTarget(12, T_WT12, target_size);
    tf_t T_WT13{vec3_t{0.0, 3.995, 0.956}, q_WT};
    viefk.setTarget(13, T_WT13, target_size);
    tf_t T_WT14{vec3_t{0.0, 4.225, 0.956}, q_WT};
    viefk.setTarget(14, T_WT14, target_size);
    tf_t T_WT15{vec3_t{0.0, 4.997, 0.951}, q_WT};
    viefk.setTarget(15, T_WT15, target_size);
    tf_t T_WT16{vec3_t{0.0, 5.653, 0.950}, q_WT};
    viefk.setTarget(16, T_WT16, target_size);
    tf_t T_WT17{vec3_t{0.0, 6.412, 0.947}, q_WT};
    viefk.setTarget(17, T_WT17, target_size);
    tf_t T_WT18{vec3_t{0.0, 7.189, 0.942}, q_WT};
    viefk.setTarget(18, T_WT18, target_size);

  // Set the waypoints for the CHALLENGE
  std::deque<arp::Autopilot::Waypoint> waypoints_forth;
  std::deque<arp::Autopilot::Waypoint> waypoints_back;
  // Start point
  arp::Autopilot::Waypoint pointA;
  pointA.x = 2.8;
  pointA.y = 2.2;
  pointA.z = 1.4;
  pointA.yaw = 2.7;
  pointA.posTolerance = 0.1;
  // Intermediate point 1
  arp::Autopilot::Waypoint pointC;
  pointC.x = 1.8;
  pointC.y = 2.2;
  pointC.z = 1.4;
  pointC.yaw = 2.7;
  pointC.posTolerance = 0.1;
  // Intermediate point 2
  arp::Autopilot::Waypoint pointD;
  pointD.x = 1.8;
  pointD.y = 4.2;
  pointD.z = 1.4;
  pointD.yaw = 2.7;
  pointD.posTolerance = 0.1;
  // Intermediate point 3
  arp::Autopilot::Waypoint pointE;
  pointE.x = 1.8;
  pointE.y = 6.5;
  pointE.z = 1.4;
  pointE.yaw = 2.7;
  pointE.posTolerance = 0.1;
  // Delivery point
  arp::Autopilot::Waypoint pointB;
  pointB.x = 2.8;
  pointB.y = 6.5;
  pointB.z = 1.4;
  pointB.yaw = 2.7;
  pointB.posTolerance = 0.1;
  // Add all the above points to the deque
  waypoints_forth.push_back(pointA);  // start
  waypoints_forth.push_back(pointC);
  waypoints_forth.push_back(pointD);
  waypoints_forth.push_back(pointE);
  waypoints_forth.push_back(pointB);  // delivery

  waypoints_back.push_front(pointA);  // deliver
  waypoints_back.push_front(pointC);
  waypoints_back.push_front(pointD);
  waypoints_back.push_front(pointE);
  waypoints_back.push_front(pointB);  // back


  //set camera Extrinsics
  Eigen::Matrix4d T_SC_mat;
  T_SC_mat << -0.00195087, -0.03257782, 0.99946730, 0.17409445,
  -0.99962338, -0.02729525, -0.00284087, 0.02255834,
  0.02737326, -0.99909642, -0.03251230, 0.00174723,
  0.00000000, 0.00000000, 0.00000000, 1.00000000;
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viefk.setCameraExtrinsics(T_SC);

  //get undistorded camera model from frontend
  //and use it together with the estiator
  arp::cameras::PinholeCamera<arp::cameras::NoDistortion> undistCam =
      frontend.undistortedCameraModel();
  viefk.setCameraIntrinsics(undistCam); //sets the camera parameters, eg. model, properties

  // Call autopilot after computing the state ???????
  visualTracker.setControllerCallback(std::bind(&arp::Autopilot::controllerCallback, &autopilot,
  std::placeholders::_1, std::placeholders::_2));

  visualTracker.setVisualisationCallback(std::bind(&arp::StatePublisher::publish,
    &posePublisher, std::placeholders::_1, std::placeholders::_2));

  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  while (ros::ok()) {

    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // check states!640, 360, 350, 360, 320, 130,
    auto droneStatus = autopilot.droneStatus();

    // Check if we are doings tasks for the challenge and that the drone is in auto status
    if (flyChallenge && autopilot.isAutomatic() && autopilot.waypointsLeft() == 0) {
      bool success = autopilot.land();
      if (success) {
        std::cout << " Should be landing right now! " << std::endl;
        if (autopilot.droneStatus() == arp::Autopilot::Landed && !wayback) {
          autopilot.flyPath(waypoints_back);
          wayback = true;
          // Take off now
          bool success_t = autopilot.takeoff();
          if (success_t) {
            std::cout << " Should be taking off right now! " << std::endl;
          }

        }
        if (autopilot.droneStatus() == arp::Autopilot::Landed && wayback) {
          std::cout << " Challenge done " << std::endl;
          flyChallenge = false;
          autopilot.setManual();

        }
      }




    }

    // render image, if there is a new one available
    cv::Mat image;
    cv::Mat image2;
    if (subscriber.getLastImage(image)) {

      // Undistort image
      pinCam.undistortImage(image, image);

      //Task4 week2
      //print tags
      //std::vector<arp::Frontend::Detection, Eigen::aligned_allocator<arp::Frontend::Detection>> detections;
      //frontend.detect(image, detections);

      //for(arp::Frontend::Detection det: detections){
        //publish something
      // std::cout << "DETECTED";
      //  autopilot.publishTag(det);
      //}
      //image = image2;

      // TODO: add overlays to the cv::Mat image, e.g. text
      cv::putText(image,
      "takoff/land: T/L, stop: ESC, forward/backward: UP/DOWN, left/right: LEFT/RIGHT, up/down: W/S, yaw left/right: A/D.",
      cvPoint(10,10),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, cvScalar(0,0,0), 1, CV_AA);

      float batteryStatus = autopilot.batteryStatus();
      std::ostringstream batteryString;
      batteryString << "Battery: " << batteryStatus << "%";
      cv::putText(image, batteryString.str().c_str(),
      cvPoint(10,30),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0,255,0), 1, CV_AA);
      cv::String droneStatusString;
      switch(droneStatus)
      {
        case 0: droneStatusString = "[Unknown]"; break;
        case 1: droneStatusString = "[Inited]"; break;
        case 2: droneStatusString = "[Landed]"; break;
        case 3: droneStatusString = "[Flying]"; break;
        case 4: droneStatusString = "[Hovering]"; break;
        case 5: droneStatusString = "[Test]"; break;
        case 6: droneStatusString = "[TakingOff]"; break;
        case 7: droneStatusString = "[Flying2]"; break;
        case 8: droneStatusString = "[Landing]"; break;
        case 9: droneStatusString = "[Looping]"; break;
      }
      cv::putText(image, droneStatusString,
      cvPoint(10,40),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0,0,255), 1, CV_AA);


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

    // NEW CW5 : automatic flight
    //  right Ctrl (SDL_SCANCODE_RCTRL) key to enable automatic control
    if (state[SDL_SCANCODE_RCTRL]) {
      std::cout << "Going to automatic control..." << std::endl;
      autopilot.setAutomatic();
    }

    // spacebar (SDL_SCANCODE_SPACE) to go back to manual
    if (state[SDL_SCANCODE_SPACE]) {
      std::cout << "Going back to manual mode ..." << std::endl;
      // Addition for CHALLENGE
      flyChallenge = false;
      autopilot.setManual();
    }

    // CHALLENGE TASK: press P-key
    if (state[SDL_SCANCODE_P]) {
      if (droneStatus == arp::Autopilot::Flying || droneStatus == arp::Autopilot::Hovering ||
                  droneStatus == arp::Autopilot::Flying2) {
        std::cout << "Startin Amazin Challenge ..." << std::endl;
        flyChallenge = true;
        autopilot.setAutomatic();

        // Set the waypoints to the autopilot
        // The autopilot callback will automatically detect the list is not
        // empty, and start the challenge
        autopilot.flyPath(waypoints_forth);

    // TODO: check more conditions, do stuff
    }

    }

    // Check if we are in manual mode before accepting control commands
    if (!autopilot.isAutomatic())
    {
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
      //send the move command to drone
      bool success = autopilot.manualMove(forward,left,up,rotateLeft);

      if (success) {
          std::cout << " [ OK ]" << std::endl;
      } else {
          std::cout << " [FAIL]" << std::endl;
      }

    }




  }


  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
