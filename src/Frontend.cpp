/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

Frontend::~Frontend() {
  if(camera_ != nullptr) {
    delete camera_;
  }
}

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  if(camera_ != nullptr) {
    delete camera_;
  }
  camera_ = new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV, arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion>
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{
  // TODO: implement

  const float k1 = -0.541596;
  const float k2 = 0.307486;
  const float p1 = -0.000014;
  const float p2 = 0.001816;
  const float tagSize =16.75;
  const float fu = 569.46;
  const float fv = 572.26;
  const float cu = 320.00;
  const float cv = 149.25;

  // imageWidth,imageHeight,focalLengthU,focalLengthV,imageCenterU,imageCenterV
  //next line is probably not needed as you set the parameters(using setCameraParameters) after calling the Frontend constructor
  //setCameraParameters(640,360,569.46,572.26,320.00,149.25, k1,k2,p1,p2);
  //same case as with the above line: we call it when we construct Frontend
  //setTarget(0, tagSize); // to register 1 target, should be changed if we have more

  //undistort the input image and transform to grayscale
  cv::Mat undistorted_grey_image;
  cv::Mat distorted_grey_image;
  cv::cvtColor(image, distorted_grey_image, CV_BGR2GRAY);
  camera_->undistortImage(distorted_grey_image,undistorted_grey_image);



  //Extract AprilTags ie. get raw detactions using tagDetector_
  std::vector<AprilTags::TagDetection> rawdetections = tagDetector_.extractTags(undistorted_grey_image);


  for (auto const& rawdetection: rawdetections)
  {
      int detected_ID = rawdetection.id;
      if (idToSize_.find(detected_ID) == idToSize_.end())
      {
        // id not found
        break;
      } else {
        // id found
        float targetSize = idToSize_[detected_ID];
        Eigen::Matrix4d transform = rawdetection.getRelativeTransform(targetSize,fu, fv, cu, cv);
        Detection detection; // maybe add arp:: etc.
        detection.T_CT = kinematics::Transformation(transform);
        const std::pair<float, float> *pi = rawdetection.p;
        // std::pair<float, float> p1 = rawdetection.p[1];
        // std::pair<float, float> p3 = rawdetection.p[3];
        // std::pair<float, float> p2 = rawdetection.p[2];
        Eigen::Matrix<double, 2, 4> matrix;
        matrix(0,0) = pi[0].first;
        matrix(0,1) = pi[1].first;
        matrix(0,2) = pi[2].first;
        matrix(0,3) = pi[3].first;
        matrix(1,0) = pi[0].second;
        matrix(1,1) = pi[1].second;
        matrix(1,2) = pi[2].second;
        matrix(1,3) = pi[3].second;
        detection.points = matrix;
        detection.id = detected_ID;
        detections.push_back(detection);
      }
  }
  return detections.size();
}

 /// \brief A simple struct containing all the necessary information about a
  ///        tag detection.
  // struct Detection {
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //   kinematics::Transformation T_CT; ///< The pose of the camera relative to the tag.
  //   Eigen::Matrix<double,2,4> points; ///< The tag corner points in pixels [u1...u4; v1...v4].
  //   int id; ///< The ID of the detected tag.
  // };
  // typedef std::vector<Detection, Eigen::aligned_allocator<Detection>> DetectionVec;


bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp
