// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// based on opencv sample
// added ros interfaces

#ifndef LINEMOD_DEVEL_LIB_HH_
#define LINEMOD_DEVEL_LIB_HH_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

namespace linemod {

  class LinemodLib {

  public: LinemodLib(ros::NodeHandle _nh);

  public: ~LinemodLib();

    //image subscribers

  private: ros::Subscriber rgb_sub_;

  private: ros::Subscriber depth_sub_;


    //callback
  private: void imageCb(const sensor_msgs::ImageConstPtr& _rgb_msg);

  private: void depthCb(const sensor_msgs::ImageConstPtr& _depth_msg);

    // Function prototypes
  private: void help();

  private: cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename);

  private: void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename);

  private: void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f);

  private: void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f);

  private: void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);

  private: std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                                 int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst);

  private: void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst);

  private: void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

  private: cv::Mat displayQuantized(const cv::Mat& quantized);

  private: ros::NodeHandle nh_;

  private: cv::Mat color_;

  private: cv::Mat depth_;

  };

  typedef std::shared_ptr<LinemodLib> LinemodLibPtr;
}

// Copy of cv_mouse from cv_utilities
class Mouse
{
public:
  static void start(const std::string& a_img_name)
    {
      cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
    }
  static int event(void)
    {
      int l_event = m_event;
      m_event = -1;
      return l_event;
    }
  static int x(void)
    {
      return m_x;
    }
  static int y(void)
    {
      return m_y;
    }

private:
  static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *)
    {
      m_event = a_event;
      m_x = a_x;
      m_y = a_y;
    }

  static int m_event;
  static int m_x;
  static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;


class Timer
{
public:
  Timer() : start_(0), time_(0) {}

  void start()
    {
      start_ = cv::getTickCount();
    }

  void stop()
    {
      CV_Assert(start_ != 0);
      int64 end = cv::getTickCount();
      time_ += end - start_;
      start_ = 0;
    }

  double time()
    {
      double ret = time_ / cv::getTickFrequency();
      time_ = 0;
      return ret;
    }

private:
  int64 start_, time_;
};

#endif
