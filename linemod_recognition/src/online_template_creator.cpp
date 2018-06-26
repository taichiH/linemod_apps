#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <linemod_msgs/Scored2DBoxArray.h>
#include <linemod_msgs/Scored2DBox.h>
#include <std_msgs/Header.h>

cv::Ptr<cv::linemod::Detector> detector;
int num_modalities;
bool create_template = false;
bool do_matching = false;

int threshold;
int match_num;
bool debug_view;
int l_top_x;
int l_top_y;
int r_buttom_x;
int r_buttom_y;

ros::Publisher box_pub_;
linemod_msgs::Scored2DBoxArray box_array_;

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  cv::Mat& dst, cv::Point offset, int T){
  const cv::Scalar color[2] = {CV_RGB(255, 0, 0), CV_RGB(0, 0, 255)};

  cv::rectangle(dst,
		cv::Point(offset.x, offset.y),
		cv::Point(offset.x+templates[0].width,
			  offset.y+templates[0].height),
		color[0], 3);
  cv::circle(dst,
	     cv::Point(offset.x+templates[0].width/2,
		       offset.y+templates[0].height/2),
	     5, color[1], -1);
}

bool onlineMatching(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  std::cout << "server called" << std::endl;
  create_template = true;
  return true;
}

void callback(const sensor_msgs::ImageConstPtr &rgb_image){
  std_msgs::Header header = rgb_image->header;
  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat color = cv_rgb->image;
  cv::Mat display = color.clone();

  std::vector<cv::Mat> sources;
  sources.push_back(color);

  std::string class_id;
  int template_id;

  cv::rectangle(display,
		cv::Point(l_top_x, l_top_y),
		cv::Point(r_buttom_x,
			  r_buttom_y),
		CV_RGB(0, 0, 255), 3);


  if(create_template){
    cv::Mat mask;
    cv::Mat gray = color.clone();
    cv::cvtColor(color, gray , CV_BGR2GRAY);
    cv::Canny(gray, mask, 250, 500);

    cv::rectangle(mask, cv::Point(0,0), cv::Point(color.cols, l_top_y), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,r_buttom_y), cv::Point(color.cols, color.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(0,0), cv::Point(l_top_x, color.rows), cv::Scalar(0,0,0), -1, CV_AA);
    cv::rectangle(mask, cv::Point(r_buttom_x), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);

    cv::imshow("mask", mask);
    cv::waitKey(10);

    class_id = cv::format("ategi");
    cv::Rect bb;
    template_id = detector->addTemplate(sources, class_id, mask, &bb);
    create_template = false;
    do_matching = true;
  }

  if(do_matching){
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> dummy;
    detector->match(sources, (float)threshold, matches, class_ids, dummy);

    if (!matches.empty()){
      cv::linemod::Match m = matches[0];
      const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);

      if(debug_view){
	drawResponse(templates, display, cv::Point(m.x, m.y), detector->getT(0));
	cv::imshow("display", display);
	cv::waitKey(2);
      }

      std::vector<linemod_msgs::Scored2DBox> boxes;
      linemod_msgs::Scored2DBox box;
      std::string name = m.class_id.c_str();
      box.label = name;
      box.width = templates[0].width;
      box.height = templates[0].height;
      box.x = m.x;
      box.y = m.y;
      box.score = m.similarity;
      boxes.push_back(box);

      linemod_msgs::Scored2DBoxArray boxes_msg;
      boxes_msg.header = header;
      boxes_msg.boxes = boxes;
      box_pub_.publish(boxes_msg);
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "online_linemod");
  ros::NodeHandle nh("~");

  detector = cv::linemod::getDefaultLINE();
  num_modalities = (int)detector->getModalities().size();

  nh.getParam("threshold", threshold);
  nh.getParam("match_num", match_num);
  nh.getParam("debug_view", debug_view);
  nh.getParam("l_top_x", l_top_x);
  nh.getParam("l_top_y", l_top_y);
  nh.getParam("r_buttom_x", r_buttom_x);
  nh.getParam("r_buttom_y", r_buttom_y);

  cv::namedWindow("mask", CV_WINDOW_NORMAL);
  cv::resizeWindow("mask", 640, 480);
  if(debug_view){
    cv::namedWindow("display", CV_WINDOW_NORMAL);
    cv::resizeWindow("display", 640, 480);
  }

  ros::Subscriber img_sub_;
  img_sub_ = nh.subscribe("input",1, callback);
  ros::ServiceServer online_linemod_server = nh.advertiseService("create_frag", &onlineMatching);
  box_pub_ = nh.advertise<linemod_msgs::Scored2DBoxArray>("ategi_out", 1);

  ros::spin();
  return 0;
}
