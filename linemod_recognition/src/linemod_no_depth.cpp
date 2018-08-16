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
#include <linemod_msgs/Scored2DBoxArray.h>
#include <linemod_msgs/Scored2DBox.h>
#include <std_msgs/Header.h>
#include <cmath>

std::vector<cv::Ptr<cv::linemod::Detector>> detector_vec;
std::string filename;
int template_num = 3;
int num_classes = 0;
int threshold;
int match_num;
bool debug_view;

ros::Publisher box_pub_;
ros::Publisher img_pub_;
linemod_msgs::Scored2DBoxArray box_array_;

bool recognizeHighestMatch(int match_num, cv::Mat &recog_img, std_msgs::Header &header);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  cv::Mat& dst, cv::Point offset, int T);
void drawResponse2(const std::vector<cv::linemod::Template>& templates,
                   cv::Mat& dst, cv::Point offset, int T);
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename);
bool splitClassId(std::string &name, std::string &result);

std::string updatePartsName(cv::linemod::Match m){
  std::string parts_name = m.class_id.c_str();
  char split = ',';
  std::string parts_result;
  auto first = parts_name.begin();
  while(first != parts_name.end()){
    auto last = first;
    while( last != parts_name.end() && *last != split)
      ++last;
    parts_result = std::string(parts_name.begin(), first);
    if (last != parts_name.end())
      ++last;
    first = last;
  }
  return parts_result;
}

int updateDeg(cv::linemod::Match m){
  std::string parts_name = m.class_id.c_str();
  char split = ',';
  std::string parts_result;
  int deg;
  auto first = parts_name.begin();
  while(first != parts_name.end()){
    auto last = first;
    while( last != parts_name.end() && *last != split)
      ++last;
    parts_result = std::string(first, last);
    if (last != parts_name.end())
      ++last;
    first = last;
  }
  deg = std::stoi(parts_result);
  return deg;
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
  recognizeHighestMatch(match_num, color, header);
}

bool recognizeHighestMatch(int match_num, cv::Mat &recog_img, std_msgs::Header &header){
  cv::Mat image = recog_img.clone();
  cv::Mat display = recog_img.clone();
  cv::Mat masked_image = recog_img.clone();
  cv::rectangle(masked_image,
                cv::Point(0,0),
                cv::Point(masked_image.cols * 0.33, masked_image.rows),
                cv::Scalar(0,0,0),
                -1,
                CV_AA);
  cv::rectangle(masked_image,
                cv::Point(masked_image.cols * 0.66, 0),
                cv::Point(masked_image.cols, masked_image.rows),
                cv::Scalar(0,0,0),
                -1,
                CV_AA);

  std::vector<linemod_msgs::Scored2DBox> boxes;
  for(int i=0; i<template_num; i++){
    // image = masked_image;
    image = recog_img.clone();

    for(int j=0; j<match_num; j++){
      std::vector<cv::Mat> sources;
      sources.push_back(image);
      std::vector<cv::linemod::Match> matches;
      std::vector<std::string> class_ids;
      std::vector<cv::Mat> quantized_images;
      linemod_msgs::Scored2DBox box;

      detector_vec.at(i)->match(sources, (float)threshold,
                      matches,class_ids, quantized_images);

      if (matches.size() <= 0)
        continue;

      cv::linemod::Match m = matches[0];
      const std::vector<cv::linemod::Template>& templates = detector_vec.at(i)->getTemplates(m.class_id, m.template_id);
    

      std::string name = m.class_id.c_str();
      std::string result;
      splitClassId(name, result);

      int deg = updateDeg(m);

      int similarity = (int)m.similarity; 
      std::string result_text = std::string(std::to_string(similarity))+"%"+" "+(std::to_string(deg))+"deg";
      std::string parts_name = updatePartsName(m);
      parts_name.pop_back();

      if(i >= 0){
        drawResponse2(templates,
                      display,
                      cv::Point(m.x, m.y),
                      detector_vec.at(i)->getT(0));
        
        cv::Point pt1 = cv::Point(m.x+templates[0].width/2,
                                  m.y+templates[0].height/2);
        float len = 50;
        float x2 = pt1.x + len * std::cos(deg * M_PI / 180);
        float y2 = pt1.y + len * std::sin(deg * M_PI / 180);
        cv::Point pt2 = cv::Point(x2, y2);
        cv::line(display, pt1, pt2, CV_RGB(255, 0, 0), 10, 4);

        float x3 = pt1.x + len * std::cos((deg + 90) * M_PI / 180);
        float y3 = pt1.y + len * std::sin((deg + 90) * M_PI / 180);
        cv::Point pt3 = cv::Point(x3, y3);
        cv::line(display, pt1, pt3, CV_RGB(0, 0, 255), 10, 4);


        cv::putText(display, result_text,
                    cv::Point2i(m.x, m.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0,0, 0),
                    2);
        cv::putText(display, parts_name,
                    cv::Point2i(m.x, m.y - 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 0),
                    2);
      }
    

      box.label = result;
      box.width = templates[0].width;
      box.height = templates[0].height;
      box.x = m.x;
      box.y = m.y;
      box.deg = deg;
      box.score = m.similarity;
      boxes.push_back(box);
      cv::Point offset(0, 30);

      cv::rectangle(image,
                    cv::Point(m.x + offset.x, m.y + offset.y),
                    cv::Point(m.x + templates[0].width - offset.x, 
                              m.y + templates[0].height - offset.y),
                    cv::Scalar(0,0,0), -1, CV_AA);

    }
  }

  if(debug_view){
    cv::imshow("display", display);
    cv::waitKey(2);
  }

  linemod_msgs::Scored2DBoxArray boxes_msg;
  boxes_msg.header = header;
  boxes_msg.boxes = boxes;
  box_pub_.publish(boxes_msg);

  img_pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display).toImageMsg());
}

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename){
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  cv::Mat& dst, cv::Point offset, int T){
  const cv::Scalar color[3] = {CV_RGB(255, 0, 0), CV_RGB(0, 0, 255), CV_RGB(0, 255, 0)};

  // cv::rectangle(dst,
  //       	cv::Point(offset.x, offset.y),
  //       	cv::Point(offset.x+templates[0].width,
  //       		  offset.y+templates[0].height),
  //       	color[0], 5);
  cv::circle(dst,
	     cv::Point(offset.x+templates[0].width/2,
		       offset.y+templates[0].height/2),
	     5, color[1], -1);
}

void drawResponse2(const std::vector<cv::linemod::Template>& templates,
                  cv::Mat& dst, cv::Point offset, int T){
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 255, 0),
                                        CV_RGB(0, 0, 255),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  cv::Scalar color = COLORS[0];
  for (int i = 0; i < (int)templates[0].features.size(); ++i) {
    cv::rectangle(dst, cv::Point(offset.x, offset.y), cv::Point(offset.x+templates[0].width, offset.y+templates[0].height), CV_RGB(255,0,0), 3);
    cv::linemod::Feature f = templates[0].features[i];
    cv::Point pt(f.x + offset.x, f.y + offset.y);
    cv::circle(dst, pt, 3, color);
    cv::circle(dst, cv::Point(offset.x+templates[0].width/2, offset.y+templates[0].height/2), 8, cv::Scalar(0,0,255), -1);
  }

}


bool splitClassId(std::string &name, std::string &result){
  char split = ',';
  auto first = name.begin();
  while(first != name.end()){
    auto last = first;
    while( last != name.end() && *last != split)
      ++last;
    result = std::string(name.begin(), first);
    if (last != name.end())
      ++last;
    first = last;
  }
  result.pop_back();
  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "linemod_no_depth");
  ros::NodeHandle nh("~");

  std::vector<std::string> template_vec(template_num);
  std::string template_name;
  nh.getParam("pie", template_vec.at(0));
  nh.getParam("juice", template_vec.at(1));
  nh.getParam("caffelate", template_vec.at(2));
  nh.getParam("threshold", threshold);
  nh.getParam("match_num", match_num);
  nh.getParam("debug_view", debug_view);

  for(int i=0; i<template_vec.size(); i++){
    cv::Ptr<cv::linemod::Detector> detector = readLinemod(template_vec.at(i));
    detector_vec.push_back(detector);
    std::vector<std::string> ids = detector_vec.at(i)->classIds();
    num_classes = detector_vec.at(i)->numClasses();
    printf("Loaded %s with %d classes and %d templates\n",
           argv[1], num_classes, detector_vec.at(i)->numTemplates());
    if (!ids.empty()) {
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }
  std::cerr << "detector_vec.size(): " << detector_vec.size() << std::endl;

  ros::Subscriber img_sub_;
  img_sub_ = nh.subscribe("input",1 , callback);
  box_pub_ = nh.advertise<linemod_msgs::Scored2DBoxArray>("box_out", 1);
  img_pub_ = nh.advertise<sensor_msgs::Image>("output_image", 1);
  ros::spin();
  return 0;
}
