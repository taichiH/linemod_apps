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

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename){
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold = 80;
ros::Publisher box_pub_;
linemod_msgs::Scored2DBoxArray box_array_;
int match_num = 5;

bool recognizeHighestMatch(int match_num, cv::Mat &recog_img);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  cv::Mat& dst, cv::Point offset, int T);

void callback(const sensor_msgs::ImageConstPtr &rgb_image){
  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat color = cv_rgb->image;
  recognizeHighestMatch(match_num, color);
}

bool recognizeHighestMatch(int match_num, cv::Mat &recog_img){
  cv::Mat image = recog_img.clone();
  cv::Mat display = recog_img.clone();

  std::vector<linemod_msgs::Scored2DBox> boxes;
  for(int i=0; i<match_num; i++){
    std::vector<cv::Mat> sources;
    sources.push_back(image);
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    linemod_msgs::Scored2DBox box;

    detector->match(sources, (float)matching_threshold,
                    matches,class_ids, quantized_images);

    if (matches.size() <= 0)
      continue;

    cv::linemod::Match m = matches[0];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
    
    drawResponse(templates, display, cv::Point(m.x, m.y), detector->getT(0));

    box.width = templates[0].width;
    box.height = templates[0].height;
    box.x = m.x;
    box.y = m.y;
    boxes.push_back(box);
    cv::Point offset(30, 30);

    cv::rectangle(image,
                  cv::Point(m.x + offset.x, m.y + offset.y),
                  cv::Point(m.x + templates[0].width - offset.x, 
			    m.y + templates[0].height - offset.y),
                  cv::Scalar(0,0,0), -1, CV_AA);
  }
  cv::imshow("display", display);
  cv::waitKey(2);
  linemod_msgs::Scored2DBoxArray boxes_msg;
  boxes_msg.boxes = boxes;
  box_pub_.publish(boxes_msg);
}

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

int main(int argc, char** argv){
  ros::init(argc, argv, "linemod_no_depth");
  ros::NodeHandle nh("~");
  std::string template_name;
  nh.getParam("template_name", template_name);
  std::cout << template_name << std::endl;
  detector = readLinemod(template_name);

  std::vector<std::string> ids = detector->classIds();
  num_classes = detector->numClasses();
  printf("Loaded %s with %d classes and %d templates\n",
	 argv[1], num_classes, detector->numTemplates());
  if (!ids.empty()) {
    printf("Class ids:\n");
    std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  }

  ros::Subscriber img_sub_;
  img_sub_ = nh.subscribe("input",1 , callback);
  box_pub_ = nh.advertise<linemod_msgs::Scored2DBoxArray>("box_out", 1);

  ros::spin();
  return 0;
}
