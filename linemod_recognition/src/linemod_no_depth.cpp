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
cv::Mat color;

void callback(const sensor_msgs::ImageConstPtr &rgb_image){
  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  color = cv_rgb->image;
}

bool recognizeHighestMatch(int match_num){
  cv::Mat recog_img = color.clone();
  std::vector<linemod_msgs::Scored2DBox> boxes;
  for(int i=0; i<match_num; i++){
    std::vector<cv::Mat> sources;
    sources.push_back(recog_img);
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    linemod_msgs::Scored2DBox box;

    detector->match(sources, (float)matching_threshold,
                    matches,class_ids, quantized_images);
    cv::linemod::Match m = matches[0];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);

    box.width = templates[0].width;
    box.height = templates[0].height;
    box.x = m.x;
    box.y = m.y;
    boxes.push_back(box);

    cv::rectangle(recog_img,
                  cv::Point(m.x,m.y),
                  cv::Point(m.x + templates[0].width, m.y + templates[0].height),
                  cv::Scalar(0,0,0), -1, CV_AA);
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "linemod_no_depth");

  if (argc == 1) {
    detector = cv::linemod::getDefaultLINE();
  } else {
    detector = readLinemod(argv[1]);

    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    printf("Loaded %s with %d classes and %d templates\n",
           argv[1], num_classes, detector->numTemplates());
    if (!ids.empty()) {
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }

  ros::NodeHandle nh;
  ros::Subscriber img_sub_;
  img_sub_ = nh.subscribe("/stereo/left/image_rect_color",1, callback);

  recognizeHighestMatch(10);

  ros::spin();
  return 0;
}
