#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

// linemod has been moved to opencv_contrib
// http://code.opencv.org/projects/opencv/repository/revisions/1ad9827fc4ede1b9c42515569fcc5d8d1106a4ea
#if CV_MAJOR_VERSION < 3

// Function prototypes
std::string updatePartsName(cv::linemod::Match m);
int updateDeg(cv::linemod::Match m);

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

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

static void help()
{
  printf("Usage: openni_demo [templates.yml]\n\n"
         "Place your object on a planar, featureless surface. With the mouse,\n"
         "frame it in the 'color' window and right click to learn a first template.\n"
         "Then press 'l' to enter online learning mode, and move the camera around.\n"
         "When the match score falls between 90-95%% the demo will add a new template.\n\n"
         "Keys:\n"
         "\t h   -- This help page\n"
         "\t l   -- Toggle online learning\n"
         "\t m   -- Toggle printing match result\n"
         "\t t   -- Toggle printing timings\n"
         "\t w   -- Write learned templates to disk\n"
         "\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
         "\t q   -- Quit\n\n");
}

// Adapted from cv_timer in cv_utilities
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

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<std::string> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

// global
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold = 80;
std::string template_path = "/home/higashide/ais-project/catkin_ws/src/devel/higashide/higa_linemod_pkg/";
ros::Publisher obj_pose_pub_;
geometry_msgs::Pose2D obj_pose_;

void callback(const sensor_msgs::ImageConstPtr &rgb_image)
{
  bool show_match_result = true;
  bool show_timings = false;
  bool learn_online = false;  
  //hogehoge
  obj_pose_.x = 0.0;
  obj_pose_.y = 0.0; 
  obj_pose_.theta = 0.0;
  /// @todo Keys for changing these?
  cv::Size roi_size(450, 300);
  // cv::Size roi_size(350,350);
  int learning_lower_bound = 90;
  int learning_upper_bound = 95;
  // Timers
  Timer extract_timer;
  Timer match_timer;
  int num_modalities = (int)detector->getModalities().size();

  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat color = cv_rgb->image;
  std::vector<cv::Mat> sources;
  sources.push_back(color);
  cv::Mat display = color.clone();
  cv::Mat hogehoge = color.clone();
  cv::rectangle(hogehoge, cv::Point(0,0), cv::Point(color.cols,Mouse::y()-roi_size.height/2), cv::Scalar(0,0,0), -1, CV_AA);
  cv::rectangle(hogehoge, cv::Point(0,Mouse::y()+roi_size.height/2), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
  cv::rectangle(hogehoge, cv::Point(0,0), cv::Point(Mouse::x()-roi_size.width/2,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
  cv::rectangle(hogehoge, cv::Point(Mouse::x()+roi_size.width/2,0), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
  cv::imshow("hogehoge", hogehoge);

  if (!learn_online)
  {
    cv::Point mouse(Mouse::x(), Mouse::y());
    int event = Mouse::event();

    // Compute ROI centered on current mouse location
    cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
    cv::Point pt1 = mouse - roi_offset; // top left
    cv::Point pt2 = mouse + roi_offset; // bottom right
    
    std::cout << color.rows << ", " << color.cols << std::endl;

    if (event == CV_EVENT_RBUTTONDOWN)
    {
      // Compute object mask by subtracting the plane within the ROI
      std::vector<CvPoint> chain(4);
      chain[0] = pt1;
      chain[1] = cv::Point(pt2.x, pt1.y);
      chain[2] = pt2;
      chain[3] = cv::Point(pt1.x, pt2.y);
      cv::Mat mask;
      cv::Mat gray = color.clone();
      cv::cvtColor(color, gray , CV_BGR2GRAY);
      cv::Canny(gray, mask, 200, 400);
      // cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY | cv::THRESH_OTSU);
      // cv::threshold(gray, mask, 100, 255, CV_THRESH_BINARY);
      cv::rectangle(mask, cv::Point(0,0), cv::Point(color.cols,Mouse::y()-roi_size.height/2), cv::Scalar(0,0,0), -1, CV_AA);
      cv::rectangle(mask, cv::Point(0,Mouse::y()+roi_size.height/2), cv::Point(color.cols, color.rows), cv::Scalar(0,0,0), -1, CV_AA);
      cv::rectangle(mask, cv::Point(0,0), cv::Point(Mouse::x()-roi_size.width/2,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
      cv::rectangle(mask, cv::Point(Mouse::x()+roi_size.width/2,0), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);

      cv::Mat tmp_color = color.clone();
      std::vector<cv::Mat> tmp_source;
      tmp_source.push_back(tmp_color);

      char parts_name[50];
      printf("input parts name -> \n");
      scanf("%s",parts_name);
      
      for (int i=0; i<1; i++){
	// Extract template
	int deg = i*2;
	std::string class_id = cv::format("%s,%d", parts_name, deg);
	cv::Rect bb;
	extract_timer.start();
	int template_id = detector->addTemplate(tmp_source, class_id, mask, &bb);
	extract_timer.stop();

	if (template_id != -1){
	  printf("*** Added template (id %d) for new object class %d***\n", template_id, num_classes);
	}
	float angle = 2;
	float scale = 1;
	const cv::Mat affine_matrix = cv::getRotationMatrix2D(mouse, angle, scale );
	cv::warpAffine(mask, mask, affine_matrix, mask.size());
	cv::warpAffine(tmp_color, tmp_color, affine_matrix, mask.size());
	cv::imshow("mask", mask);
	++num_classes;
      }
    }

    // Draw ROI for display
    cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
    cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);
  }

  // Perform matching
  std::vector<cv::linemod::Match> matches;
  std::vector<std::string> class_ids;
  std::vector<cv::Mat> quantized_images;

  match_timer.start();
  detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
  match_timer.stop();  

  int classes_visited = 0;
  std::set<std::string> visited;
  printf("%d\n",(int)matching_threshold);
 
  for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
  {
    // cv::linemod::Match m = matches[0];
    cv::linemod::Match m = matches[i];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
    // std::string parts_name = updatePartsName(m);
    std::string parts_name = "hoge";
    parts_name.pop_back();
    // int deg = updateDeg(m);
    // obj_pose_.theta = deg;
    int deg = 0;

    if (visited.insert(m.class_id).second){
      ++classes_visited;
      if (show_match_result){
        printf("Similarity:%5.1f%%; x:%3d; y:%3d; %s\n",
               m.similarity, m.x, m.y, m.class_id.c_str());
      }

      drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
      int similarity = (int)m.similarity; 
      std::string result_text = std::string(std::to_string(similarity))+"%"+" "+(std::to_string(deg))+"deg";
      cv::putText(display, result_text,
		  cv::Point2i(m.x, m.y), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);
      cv::putText(display, parts_name,
		  cv::Point2i(m.x, m.y-20), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(100,200, 0), 1);
      obj_pose_.x = matches[0].x;
      obj_pose_.y = matches[0].y;
      obj_pose_pub_.publish(obj_pose_);
    }
  }

  if (show_match_result && matches.empty())
    printf("No matches found...\n");
  if (show_timings)
  {
    printf("Training: %.2fs\n", extract_timer.time());
    printf("Matching: %.2fs\n", match_timer.time());
  }
  if (show_match_result || show_timings)
    printf("------------------------------------------------------------\n");

  cv::imshow("color", display);
  cv::imshow("normals", quantized_images[1]);

  cv::FileStorage fs;
  char key = (char)cvWaitKey(10);
  // if( key == 'q' )
  //   break;

  switch (key)
  {
  case 'h':
    help();
    break;
  case 'm':
    // toggle printing match result
    show_match_result = !show_match_result;
    printf("Show match result %s\n", show_match_result ? "ON" : "OFF");
    break;
  case 't':
    // toggle printing timings
    show_timings = !show_timings;
    printf("Show timings %s\n", show_timings ? "ON" : "OFF");
    break;
  case 'l':
    // toggle online learning
    learn_online = !learn_online;
    printf("Online learning %s\n", learn_online ? "ON" : "OFF");
    break;
  case '[':
    // decrement threshold
    matching_threshold = std::max(matching_threshold - 1, -100);
    printf("New threshold: %d\n", matching_threshold);
    break;
  case ']':
    // increment threshold
    matching_threshold = std::min(matching_threshold + 1, +100);
    printf("New threshold: %d\n", matching_threshold);
    break;
  case 'w':
    // write model to disk
    std::cout << "input name of template file name (.yml format) ->" << std::endl;
    std::cin >> filename;
    filename = template_path + filename;
    writeLinemod(detector, filename);
    printf("Wrote detector and templates to %s\n", filename.c_str());
    break;
  default:
    ;
  }
}

#endif
int main(int argc, char** argv)
{
  ros::init(argc, argv, "linemod");
  ROS_ERROR("linemod has been moved to opencv_contrib in OpenCV3");
#if CV_MAJOR_VERSION < 3
  // Various settings and flags
  help();
  cv::namedWindow("color");
  cv::namedWindow("normals");
  Mouse::start("color");

  // Initialize LINEMOD data structures
  
  
  if (argc == 1)
  {
    // filename = "linemod_templates.yml";
    // detector = cv::linemod::getDefaultLINEMOD();
    detector = cv::linemod::getDefaultLINE();
  }
  else
  {
    detector = readLinemod(argv[1]);

    std::vector<std::string> ids = detector->classIds();
    num_classes = detector->numClasses();
    printf("Loaded %s with %d classes and %d templates\n",
           argv[1], num_classes, detector->numTemplates());
    if (!ids.empty())
    {
      printf("Class ids:\n");
      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    }
  }

  // register callback
  ros::NodeHandle nh;
  ros::Subscriber img_sub_;
  img_sub_ = nh.subscribe("/stereo/left/image_rect_color",1, callback);
  obj_pose_pub_ = nh.advertise<geometry_msgs::Pose2D>("obj_pose", 1);

#endif
  ros::spin();
  return 0;
}

#if CV_MAJOR_VERSION < 3

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

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 255, 0),
                                        CV_RGB(0, 0, 255),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::rectangle(dst, cv::Point(offset.x, offset.y), cv::Point(offset.x+templates[m].width, offset.y+templates[m].height), CV_RGB(255,0,0), 3);
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T, color);
      cv::circle(dst, cv::Point(offset.x+templates[m].width/2, offset.y+templates[m].height/2), 5, cv::Scalar(0,0,255), -1);
    }
  }
}
#endif
