#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#define TF_OFFSET 5

//global
ros::Publisher flag_pub;
std_msgs::Int16 flag;
tf::TransformListener* listener_ptr;
tf2_ros::Buffer* tf_buffer;
cv::Ptr<cv::linemod::Detector> detector;
std::string filename;
int num_classes = 0;
int matching_threshold;
int pose_flag;

bool depth_use;
bool canny;
bool otsu;
bool binary;
int canny_val1;
int canny_val2;
float initial_dist;
int scale_change;
int roi_x;
int roi_y;
const int end_index = 4381;

class Mouse{
public:
    static void start(const std::string& a_img_name){
        cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
    }
    static int event(void){
        int l_event = m_event;
        // m_event = -1;
        return l_event;
    }
    static int x(void){
        return m_x;
    }
    static int y(void) {
        return m_y;
    }
    static int mouse_flag(void){
        return m_flag;
    }
private:
    static void cv_on_mouse(int a_event, int a_x, int a_y, int a_flag, void *){
        m_event = a_event;
        m_x = a_x;
        m_y = a_y;
        m_flag = a_flag;
    }
    static int m_event;
    static int m_x;
    static int m_y;
    static int m_flag;
};

int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;
int Mouse::m_flag;

static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename){
    cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    detector->read(fs.root());
    cv::FileNode fn = fs["classes"];
    for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
        detector->readClass(*i);
    return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename){
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    detector->write(fs);
    std::vector<std::string> ids = detector->classIds();
    fs << "classes" << "[";
    for (int i = 0; i < (int)ids.size(); ++i) {
        fs << "{";
        detector->writeClass(ids[i], fs);
        fs << "}";
    }
    fs << "]";
}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities,
                  cv::Mat& dst,
                  cv::Point offset,
                  int T){
    static const cv::Scalar COLORS[5] = {CV_RGB(0, 0, 255), CV_RGB(0, 255, 0)};
    for (int m = 0; m < num_modalities; ++m){
        cv::Scalar color = COLORS[m];
        for (int i = 0; i < (int)templates[m].features.size(); ++i) {
            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);
            cv::circle(dst, pt, T / 2, color);
        }
    }
}

bool cropImage(cv::Mat &mask, cv::Size &roi_size){
        cv::rectangle(mask,
                      cv::Point(0,0),
                      cv::Point(mask.cols,Mouse::y()-roi_size.height/2),
                      cv::Scalar(0,0,0),
                      -1,
                      CV_AA);
        cv::rectangle(mask,
                      cv::Point(0,Mouse::y()+roi_size.height/2),
                      cv::Point(mask.cols,mask.rows),
                      cv::Scalar(0,0,0),
                      -1,
                      CV_AA);
        cv::rectangle(mask,
                      cv::Point(0,0),
                      cv::Point(Mouse::x()-roi_size.width/2,mask.rows),
                      cv::Scalar(0,0,0),
                      -1,
                      CV_AA);
        cv::rectangle(mask,
                      cv::Point(Mouse::x()+roi_size.width/2,0),
                      cv::Point(mask.cols,mask.rows),
                      cv::Scalar(0,0,0),
                      -1,
                      CV_AA);
        return true;
}

void updateCreateTemplateCb(const std_msgs::Int16 msg){
    pose_flag = msg.data;
}

void callback(const sensor_msgs::Image::ConstPtr& rgb_image,
              const sensor_msgs::Image::ConstPtr& depth_image,
              const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info,
              const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info){
    bool show_match_result = true;
    cv::Size roi_size(roi_x, roi_y);
    int num_modalities = (int)detector->getModalities().size();
    cv_bridge::CvImagePtr cv_rgb = 
        cv_bridge::toCvCopy(rgb_image,sensor_msgs::image_encodings::BGR8);
    cv::Mat color = cv_rgb->image;

    std::vector<cv::Mat> sources;
    sources.push_back(color);

    cv::Mat depth;
    if(depth_use){
        cv_bridge::CvImagePtr cv_depth = 
            cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_m =  cv_depth->image;
        cv::imshow("depth_m", depth_m);
        depth_m.convertTo(depth, CV_16UC1, 1000.0);
        double focal_length = depth_camera_info->K[0];
        sources.push_back(depth/1000.0);
    }

    cv::Mat display = color.clone();

    cv::Point mouse(Mouse::x(), Mouse::y());
    int event = Mouse::event();
    int mouse_event_flag = Mouse::mouse_flag();

    cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
    cv::Point pt1 = mouse - roi_offset;
    cv::Point pt2 = mouse + roi_offset;
    std::vector<CvPoint> chain(4);
    chain[0] = pt1;
    chain[1] = cv::Point(pt2.x, pt1.y);
    chain[2] = pt2;
    chain[3] = cv::Point(pt1.x, pt2.y);

    pose_flag = 0;

    int classes_index = num_classes-TF_OFFSET;

    if((classes_index) % scale_change == 0 && classes_index != 0){
        printf("chage camera distance\n");
        cv::waitKey(0);
        pose_flag = 1;
    }

    cv::Mat canny_img;
    cv::Mat gray_conf = color.clone();
    cv::cvtColor(color, gray_conf , CV_BGR2GRAY);
    cv::Canny(gray_conf, canny_img, canny_val1, canny_val2);
    cv::imshow("canny_img", canny_img);

    if (mouse_event_flag == CV_EVENT_FLAG_RBUTTON) {
        // if(event == CV_EVENT_RBUTTONDOWN){
        cv::Mat mask;
        cv::Mat gray = color.clone();
        cv::cvtColor(color, gray , CV_BGR2GRAY);

        if(canny){
            cv::Canny(gray, mask, canny_val1, canny_val2);
        } else if(otsu){
            cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY | cv::THRESH_OTSU);
        } else {
            cv::threshold(gray, mask, 100, 255, CV_THRESH_BINARY);
        }

        // crop
        cropImage(mask, roi_size);
        cv::imshow("mask", mask);

        geometry_msgs::TransformStamped transform_stamped;

        try{
            transform_stamped = tf_buffer->lookupTransform("linemod_camera",
                                                           "linemod_template",
                                                           ros::Time(0));
        } catch(tf2::TransformException &ex) {
            ROS_WARN("tf2 error: %s", ex.what());
        }
    
        double qx = transform_stamped.transform.rotation.x;
        double qy = transform_stamped.transform.rotation.y;
        double qz = transform_stamped.transform.rotation.z;
        double qw = transform_stamped.transform.rotation.w;

        std::string class_id = cv::format("%d,%lf,%lf,%lf,%lf",
                                          classes_index,
                                          qx,
                                          qy,
                                          qz,
                                          qw);
        cv::Rect bb;
        int template_id;
        if(num_classes >= 5)
            template_id = detector->addTemplate(sources, class_id, mask, &bb);

        flag.data = pose_flag;
        flag_pub.publish(flag);
        ++num_classes;
    }

    cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
    cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);

    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);

    if(!matches.empty()){
        cv::linemod::Match m = matches[0];
        std::cout << m.class_id.c_str() << std::endl;

        const std::vector<cv::linemod::Template>& templates =
            detector->getTemplates(m.class_id, m.template_id);
        drawResponse(templates,
                     num_modalities,
                     display,
                     cv::Point(m.x, m.y),
                     detector->getT(0));
        int similarity = (int)m.similarity;
        cv::putText(display,
                    std::string(std::to_string(similarity))+"%",
                    cv::Point2i(m.x, m.y),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.8,
                    cv::Scalar(255, 0, 0),
                    2);
    } else {
        printf("No matches found...\n");
    }
    cv::imshow("color", display);
    cv::FileStorage fs;
    char key = (char)cvWaitKey(10);

    if(classes_index == end_index)
        writeLinemod(detector, filename);

    printf("-----------------------------------------------------\n");
    cv::waitKey(500);
}


typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo
    > SyncPolicy;

int main(int argc, char** argv){
    ros::init(argc, argv, "gazebo_linemod");
    ros::NodeHandle nh("~");
    cv::namedWindow("color");
    Mouse::start("color");

    nh.getParam("filename", filename);
    nh.getParam("depth_use", depth_use);
    nh.getParam("canny", canny);
    nh.getParam("otsu", otsu);
    nh.getParam("binary", binary);
    nh.getParam("canny_val1", canny_val1);
    nh.getParam("canny_val2", canny_val2);
    nh.getParam("initial_dist", initial_dist);
    nh.getParam("scale_change", scale_change);
    nh.getParam("roi_x", roi_x);
    nh.getParam("roi_y", roi_y);
    nh.getParam("matching_threshold", matching_threshold);

    if (argc == 1){
        filename = "./gazebo_linemod_templates.yml";
        if(depth_use){
            detector = cv::linemod::getDefaultLINEMOD();
        } else {      
            detector = cv::linemod::getDefaultLINE();
        }
    }else{
        detector = readLinemod(argv[1]);
        std::vector<std::string> ids = detector->classIds();
        num_classes = detector->numClasses();
        printf("Loaded %s with %d classes and %d templates\n",
               argv[1],
               num_classes,
               detector->numTemplates());
        if (!ids.empty()){
            printf("Class ids:\n");
            std::copy(ids.begin(),
                      ids.end(),
                      std::ostream_iterator<std::string>(std::cout, "\n"));
        }
    }

    tf2_ros::Buffer tf_buffer_ptr;
    tf_buffer = &tf_buffer_ptr;
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_image;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_camera_info;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_camera_info;
    sub_rgb_image.subscribe(nh, "rgb_image", 1);
    sub_depth_image.subscribe(nh, "depth_image", 1);
    sub_rgb_camera_info.subscribe(nh, "rgb_camera_info", 1);
    sub_depth_camera_info.subscribe(nh, "depth_camera_info", 1);
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_ = 
        boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
    sync_->connectInput(sub_rgb_image,
                        sub_depth_image,
                        sub_rgb_camera_info,
                        sub_depth_camera_info);
    sync_->registerCallback(callback);

    ros::Subscriber flag_sub = 
        nh.subscribe("/viewpoint_planner/poseFrag", 1000, updateCreateTemplateCb);
    flag_pub =
        nh.advertise<std_msgs::Int16>("/viewpoint_planner/createFrag", 1000);

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}
