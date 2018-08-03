#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <stdexcept>
#include <std_msgs/Int16.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Pose start_pose;
geometry_msgs::Twist start_twist;
geometry_msgs::Pose kinect_pose;
geometry_msgs::Twist kinect_twist;

geometry_msgs::Vector3 gazebo_pose;
gazebo_msgs::ModelState modelstate;
gazebo_msgs::SetModelState setmodelstate;
gazebo_msgs::ModelState kinect_modelstate;
gazebo_msgs::SetModelState kinect_setmodelstate;

ros::ServiceClient *client_ptr;
ros::ServiceClient *get_model_state_ptr;
std::string object  = "linemod_template";
std::string camera = "linemod_camera";
std::string line;
std::string filename;
std::ifstream ifs("./data/hemisphere_icosahedron.csv");
// std::ifstream ifs(filename);
ros::Publisher pose_flag_pub;
std_msgs::Int16 pose_flag;

float dist_step = 0;
float initial_dist;
float initial_z;

int index_ = 0;
const int tf_offset = 5;

std::vector<std::string> split(std::string& input, char delimiter){
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

bool setPose(geometry_msgs::Pose &pose,
             geometry_msgs::Twist &twist,
             geometry_msgs::Quaternion &quaternion,
             const double &x,
             const double &y,
             const double &z){
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = quaternion.x;
    pose.orientation.y = quaternion.y;
    pose.orientation.z = quaternion.z;
    pose.orientation.w = quaternion.w;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    return true;
}

bool setModel(const std::string &model_name,
              const std::string &ref_frame,
              const geometry_msgs::Pose &pose,
              const geometry_msgs::Twist &twist,
              gazebo_msgs::ModelState &modelstate){
    modelstate.model_name = model_name;
    modelstate.reference_frame = ref_frame;
    modelstate.pose = pose;
    modelstate.twist = twist;

    return true;
}

void updatePoseCb(const std_msgs::Int16 msg){
    ros::ServiceClient client = (ros::ServiceClient)*client_ptr;
    ros::ServiceClient get_model_state = (ros::ServiceClient)*get_model_state_ptr;
    std::cout << "---------------------------------" << std::endl << std::endl;;
    std::vector<double> vec;
    if(index_ > tf_offset){
        std::getline(ifs, line);
        std::vector<std::string> strvec = split(line, ' ');
        for (int i=0; i<strvec.size();i++){
            try{
                vec.push_back(std::stod(strvec[i]));
                std::cout << vec.at(i) << ", " << i <<  std::endl;
            } catch(std::invalid_argument e) {
                std::cout << "error! check csv file" << std::endl;
            }
        }
    } else {
        ROS_WARN("meaningless process to avoid tf error. todo: revise this.");
        for(int i=0; i<6; i++){
            vec.push_back(0);
        }
    }

    if(pose_flag.data == 1)
        dist_step = dist_step + 0.1;

    double x, y, z, roll, pitch, yaw; 
    pitch = vec.at(3); // theta
    yaw = vec.at(4); // phi
    x = 0;
    y = 0;
    z = initial_z;
  
    float camera_position_z = z + initial_dist + dist_step;
    std::cout << "camera_distance: " << camera_position_z << std::endl;
    pose_flag.data = msg.data;

    geometry_msgs::Quaternion quaternion;
    tf::Quaternion quat;
    quat.setEulerZYX(yaw, pitch, 0);
    tf::Quaternion offset_quat = tf::createQuaternionFromRPY(M_PI/2, 0, 0);

    quat *= offset_quat;
    quaternionTFToMsg(quat, quaternion);

    setPose(start_pose, start_twist, quaternion, x, y, z);
    setModel(object, "world", start_pose, start_twist, modelstate);
    setmodelstate.request.model_state = modelstate;
    bool is_seted_template = client.call(setmodelstate);

    // in-plane rotation
    tf::Quaternion inplane_quat = tf::createQuaternionFromRPY(0, 0, vec[5]); 
    tf::Quaternion kinect_offset_quat = tf::createQuaternionFromRPY(0, M_PI/2, 0);
    inplane_quat *= kinect_offset_quat;
    geometry_msgs::Quaternion kinect_quat;
    quaternionTFToMsg(inplane_quat, kinect_quat);

    setPose(kinect_pose, kinect_twist, kinect_quat, 0, 0, camera_position_z);
    setModel("camera_sim", "world", kinect_pose, kinect_twist, kinect_modelstate);
    kinect_setmodelstate.request.model_state = kinect_modelstate;
    bool is_seted_camera = client.call(kinect_setmodelstate);

    for(int i=0; i<10; i++){
        static tf::TransformBroadcaster broadcaster;
        tf::Transform world_to_camera_tf;
        world_to_camera_tf.setOrigin(tf::Vector3(0, 0, camera_position_z));
        tf::Quaternion kinect_quat_tf(kinect_quat.x,
                                      kinect_quat.y,
                                      kinect_quat.z,
                                      kinect_quat.w);
        world_to_camera_tf.setRotation(kinect_quat_tf);
        broadcaster.sendTransform(tf::StampedTransform(world_to_camera_tf,
                                                       ros::Time::now(),
                                                       "world",
                                                       camera));
        tf::Transform world_to_template_tf;
        world_to_template_tf.setOrigin(tf::Vector3(0, 0, 2));
        world_to_template_tf.setRotation(quat);
        broadcaster.sendTransform(tf::StampedTransform(world_to_template_tf,
                                                       ros::Time::now(),
                                                       "world",
                                                       object));
        ros::Duration(0.001).sleep();
    }
    pose_flag_pub.publish(pose_flag);
    index_++;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "viewpoint_planner");
    ros::NodeHandle nh("~");

    nh.getParam("filename", filename);
    nh.getParam("initial_dist", initial_dist);
    nh.getParam("initial_z", initial_z);

    ros::ServiceClient client =
        nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient get_model_state =
        nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    client_ptr = &client;
    get_model_state_ptr = &get_model_state;
    ros::Subscriber pose_sub =
        nh.subscribe("/viewpoint_planner/createFrag", 1000, updatePoseCb);
    pose_flag_pub =
        nh.advertise<std_msgs::Int16>("/viewpoint_planner/poseFrag",1000);
    ros::spin();
    return 0;
}
