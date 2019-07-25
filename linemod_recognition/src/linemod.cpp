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


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/rgbd/linemod.hpp>

#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

    std::vector<cv::String> ids = detector->classIds();
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

void callback(const sensor_msgs::Image::ConstPtr& rgb_image)
{
    bool show_match_result = true;
    bool show_timings = false;

    cv::Size roi_size(200, 200);
    int learning_lower_bound = 90;
    int learning_upper_bound = 95;

    // Timers
    Timer extract_timer;
    Timer match_timer;
    int num_modalities = (int)detector->getModalities().size();
    cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat color = cv_rgb->image;
    std::vector<cv::Mat> sources;
    sources.push_back(color);
    cv::Mat display = color.clone();
    cv::imshow("color", display);

    std::cerr << 0 << std::endl;

    cv::Point mouse(Mouse::x(), Mouse::y());
    int event = Mouse::event();

    // Compute ROI centered on current mouse location
    cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
    cv::Point pt1 = mouse - roi_offset; // top left
    cv::Point pt2 = mouse + roi_offset; // bottom right

    if (event == CV_EVENT_RBUTTONDOWN)
        {
            cv::Mat mask;
            cv::Mat gray = color.clone();
            cv::cvtColor(color, gray , CV_BGR2GRAY);
            cv::Canny(gray, mask, 250, 500);

            cv::rectangle(display, pt1, pt2, CV_RGB(0, 0, 255), 3);
            cv::rectangle(mask, cv::Point(0,0), cv::Point(color.cols, pt2.y), cv::Scalar(0,0,0), -1, CV_AA);
            cv::rectangle(mask, cv::Point(0,pt2.y), cv::Point(color.cols, color.rows), cv::Scalar(0,0,0), -1, CV_AA);
            cv::rectangle(mask, cv::Point(0,0), cv::Point(pt1.x, color.rows), cv::Scalar(0,0,0), -1, CV_AA);
            cv::rectangle(mask, cv::Point(pt2.x,0), cv::Point(color.cols,color.rows), cv::Scalar(0,0,0), -1, CV_AA);
            cv::imshow("mask", mask);

            // Extract template
            std::string class_id = cv::format("class%d", num_classes);
            cv::Rect bb;
            extract_timer.start();
            int template_id = detector->addTemplate(sources, class_id, mask, &bb);
            extract_timer.stop();
            if (template_id != -1)
                {
                    printf("*** Added template (id %d) for new object class %d***\n",
                           template_id, num_classes);
                    //printf("Extracted at (%d, %d) size %dx%d\n", bb.x, bb.y, bb.width, bb.height);
                }

            ++num_classes;
        }

    // Draw ROI for display
    cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
    cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);



    // Perform matching
    std::vector<cv::linemod::Match> matches;
    std::vector<cv::String> class_ids;
    std::vector<cv::Mat> quantized_images;
    match_timer.start();
    std::cerr << 1 << std::endl;
    std::cerr << "sources.size(): " << sources.size() << std::endl;
    detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
    std::cerr << 2 << std::endl;
    match_timer.stop();



    int classes_visited = 0;
    std::set<std::string> visited;

    for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
        {
            cv::linemod::Match m = matches[i];

            if (visited.insert(m.class_id).second)
                {
                    ++classes_visited;

                    if (show_match_result)
                        {
                            printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
                                   m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
                        }

                    // Draw matching template
                    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
                    drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));

                }
        }

    std::cerr << 3 << std::endl;

    if (show_match_result && matches.empty())
        printf("No matches found...\n");
    if (show_timings)
        {
            printf("Training: %.2fs\n", extract_timer.time());
            printf("Matching: %.2fs\n", match_timer.time());
        }
    if (show_match_result || show_timings)
        printf("------------------------------------------------------------\n");



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
            writeLinemod(detector, filename);
            printf("Wrote detector and templates to %s\n", filename.c_str());
            break;
        default:
            ;
        }
}

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo
    > SyncPolicy;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "linemod");
    help();
    cv::namedWindow("color");
    Mouse::start("color");

    if (argc == 1)
        {
            filename = "linemod_templates.yml";
            // detector = cv::linemod::getDefaultLINEMOD();
            detector = cv::linemod::getDefaultLINE();
        }
    else
        {
            detector = readLinemod(argv[1]);

            std::vector<cv::String> ids = detector->classIds();
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
    ros::NodeHandle nh("~");
    ros::Subscriber img_sub_ = nh.subscribe("input",1 , callback);
    ros::spin();
    return 0;
}


void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                          CV_RGB(0, 255, 0),
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
                    cv::linemod::Feature f = templates[m].features[i];
                    cv::Point pt(f.x + offset.x, f.y + offset.y);
                    cv::circle(dst, pt, T / 2, color);
                }
        }
}
