/**
* king@2018.05.11
* desp: rosbag to tum style png file
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// king
#include <unistd.h>
#include <iomanip>

class ImageGrabber
{
public:
    ImageGrabber();
    ~ImageGrabber();
    void run();
private:
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
public:
    // ros::NodeHandle node;
private:
    // ros::NodeHandle private_node("~");
    std::string save_folder_;    // where to save tum dataset
    std::string rgb_topic_;      // rgb image topic name
    std::string depth_topic_;    // depth image
    std::ofstream f1_;           // rgb.txt
    std::ofstream f2_;           // depth.txt

};

// Constructor
ImageGrabber::ImageGrabber()
{

}   

// Destructor
ImageGrabber::~ImageGrabber()
{
    //
    f1_.close();
    f2_.close();
    ros::shutdown();
}

void ImageGrabber::run()
{
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    // parameters
    private_node.param<std::string>("save_folder", save_folder_, std::string("./image"));
    private_node.param<std::string>("rgb_topic", rgb_topic_, std::string("/kinect2/qhd/image_color_rect"));
    private_node.param<std::string>("depth_topic", depth_topic_, std::string("/kinect2/qhd/image_depth_rect"));
    // # timestamp filename
    f1_.open(save_folder_+"/rgb.txt");
    f2_.open(save_folder_+"/depth.txt");
    f1_ << "# timestamp filename\n";
    f2_ << "# timestamp filename\n";

    // 
    std::cout << "Subscribe to: " << rgb_topic_ << " & " << depth_topic_ << std::endl;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node, rgb_topic_, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node, depth_topic_, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,this,_1,_2));

    ros::spin();
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // record
    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed);  //只有在这项设置后，setprecision才是设置小数的位数。
    ss << std::setprecision(6);
    // rgb
    ss << cv_ptrRGB->header.stamp.toSec();
    std::string rgb_name = "rgb/" + ss.str() + ".png";
    f1_ << ss.str() << " " << rgb_name << std::endl;
    // depth
    ss.str("");
    ss << cv_ptrD->header.stamp.toSec();
    std::string depth_name = "depth/" + ss.str() + ".png";
    f2_ << ss.str() << " " << depth_name << std::endl;
    //
    rgb_name = save_folder_ + "/" + rgb_name;
    depth_name = save_folder_ + "/" + depth_name;
    std::cout << "rgb_name: " << rgb_name << std::endl;
    cv::imwrite(rgb_name, cv_ptrRGB->image);
    cv::imwrite(depth_name, cv_ptrD->image);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag2img_node");
    ros::start();
    // Create bag2img system. 
    ImageGrabber igb;
    igb.run();
    // ros::spin();
    ros::shutdown();
    
    return 0;
}



