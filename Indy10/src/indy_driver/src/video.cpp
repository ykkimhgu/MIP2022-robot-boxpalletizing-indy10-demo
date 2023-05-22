#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
using namespace std;
using namespace cv;


double img_size =0;
cv::Mat color_imgshow, depth_imgshow;

float cmdata[] = {1190.596624173294, 0, 613.8264982857706, 0, 1197.481389143005, 496.568669564977, 0, 0, 1};
float dsdata[] ={-0.3169705527285323, 0.157151705099085, -0.003364806857495433, 0.008141312356792239, 0};
cv::Mat cmtrx(3,3,CV_32F,cmdata);
cv::Mat dstrx(1,5,CV_32F,dsdata);



void color_imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
        cv::Mat color_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat color_calframe ;
        cv::undistort(color_frame, color_calframe, cmtrx, dstrx);

        cv::Point2f color_src_p[4], color_dst_p[4];
        color_src_p[0] = cv::Point2f(244,143);
        color_src_p[1] = cv::Point2f(1038,132);
        color_src_p[2] = cv::Point2f(91,909);
        color_src_p[3] = cv::Point2f(1212,896);

        color_dst_p[0] = cv::Point2f(244,132);
        color_dst_p[1] = cv::Point2f(1038,132);
        color_dst_p[2] = cv::Point2f(244,896);
        color_dst_p[3] = cv::Point2f(1038,896);

        cv::Mat color_perspect_mat = cv::getPerspectiveTransform(color_src_p, color_dst_p);

        cv::Mat color_ipm;
        cv::warpPerspective(color_calframe, color_ipm, color_perspect_mat, Size(1288,964));

        Rect rect(244,132,794,764);
        color_imgshow = color_ipm(rect);

    }

    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void depth_imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv::imshow("view2", cv_bridge::toCvShare(msg, "TYPE_16UC1")->image);
        cv::waitKey(30);
        cv::Mat depth_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        cv::Mat depth_calframe ;
        cv::undistort(depth_frame, depth_calframe, cmtrx, dstrx);

        cv::Point2f depth_src_p[4], depth_dst_p[4];
        depth_src_p[0] = cv::Point2f(244,143);
        depth_src_p[1] = cv::Point2f(1038,132);
        depth_src_p[2] = cv::Point2f(91,909);
        depth_src_p[3] = cv::Point2f(1212,896);

        depth_dst_p[0] = cv::Point2f(244,132);
        depth_dst_p[1] = cv::Point2f(1038,132);
        depth_dst_p[2] = cv::Point2f(244,896);
        depth_dst_p[3] = cv::Point2f(1038,896);

        cv::Mat depth_perspect_mat = cv::getPerspectiveTransform(depth_src_p, depth_dst_p);

        cv::Mat depth_ipm;
        cv::warpPerspective(depth_calframe, depth_ipm, depth_perspect_mat, Size(1288,964));

        Rect rect(244,132,794,764);
        depth_imgshow = depth_ipm(rect);

    }

    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


  
int main(int argc, char **argv){
    ros::init(argc, argv, "video");
    ros::NodeHandle n01, n02, n03, n04;

    image_transport::ImageTransport it1(n01);
    image_transport::ImageTransport it2(n02);
    image_transport::ImageTransport it3(n03);
    image_transport::ImageTransport it4(n04);
    image_transport::Subscriber sub_coler_Image = it1.subscribe("camera/color/image_raw", 1, color_imageCallback);
    image_transport::Subscriber sub_depth_Image = it2.subscribe("camera/depth/image_rect_raw", 1, depth_imageCallback);
    image_transport::Publisher pub_color_Image = it3.advertise("color_video/img", 1);
    image_transport::Publisher pub_depth_Image = it4.advertise("depth_video/img", 1);

    sensor_msgs::Image::Ptr msg_colImage;
    sensor_msgs::Image::Ptr msg_depImage;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(10);
    
    while (ros::ok()){

        msg_colImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_imgshow).toImageMsg();
        msg_depImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_imgshow).toImageMsg();
        pub_color_Image.publish(msg_colImage);
        pub_depth_Image.publish(msg_depImage);
        
        loop_rate.sleep();    
    }

    
    cv::destroyWindow("view");
    cv::destroyWindow("view2");
 }
