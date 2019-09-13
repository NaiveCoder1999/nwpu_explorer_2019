#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include<stdio.h>
#include<math.h>
#include<vector>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
using namespace cv;

class ImageConverter
{
private:
    ros::NodeHandle nh_;
    //用于将msg信息转换为openCV中的Mat数据
    image_transport::ImageTransport it_;
    //订阅摄像头发布的信息
    image_transport::Subscriber image_sub_;
public:
/*
    void image_process(cv::Mat img) 
    {
       cv::Mat img_out;
       cv::cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象
       cv::imshow("Input", img);
       cv::imshow("Output", img_out);
       cv::waitKey(5);
    }
  */  
    ImageConverter()
    : it_(nh_)
    {
        //设置订阅摄像机
        //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
        //image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        //订阅自带laucnh包发布图像话题
        image_sub_ = it_.subscribe("/camera/rs/image", 1, &ImageConverter::imageCb, this);
        //image_sub_ = it_.subscribe("/camera/color/image_raw_mouse_left", 1, &ImageConverter::imageCb, this);
        printf("WTF??????\n");
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter(){}

    //收到摄像机后的回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        printf("WTF111111111\n");
        try{
            //将收到的消息使用cv_bridge转移到全局变量图像指针cv_ptr中，其成员变量image就是Mat型的图片
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //image_process(cv_ptr->image);
       //仅仅订阅消息，调用回调函数，使用自行编译的opencv代码
        //处理图片信息
        myCode();
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //cv::waitKey(3);
    }
   
    
    int myCode()
    {   
        printf("WTF22222222222\n");
        //namedWindow("WTF?");
        //Mat temp=
        Mat img=cv_ptr->image;
        //Mat img=imread("/home/szj/a.jpg");
        cv::imshow(OPENCV_WINDOW,img);
        waitKey(100);
        return 0;
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_location");
    ImageConverter ic;
    //循环等待
    ros::spin();
    return 0;
}