#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
//#include <opencv2/xfeatures2d.hpp>
//#include "faster_rcnn.hpp"
#include <string>
#include <sstream>
#include <iostream>
//三维坐标转化函数库
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
//整个程序应该是显示摄像头的程序 +信号槽，第一个while,显示图像即可，
//标定结束：点击之后跳出第一个while，赋值并到第二个while，实时detect，开始识别：对当前的图像调用函数更新数据。


using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;
//using namespace cv::dnn;
//声明dnn模块和rs的命名空间
#define PI 3.1415926
int ok=0;//控制在哪个while里
int stop=0;//控制是否更新数据
int class_num=2;

//int rect_point[4][2];
//int rect_x;
//int rect_y;
//int rect_w;
//int rect_h;
//double chang;
//double kuan;


int detected[20];

int adress[20];//id号对应的label数组下标，即adress[id]=label数组的下标
int wunums=0;//前面所有帧检测到的物体总数


/*

string int2string(int i)
{
    stringstream s;
    s << i;
    string ss = s.str();
    return ss;
}
*/







  cv_bridge::CvImagePtr cv_ptr;
/*
    ifstream inf;
	string s;
    //string model_file = "/home/szj/library/py-faster-rcnn/models/pascal_voc/ZF/faster_rcnn_alt_opt/faster_rcnn_test.pt";
    //string weights_file = "/home/szj/library/py-faster-rcnn/output/faster_rcnn_alt_opt/voc_2007_trainval/ZF_faster_rcnn_final.caffemodel";
    string model_file = "/home/szj/faster_rcnn_test.pt";
    string weights_file = "/home/szj/ZF_faster_rcnn_final.caffemodel";
    inf.open("/home/szj/library/faster-rcnn-cpp/caffe_model/coco_name.txt");
    //路径之后运行成功要进行修改
    while(getline(inf,s))
	{
		name_vec.push_back(s);
	}
    inf.close();
    //先读取物品种类
    */


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
        printf("我代码呢？");
    Mat img=cv_ptr->image;
   
	//vector<string> name_vec={"qrcode"};
    //vector<vector<int> > position;
   
   






/*
    //确定左上角顶点和size大小初始化rect，使得crop在profile输入流中居中
    const auto window_name = "Display Image";
    //auto char* const
    namedWindow(window_name, WINDOW_AUTOSIZE);


	int GPUID=0;
	Caffe::SetDevice(GPUID);
	Caffe::set_mode(Caffe::GPU);
	Detector det = Detector(model_file, weights_file, class_num);
	//初始化深度学习模块
	//Rect rect(rect_x, rect_y, rect_w, rect_h);




        // If we only received new depth frame, 
        // but the color did not update, continue


        position.clear();
		//if(position.size()!=0)
        //{
        //    position.clear();
        //}
        //不刷新position的位置很容易段错误
	    det.Detect(img, position);
        
        //char order[]="./zxing 1.jpg";
        //添加system的参数
        cout<<"position_size:"<<position.size()<<endl;
        
		for(int k = 0;k < position.size();k++)
		{
            cout<<"WTF"<<endl;
			cout<<"item:"<<name_vec[0]<<endl;
            
            Rect detection(position[k][1], position[k][2], position[k][3]-position[k][1], position[k][4]-position[k][2]);

       
        }
        */
           //dnn神经网络的处理
        imshow("qrcode", img);
        //);
        waitKey(50);
      
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

