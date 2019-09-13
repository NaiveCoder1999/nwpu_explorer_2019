#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/xfeatures2d.hpp>
#include <string>
#include <sstream>
//#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include "cv-helpers.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//整个程序应该是显示摄像头的程序 +信号槽，第一个while,显示图像即可，
//标定结束：点击之后跳出第一个while，赋值并到第二个while，实时detect，开始识别：对当前的图像调用函数更新数据。


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
//using namespace cv::dnn;
using namespace rs2;
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


//Realsense添加代码

const size_t inWidth      = 300;
const size_t inHeight     = 300;
const float WHRatio       = inWidth / (float)inHeight;
//长宽之比
const float inScaleFactor = 0.007843f;
const float meanVal       = 127.5;

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"rs_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/color/image_raw",1);
    
    
    //string model_file = "/home/szj/library/py-faster-rcnn/models/pascal_voc/ZF/faster_rcnn_alt_opt/faster_rcnn_test.pt";
    //string weights_file = "/home/szj/library/py-faster-rcnn/output/faster_rcnn_alt_opt/voc_2007_trainval/ZF_faster_rcnn_final.caffemodel";


    //Net net = readNetFromCaffe("MobileNetSSD_deploy.prototxt", 
     //                          "MobileNetSSD_deploy.caffemodel");

    rs2::pipeline pipe;
    auto config = pipe.start();
    
    //auto class rs2::pipeline_profile
    //开启D435进行流式输入
    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
                         //数据类型可以直接auto的吗？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？、
    //此处auto rs2::video_stream_profile
    rs2::align align_to(RS2_STREAM_COLOR);
    //这句话什么意思》。。。。。。。。



    /*
    //Size cropSize;
    //width() and height()是类video_stream_profile的两个成员变量
    //crop相关
    
    if (profile.width() / (float)profile.height() > WHRatio)
    {
        cropSize = Size(static_cast<int>(profile.height() * WHRatio),
                        profile.height());
    }
    else
    {
        cropSize = Size(profile.width(),
                        static_cast<int>(profile.width() / WHRatio));
    }
    //保持窗口cropsize为正方形，边长由width和height中较小的一个确定

    Rect crop(Point((profile.width() - cropSize.width) / 2,
                    (profile.height() - cropSize.height) / 2),
              cropSize);

     */





    //确定左上角顶点和size大小初始化rect，使得crop在profile输入流中居中
    //const auto window_name = "Display Image";
    //auto char* const
    //namedWindow(window_name, WINDOW_AUTOSIZE);



	//初始化深度学习模块
	//Rect rect(rect_x, rect_y, rect_w, rect_h);


    //while (1)
    //通过窗口名称获取句柄`
    //{
        // 等待接下来的几帧
        
        //auto rs2::frameset

        //确保帧数在空间上对齐
       
        //返回的也是frameset类型
        //分辨率可能造成影响

       

        // If we only received new depth frame, 
        // but the color did not update, continue
        //static int last_frame_number = 0;
        //if (color_frame.get_frame_number() == last_frame_number) continue;
        //last_frame_number = color_frame.get_frame_number();

    

        
        ros::Rate loop_rate(5);
        while (nh.ok()) 
        {
            auto data = pipe.wait_for_frames();
            data = align_to.process(data);
             auto color_frame = data.get_color_frame();
           //auto video_frame
           //auto depth_frame = data.get_depth_frame();
           //auto depth_frame
               // 将realsense的一帧转化为OpenCV的matrix格式
        auto color_mat = frame_to_mat(color_frame);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_mat).toImageMsg();
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        //video_frame转化为Mat
        //auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
        //depth_frame转化为Mat
        //imshow("depth_mat",depth_mat);
        //imshow("clor_mat",color_mat);
        //auto Mat

        //position.clear();
		//if(position.size()!=0)
        //{
        //    position.clear();
        //}
        //不刷新position的位置很容易段错误
	    //det.Detect(color_mat, position);
        
        //char order[]="./zxing 1.jpg";
        //添加system的参数
        //cout<<"position_size:"<<position.size()<<endl;
      
    //}

    return 0;
}   

