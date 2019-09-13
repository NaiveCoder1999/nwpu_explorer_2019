#ifndef FASTER_RCNN_HPP
#define FASTER_RCNN_HPP
#include <stdio.h>  // for snprintf
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include "caffe/caffe.hpp"
#include "gpu_nms.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace caffe;
using namespace std;

#define mymax(a, b) (((a)>(b)) ? (a) :(b))
#define mymin(a, b) (((a)<(b)) ? (a) :(b))

//background and soso
//const int class_num = 4;

/*
 * ===  Class  ======================================================================
 *         Name:  Detector
 *  Description:  FasterRCNN CXX Detector
 * =====================================================================================
 */
class Detector {
public:
    Detector(const string& model_file, const string& weights_file,int class_num);
    void Detect(cv::Mat frame, vector<vector<int> > & position);
    void bbox_transform_inv(const int num, const float* box_deltas, const float* pred_cls, float* boxes, float* pred, int img_height, int img_width);
    bool vis_detections(cv::Mat image, int* keep, int num_out, float* sorted_pred_cls, float CONF_THRESH, int clas, vector<vector<int> > & position,char names[]);
    void boxes_sort(int num, const float* pred, float* sorted_pred);

private:
    int class_num;
    boost::shared_ptr<Net<float> > net_;
    Detector(){}
};

//Using for box sort
struct Info
{
    float score;
    const float* head;
};
bool compare(const Info& Info1, const Info& Info2)
{
    return Info1.score > Info2.score;
}
#endif
