//
// Created by cacacadaxia on 2020/8/13.
//

#ifndef VO1_FRAME_H
#define VO1_FRAME_H



#include "Common.h"
#include "Camera.h"

class Frame{
public:
    typedef shared_ptr<Frame> Ptr;
    Camera::Ptr camera_;
    cv::Mat depth;
    cv::Mat color;
    int ID_;
    Sophus::SE3 Twc_;//这就是初始状态
public:
    Frame(const int & id):camera_(nullptr),ID_(id){

    }

    static Ptr CreatePtr(const int& id){
        return shared_ptr<Frame>(new Frame(id));
    }
};
#endif //VO1_FRAME_H
