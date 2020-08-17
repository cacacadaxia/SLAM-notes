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


    double findDepth(const cv::KeyPoint &kp){
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth.ptr<ushort>(y)[x];
        if ( d!=0 )
        {
            return double(d)/camera_->depth_scale_;
        }
        else
        {
            // check the nearby points
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = depth.ptr<ushort>( y+dy[i] )[x+dx[i]];
                if ( d!=0 )
                {
                    return double(d)/camera_->depth_scale_;/*depth_scale_内参*/
                }
            }
        }
        return -1.0;

    }
};
#endif //VO1_FRAME_H
