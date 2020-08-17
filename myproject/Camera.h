//
// Created by cacacadaxia on 2020/8/13.
//

#ifndef VO1_CAMERA_H
#define VO1_CAMERA_H
#include "Common.h"

using namespace cv;
using namespace Eigen;
using namespace Sophus;
class Camera{
public:
    typedef shared_ptr<Camera> Ptr;
    double fx_, fy_, cx_, cy_, depth_scale_;
    cv::Mat K_;
public:
    Camera(){
        fx_ = Config::get<double>("camera.fx");
        fy_ = Config::get<double>("camera.fy");
        cx_ = Config::get<double>("camera.cx");
        cy_ = Config::get<double>("camera.cy");
        depth_scale_ = Config::get<double>("camera.depth_scale");
        K_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    }

    Vector3d world2camera ( const Vector3d& p_w, const SE3& T_c_w )
    {
        return T_c_w*p_w;
    }

    Vector3d camera2world ( const Vector3d& p_c, const SE3& T_c_w )
    {
        return T_c_w.inverse() *p_c;
    }

    Vector2d camera2pixel ( const Vector3d& p_c )
    {
        return Vector2d (
                fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
                fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
        );
    }

    Vector3d pixel2camera ( const Vector2d& p_p, double depth )
    {
        return Vector3d (
                ( p_p ( 0,0 )-cx_ ) *depth/fx_,
                ( p_p ( 1,0 )-cy_ ) *depth/fy_,
                depth
        );
    }

    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
    {
        return camera2pixel ( world2camera ( p_w, T_c_w ) );
    }

    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
    {
        return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
    }

};



#endif //VO1_CAMERA_H
