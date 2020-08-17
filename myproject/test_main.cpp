//
// Created by cacacadaxia on 2020/8/11.
//

#include "Common.h"

#include "Config.h"
#include "Camera.h"
//#include "Viodometer.h"

enum VOstate{
    INI = -1,
    OK = 0,
    LOST
};
int main(){

    std::string file_name = "default.yaml";
    cv::FileStorage file_yaml = cv::FileStorage(file_name.c_str(),cv::FileStorage::READ);
    double fx = double(file_yaml["camera.fx"]);
    cout<<fx<<endl;

    Config::setParament(file_name);
    double fy = Config::get<double>("camera.fy");
    cout<<fy<<endl;

    Camera::Ptr camera_ = shared_ptr<Camera>(new Camera);
    Eigen::Vector3d tmp = camera_->pixel2camera(Eigen::Vector2d(1, 2), 3);

    int a = 100;
    int d = 200;
    int *b, *c;
    b = &a;
    c = &a;
    *c = 10;
    cout<<a<<endl;
    *b = 20;
    cout<<a<<endl;
    c = &d;
    a = 2000;
    cout<<*c<<endl;



}