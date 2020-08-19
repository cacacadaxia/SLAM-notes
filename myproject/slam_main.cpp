//
// Created by cacacadaxia on 2020/8/13.
//

#include "Common.h"
#include "Config.h"
#include "Frame.h"
#include "Camera.h"
#include "utils.h"
#include "Viodometer.h"
#include "vio.h"
int main(){
    cout<<"fuck you"<<endl;

    //读取文件
    Config::setParament("default.yaml");
    string location = Config::get<string>("dataset_dir");
    std::vector<string> ImageFilenamesRGB, ImageFilenamesD;
    std::vector<double> TimeStamps;
    Utils::LoadImages(location + "associate.txt", ImageFilenamesRGB, ImageFilenamesD, TimeStamps);

    //循环
    Viodometer::Ptr viodometer(new Viodometer);
    slambook::VisualOdometry::Ptr tmp(new slambook::VisualOdometry);
    Camera::Ptr camera(new Camera);
    for (int i = 0; i < ImageFilenamesRGB.size(); ++i) {
        cv::Mat rgb = cv::imread(location + ImageFilenamesRGB[i]);
        cv::Mat dep = cv::imread(location + ImageFilenamesD[i]);

        Frame::Ptr frame = Frame::CreatePtr(i);
        frame->color = rgb;
        frame->depth = dep;
        frame->camera_ = camera;
//        viodometer->addFrame2( frame);
        tmp->addFrame(frame);

        if (i==0) cout<<frame->Twc_<<endl;
    }


    cout<<"fuck me"<<endl;

}


