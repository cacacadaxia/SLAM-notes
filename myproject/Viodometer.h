//
// Created by cacacadaxia on 2020/8/13.
//

#ifndef VO1_VIODOMETER_H
#define VO1_VIODOMETER_H
#include "Common.h"
#include "Camera.h"
#include "Frame.h"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
using namespace Sophus;
class Viodometer{
public:
    typedef shared_ptr<Viodometer> Ptr;
    Camera::Ptr camera_ = shared_ptr<Camera>(new Camera);
    Camera::Ptr came = std::make_shared<Camera>();
    Frame::Ptr last_frame_, cur_frame_;
    enum VOstate{
        INI = -1,
        OK = 0,
        LOST
    };
    VOstate state_;
    std::vector<Frame::Ptr> KeyFrame_;//关键帧
    Sophus::SE3 T_c_l_;
    /*创建匹配相关的东西*/
    cv::Ptr<cv::FeatureDetector> dector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    std::vector<cv::KeyPoint> KeyP_cur_;
    cv::Mat descriptor_last_, descriptor_cur_;
    std::vector<cv::Point3f> point3d_ref_;
    std::vector<cv::DMatch> matches_;
public:
    Viodometer(){
        state_ = INI;
        cur_frame_ = nullptr;
        last_frame_ = nullptr;
        dector = cv::ORB::create();
        descriptor = cv::ORB::create();
    }
    bool addFrame2( Frame::Ptr frame ){
        switch (this->state_) {
            case INI:
            {
                state_ = OK;
                cur_frame_ = last_frame_ = frame;
                ComputeDes();
                Set3dPoint();

                cur_frame_->Twc_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Vector3d(0,0,0));
                break;
            }
            case OK:
            {
                cur_frame_ = frame;
                ComputeDes();
                featureMatching();
                solvePNPFunc();
//                cout<<"point"<<endl;


                cur_frame_->Twc_ = last_frame_->Twc_ * T_c_l_.inverse();
                last_frame_ = cur_frame_;

                if(CheckFeyFrame() ) {
                    Set3dPoint();
                    KeyFrame_.push_back(cur_frame_);
                }
                break;
            }
            case LOST:
            {
                return false;
                break;
            }
        }
        return true;
    }
    


    void ComputeDes(){
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        dector->detect(cur_frame_->color, KeyP_cur_);
        cur_frame_->kp_tmp = KeyP_cur_;
        descriptor->compute(cur_frame_->color, KeyP_cur_, descriptor_cur_);
    }

    void Set3dPoint() {
        point3d_ref_.clear();
        descriptor_last_ = Mat();
        for (int i = 0; i < KeyP_cur_.size(); ++i) {
            double true_d = cur_frame_->findDepth(KeyP_cur_[i]);
            if (true_d > 0) {
                /*注意这里是需要转移到相机坐标系中去的*/
                Vector3d tmp = camera_->pixel2camera(
                        Vector2d(KeyP_cur_[i].pt.x, KeyP_cur_[i].pt.y), true_d);
                point3d_ref_.push_back(cv::Point3f(tmp(0), tmp(1), tmp(2)));
                descriptor_last_.push_back(descriptor_cur_.row(i));
            }
        }
    }

    void featureMatching(){
        std::vector<DMatch> match;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(descriptor_last_, descriptor_cur_, match);
        //选择dist
        double max_dist = 0.0, min_dist = 10000.0;

        for (int j = 0; j < match.size(); ++j) {
            if (match[j].distance > max_dist) max_dist = match[j].distance;
            if (match[j].distance < min_dist) min_dist = match[j].distance;
        }
        matches_.clear();
        for (int i = 0; i < match.size(); ++i) {
            if (match[i].distance < std::max(30.0, 2 * min_dist)) {
                matches_.push_back(match[i]);
            }
        }
    }
    void solvePNPFunc(){
        std::vector<cv::Point2f> point2d_2;
        std::vector<cv::Point3f> point3d_1;
        for( DMatch m:matches_){
            point3d_1.push_back(point3d_ref_[m.queryIdx]);
            point2d_2.push_back(KeyP_cur_[m.queryIdx].pt);
        }
        cv::Mat tmp_match_pic;
        cv::drawMatches(last_frame_->color, last_frame_->kp_tmp, cur_frame_->color, KeyP_cur_, matches_, tmp_match_pic);
        cv::imshow("匹配的特征点",tmp_match_pic);
        cv::waitKey(0);


        cv::Mat r, t, inliers;
        std::cout<<point2d_2.size()<<std::endl;
        cv::solvePnPRansac(point3d_1, point2d_2, camera_->K_, Mat(), r, t, false, 100, 4.0, 0.99, inliers);
        T_c_l_ = SE3(
                SO3(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0)),
                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
        );
        cout<<inliers.rows<<endl;
    }

    bool CheckFeyFrame() {
        /*检查是否为关键帧*/


        return true;
    }

    void FeatureMatch(const cv::Mat &img1, const cv::Mat &img2,
                      std::vector<cv::KeyPoint> &keyPoints1,
                      std::vector<cv::KeyPoint> &keyPoints2,
                      std::vector<cv::DMatch> &matchs
    ) {

        /*创建匹配相关的东西*/
        cv::BFMatcher matcher(cv::NORM_HAMMING);

        /*输出的match*/
        std::vector<cv::DMatch> match;

        /*1.检测FAST角点位置*/
        dector->detect(img1, keyPoints1);
        dector->detect(img2, keyPoints2);

        /*2.根据角点位置计算 BRIEF 描述子*/
        cv::Mat descriptor_1, descriptor_2;
        descriptor->compute(img1, keyPoints1, descriptor_1);
        descriptor->compute(img2, keyPoints2, descriptor_2);

        /*3.匹配*/
        matcher.match(descriptor_1, descriptor_2, match);

        //选择dist
        double max_dist = 0.0, min_dist = 10000.0;

        for (int j = 0; j < match.size(); ++j) {
            if (match[j].distance > max_dist) max_dist = match[j].distance;
            if (match[j].distance < min_dist) min_dist = match[j].distance;
        }

        std::vector<cv::KeyPoint> keyP1s, keyP2s;
        for (int i = 0; i < match.size(); ++i) {
            if (match[i].distance < std::max(30.0, 2 * min_dist)) {
                matchs.push_back(match[i]);
                keyP1s.push_back(keyPoints1.at(i));
                keyP2s.push_back(keyPoints2.at(i));
            }
        }
    }

    };



#endif //VO1_VIODOMETER_H
