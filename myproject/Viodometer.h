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
    std::vector<Frame> KeyFrame;//关键帧
    Sophus::SE3 T_c_l_;
    /*创建匹配相关的东西*/
    cv::Ptr<cv::FeatureDetector> dector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
public:
    Viodometer(){
        state_ = INI;
        cur_frame_ = nullptr;
        last_frame_ = nullptr;
        dector = cv::ORB::create();
        descriptor = cv::ORB::create();
    }

//    bool addFrame_slambook(Frame::Ptr frame){
//        switch (this->state_) {
//            case INI:
//            {
//                state_ = OK;
//                cur_frame_ = last_frame_ = frame;
//                cur_frame_->Twc_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Vector3d(0,0,0));
//                break;
//            }
//            case OK:
//            {
//                cur_frame_ = frame;
//                dector->detect ( cur_frame_->color, keypoints_curr_ );
//                dector->compute (cur_frame_->color, keypoints_curr_, descriptors_curr_ );
//                // match desp_ref and desp_curr, use OpenCV's brute force match
//                vector<cv::DMatch> matches;
//                cv::BFMatcher matcher ( cv::NORM_HAMMING );
//                matcher.match ( descriptors_ref_, descriptors_curr_, matches );
//                // select the best matches寻找不错的匹配
//                float min_dis = std::min_element (
//                        matches.begin(), matches.end(),
//                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
//                        {
//                            return m1.distance < m2.distance;
//                        } )->distance;
//
//                feature_matches_.clear();
//                for ( cv::DMatch& m : matches )
//                {
//                    if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
//                    {
//                        feature_matches_.push_back(m);
//                    }
//                }
//                // construct the 3d 2d observations
//                vector<cv::Point3f> pts3d;
//                vector<cv::Point2f> pts2d;
//
//                std::cout<<feature_matches_.size()<<std::endl;
//                for ( cv::DMatch m:feature_matches_ )
//                {
//                    pts3d.push_back( pts_3d_ref_[m.queryIdx] );/*pts_3d_ref_ 关键帧*/
//                    pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
//                }
//
//                Mat K = ( cv::Mat_<double>(3,3)<<
//                                               ref_->camera_->fx_, 0, ref_->camera_->cx_,
//                        0, ref_->camera_->fy_, ref_->camera_->cy_,
//                        0,0,1
//                );/*相机内参*/
//                Mat rvec, tvec, inliers;
//                cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
//
//
//
//
//                //最后一步
//                last_frame_ = cur_frame_;
//                break;
//            }
//            case LOST:
//            {
//
//
//                break;
//            }
//        }
//        return true;
//    }

    bool addFrame( Frame::Ptr frame ){
        switch (this->state_) {
            case INI:
            {
                state_ = OK;
                cur_frame_ = last_frame_ = frame;
                cur_frame_->Twc_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Vector3d(0,0,0));
                break;
            }
            case OK:
            {
                cur_frame_ = frame;
                std::vector<cv::KeyPoint> keyPoints1,keyPoints2;
                std::vector<cv::DMatch> matches;
                FeatureMatch(last_frame_->color, cur_frame_->color,
                             keyPoints1, keyPoints2, matches);
                std::vector<cv::Point2f> point2d_2;
                std::vector<cv::Point3f> point3d_1;
                cout<<matches.size()<<endl;
                if (matches.size() < 30) {
                    cout<<matches.size()<<endl;
                    return false;
                }


                /***************
                 * 注意：
                 * 这里的写法，对于true_d的处理，需要首先判断深度点是否错误，然后
                 * 再判断点的深度是否大于0，这是很关键的一步。
                 *
                 *
                 * ***********/


                for (cv::DMatch m:matches) {

                    double true_d = last_frame_->findDepth(keyPoints1.at(m.queryIdx));
                    if (true_d > 0) {
                        /*注意这里是需要转移到相机坐标系中去的*/
                        Vector3d tmp = camera_->pixel2camera(
                                Vector2d(keyPoints1.at(m.queryIdx).pt.x, keyPoints1.at(m.queryIdx).pt.y), true_d);
                        point3d_1.push_back(cv::Point3f(tmp(0), tmp(1), tmp(2)));
                        point2d_2.push_back(keyPoints2.at(m.queryIdx).pt);
                    }

                }
                cv::Mat r, t, inliers;
                cv::solvePnPRansac(point3d_1, point2d_2, camera_->K_, Mat(), r, t, false, 100, 4.0, 0.99, inliers);
                int num_inliers_ = inliers.rows;

//                T_c_l_ = SE3(
//                        SO3(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0)),
//                        Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
//                );
//                cur_frame_->Twc_ = last_frame_->Twc_ * T_c_l_.inverse();


                //最后一步
                last_frame_ = cur_frame_;
                break;
            }
            case LOST:
            {


                break;
            }
        }
        return true;

    }

//    void SolvePNP(
//            const vector<KeyPoint> &keyPoints1,const vector<KeyPoint> &keyPoints2,
//            const vector<DMatch> & matches) {
//        std::vector<cv::Point2f> point2d_2;
//        std::vector<cv::Point3f> point3d_1;
//
//        for (cv::DMatch m:matches) {
//            /*注意这里的图像的存放方式*/
//            double d = last_frame_->depth.ptr<unsigned short>(int(keyPoints1.at(m.queryIdx).pt.y))[int(
//                    keyPoints1.at(m.queryIdx).pt.x)];
//            if (d == 0) {
//                break;
//            }
//            double true_d = d / 1000.0;
//            /*注意这里是需要转移到相机坐标系中去的*/
//            Point2d tmp = pixel2cam(keyPoints1.at(m.queryIdx).pt, camera_->K_);
//
//
//            double x = tmp.x;
//            double y = tmp.y;
//            point3d_1.push_back(cv::Point3f(x * true_d, y * true_d, true_d));
//            point2d_2.push_back(keyPoints2.at(m.queryIdx).pt);
//        }
//        cv::Mat r, t;
//        cv::solvePnP(point3d_1, point2d_2, camera_->K_, Mat(), r, t);
//    }




    void FeatureMatch(const cv::Mat &img1, const cv::Mat &img2,
                      std::vector<cv::KeyPoint> &keyPoints1,
                      std::vector<cv::KeyPoint> &keyPoints2,
                      std::vector<cv::DMatch> &matchs
    ) {
        /*创建匹配相关的东西*/
//        cv::Ptr<cv::FeatureDetector> dector = cv::ORB::create();
//        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
////        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
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
//        std::cout << "match size is " << match.size() << std::endl;
//        std::cout << "descriptor_1 size is " << descriptor_1.rows << std::endl;

        //选择dist
        double max_dist = 0.0, min_dist = 10000.0;

        for (int j = 0; j < match.size(); ++j) {
            if (match[j].distance > max_dist) max_dist = match[j].distance;
            if (match[j].distance < min_dist) min_dist = match[j].distance;
        }
//        std::cout << "min_dist is " << min_dist << endl;

        std::vector<cv::KeyPoint> keyP1s, keyP2s;
        for (int i = 0; i < match.size(); ++i) {
            if (match[i].distance < std::max(30.0, 2 * min_dist)) {
                matchs.push_back(match[i]);
                keyP1s.push_back(keyPoints1.at(i));
                keyP2s.push_back(keyPoints2.at(i));
            }
        }
        /*注意这里的keypoints，他的维度是500，与match的维度不一致的*/
//        cv::Mat tmp_match_pic;
//        cv::drawMatches(img1, keyPoints1, img2, keyPoints2, matchs, tmp_match_pic);
//        cv::imshow("匹配的特征点", tmp_match_pic);
//        cv::waitKey(0);
//        std::cout << "match size is " << matchs.size() << std::endl;
//        std::cout << "keyPoints1 is " << keyPoints1.size() << std::endl;
    }


    };



#endif //VO1_VIODOMETER_H
