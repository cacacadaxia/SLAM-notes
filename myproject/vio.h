//
// Created by cacacadaxia on 2020/8/17.
//

#ifndef VO1_VIO_H
#define VO1_VIO_H
#include "Common.h"


namespace slambook{
    class VisualOdometry{
    public:
        typedef shared_ptr<VisualOdometry> Ptr;/*����ָ��*/
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState     state_;     // current VO status
        Frame::Ptr  ref_;       // reference frame
        Frame::Ptr  curr_;      // current frame

        cv::Ptr<cv::ORB> orb_;  // orb detector and computer
        vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame
        vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
        Mat                     descriptors_curr_;  // descriptor in current frame
        Mat                     descriptors_ref_;   // descriptor in reference frame
        vector<cv::DMatch>      feature_matches_;

        SE3 T_c_r_estimated_;  // the estimated pose of current frame
        int num_inliers_;        // number of inlier features in icp
        int num_lost_;           // number of lost times

        // parameters
        int num_of_features_;   // number of features
        double scale_factor_;   // scale in image pyramid
        int level_pyramid_;     // number of pyramid levels
        float match_ratio_;      // ratio for selecting  good matches
        int max_num_lost_;      // max number of continuous lost times
        int min_inliers_;       // minimum inliers

        double key_frame_min_rot;   // minimal rotation of two key-frames
        double key_frame_min_trans; // minimal translation of two key-frames

    public: // functions
        VisualOdometry();
        ~VisualOdometry();

        bool addFrame( Frame::Ptr frame );      // add a new frame

    protected:
        // inner operation
        void extractKeyPoints();
        void computeDescriptors();
        void featureMatching();
        void poseEstimationPnP();
        void         setRef3DPoints();

    };

    VisualOdometry::VisualOdometry() :
            state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ),  num_lost_ ( 0 ), num_inliers_ ( 0 )
    {
        num_of_features_    = Config::get<int> ( "number_of_features" );
        scale_factor_       = Config::get<double> ( "scale_factor" );
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame ( Frame::Ptr frame )
    {
        switch ( state_ )
        {
            /*识别是不是第一*/
            case INITIALIZING:
            {
                state_ = OK;
                curr_ = ref_ = frame;/*给参考帧赋值*/
                // extract features from first frame
                extractKeyPoints();
                computeDescriptors();

                // compute the 3d position of features in ref frame
                setRef3DPoints();

                break;
            }
            case OK:
            {
                curr_ = frame;
                extractKeyPoints();
                computeDescriptors();
                featureMatching();/*特征点匹配*/
                poseEstimationPnP();
                break;
            }
            case LOST:
            {
                cout<<"vo has lost."<<endl;
                break;
            }
        }

        return true;
    }

    void VisualOdometry::extractKeyPoints()
    {
        orb_->detect ( curr_->color, keypoints_curr_ );
    }

    void VisualOdometry::computeDescriptors()
    {
        orb_->compute ( curr_->color, keypoints_curr_, descriptors_curr_ );
//        cout<<"descriptors_curr_ is "<<descriptors_curr_.rows<<endl;
    }

    void VisualOdometry::featureMatching()
    {
        // match desp_ref and desp_curr, use OpenCV's brute force match
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher ( cv::NORM_HAMMING );
        matcher.match ( descriptors_ref_, descriptors_curr_, matches );
        // select the best matches寻找不错的匹配
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        feature_matches_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }


    }

    void VisualOdometry::setRef3DPoints()
    {
        // select the features with depth measurements
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
//        cout<<"keypoints_curr_ is "<<keypoints_curr_.size()<<endl;
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = ref_->findDepth(keypoints_curr_[i]);
            if ( d > 0)
            {
                Vector3d p_cam = ref_->camera_->pixel2camera(
                        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
                );
                pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }


    void VisualOdometry::poseEstimationPnP()
    {
        // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        std::cout<<feature_matches_.size()<<std::endl;
        for ( cv::DMatch m:feature_matches_ )
        {
            pts3d.push_back( pts_3d_ref_[m.queryIdx] );/*pts_3d_ref_ 关键帧*/
            pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
        }

        Mat K = ( cv::Mat_<double>(3,3)<<
                                       ref_->camera_->fx_, 0, ref_->camera_->cx_,
                0, ref_->camera_->fy_, ref_->camera_->cy_,
                0,0,1
        );/*相机内参*/
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        num_inliers_ = inliers.rows; /*特征点数量？*/
//        // cout<<"pnp inliers: "<<num_inliers_<<endl;
//        T_c_r_estimated_ = SE3(
//                SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
//                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
//        );
        // mycout<<SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0))<<endl;
        // mycout<<tvec.at<double>(0,0)<<"\t"<<tvec.at<double>(1,0)<<"\t"<<tvec.at<double>(2,0)<<endl;

    }





}

#endif //VO1_VIO_H
