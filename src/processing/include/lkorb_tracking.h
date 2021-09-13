#ifndef TRACKING_H
#define TRACKING_H

//Lucas-Kanade tracking with ORB feature verify

#include "camera_frame.h"
#include "include/tic_toc_ros.h"
// GPU acclerate
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>


class LKORBTracking
{
    int width,height;
public:
    DepthCamera  d_camera;
    bool use_gpu_flow;
    LKORBTracking(int width_in,int height_in, bool use_gpu_flow=false);
    bool tracking(CameraFrame &from,
                  CameraFrame &to,
                  SE3 T_c_w_guess,
                  bool use_guess,
                  vector<cv::Point2f>& lm2d_from,
                  vector<cv::Point2f>& lm2d_to,
                  vector<cv::Point2f>& outlier);
};

#endif // F2FTRACKING_H
