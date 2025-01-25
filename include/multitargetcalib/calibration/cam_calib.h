/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <iostream>
#include <fstream>
#include <limits>
#include <thread>
#include <mutex>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <multitargetcalib/calibration/calibration_helper.h>
#include <multitargetcalib/utils/sophus_utils.hpp>
#include <multitargetcalib/utils/apriltag.h>

namespace multitargetcalib
{

  class PosesOptimization;

  class CamCalib
  {
  public:
    CamCalib(const std::string &config_path);

    ~CamCalib();

    void detectImage(cv::Mat &image, int64_t image_timestamp, size_t cam_id);

    void startMultitargetCalibration();

  private:
    void parseConfig();

    void computeProjections();

    void initCamIntrinsics();

    bool initMultiTargetCamPoses();

    void initTargetExtrinsics();

    void initCamExtrinsics();

    bool initOptimization();

    void optimize();

    bool optimizeWithParam(bool print_info, std::map<std::string, double> *stats = nullptr);

    bool hasCorners() const;

    static constexpr size_t RANSAC_THRESHOLD = 10;

    // typedef Calibration::Ptr CalibrationPtr;

    // CalibrationPtr calib;

    CalibCornerMap calib_corners;
    CalibInitPoseMap calib_init_poses;

    MultiCalibCornerMap multi_calib_corners;
    MultiCalibInitPoseMap multi_calib_init_poses;

    std::shared_ptr<PosesOptimization> calib_opt;

    std::map<TimeCamId, ProjectedCornerData> reprojected_corners;

    std::string config_path;

    std::string target_type;

    std::shared_ptr<ApriltagDetector> april_grid;

    int skip_images;
    std::mutex calib_mutex;

    size_t cam_num;
    size_t target_num;

    std::vector<std::string> cam_types;
    std::vector<std::pair<int, int>> cam_resolutions; //[image_width, image_height]

    std::vector<Eigen::Vector4d> cam_intrinsic_init;

    std::vector<std::pair<size_t, size_t>> target_cols_rows;

    Eigen::aligned_vector<Eigen::Vector4d> aprilgrid_corner_pos_3d;
    std::vector<Eigen::aligned_vector<Eigen::Vector4d>> multi_aprilgrid_corner_pos_3d;
    std::vector<int> cornerIdStartAndEnd;

    std::vector<int64_t> image_timestamps;

    bool opt_intr;
    bool init_intrinsic_from_config;

    size_t MIN_CORNERS;
    double huber_thresh;
    double stop_thresh;

    YAML::Emitter emitter;
  };

} // namespace multitargetcalib
