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
#include <unordered_set>
#include <multitargetcalib/calibration/cam_calib.h>
#include <multitargetcalib/optimization/poses_optimize.h>

namespace multitargetcalib
{

  CamCalib::CamCalib(const std::string &config_path)
      : config_path(config_path)
  {
    parseConfig();
  }

  CamCalib::~CamCalib()
  {
  }

  void CamCalib::parseConfig()
  {
    std::ifstream file(config_path);
    YAML::Node config = YAML::Load(file);

    opt_intr = config["opt_intr"].as<bool>();
    init_intrinsic_from_config = config["init_intrinsic_from_config"].as<bool>();
    MIN_CORNERS = config["MIN_CORNERS"].as<int>();
    huber_thresh = config["huber_thresh"].as<double>();
    stop_thresh = config["stop_thresh"].as<double>();

    // 解析cam节点
    cam_num = config["cam_num"].as<int>();
    std::cout << "cam_num: " << cam_num << std::endl;

    YAML::Node cam = config["cam"];
    for (size_t cam_id = 0; cam_id < cam_num; ++cam_id)
    {
      std::string cam_name = "cam" + std::to_string(cam_id);
      YAML::Node cam_node = cam[cam_name];

      cam_types.push_back(cam_node["cam_type"].as<std::string>());

      int image_width = cam_node["image_width"].as<int>();
      int image_height = cam_node["image_height"].as<int>();
      cam_resolutions.push_back(std::make_pair(image_width, image_height));

      YAML::Node camera_intrinsic = cam_node["camera_intrinsic"];
      Eigen::Vector4d cam_intrinsic_tmp;

      for (std::size_t i = 0; i < camera_intrinsic.size(); ++i)
      {
        cam_intrinsic_tmp[i] = camera_intrinsic[i].as<double>();
      }
      cam_intrinsic_init.push_back(cam_intrinsic_tmp);
    }

    // 解析target节点
    target_num = config["target_num"].as<int>();
    std::cout << "target_num: " << target_num << std::endl;
    target_type = config["target_type"].as<std::string>();
    YAML::Node target = config["target"];
    for (size_t target_id = 0; target_id < target_num; target_id++)
    {
      std::string target_name = "target" + std::to_string(target_id);
      YAML::Node target_node = target[target_name];

      size_t tagCols = target_node["tagCols"].as<int>();
      size_t tagRows = target_node["tagRows"].as<int>();
      double tagSize = target_node["tagSize"].as<double>();
      double tagSpacing = target_node["tagSpacing"].as<double>();
      int tagIdStart = target_node["tagIdStart"].as<int>();
      april_grid = std::make_shared<ApriltagDetector>(tagRows, tagCols, tagSize, tagSpacing);
      target_cols_rows.push_back(std::make_pair(april_grid->getTagCols(), april_grid->getTagRows()));
      aprilgrid_corner_pos_3d = april_grid->getAprilgridCornerPos3d();
      multi_aprilgrid_corner_pos_3d.push_back(aprilgrid_corner_pos_3d);
      cornerIdStartAndEnd.push_back(tagIdStart * 4);                                    // start
      cornerIdStartAndEnd.push_back((tagIdStart * 4) + aprilgrid_corner_pos_3d.size()); // end
    }
  }

  void CamCalib::detectImage(cv::Mat &image, int64_t image_timestamp, size_t cam_id)
  {
    // std::lock_guard<std::mutex> lock(calib_mutex);

    CalibCornerData ccd_good;

    april_grid->detectTags(image, ccd_good.corners, ccd_good.corner_ids);

    std::vector<CalibCornerData> ccd_good_tmp;
    ccd_good_tmp.resize(target_num);

    for (size_t i = 0; i < ccd_good.corner_ids.size(); i++)
    {
      for (size_t j = 0; j < cornerIdStartAndEnd.size(); j += 2)
      {
        if (ccd_good.corner_ids[i] >= cornerIdStartAndEnd[j] && ccd_good.corner_ids[i] < cornerIdStartAndEnd[j + 1])
        {
          ccd_good_tmp[j / 2].corners.emplace_back(ccd_good.corners[i]);
          ccd_good_tmp[j / 2].corner_ids.emplace_back(ccd_good.corner_ids[i] - cornerIdStartAndEnd[j]);
        }
      }
    }

    for (size_t i = 0; i < ccd_good_tmp.size(); i++)
    {
      if (ccd_good_tmp[i].corners.size() > MIN_CORNERS)
      {
        std::cout << "image (" << image_timestamp << ",cam_id "
                  << cam_id << " taget_id= " << i
                  << " )  detected " << ccd_good_tmp[i].corners.size() << "/" << ccd_good.corners.size()
                  << std::endl;
        TimeCamIdTargetId tcidtargetid(image_timestamp, cam_id, i);
        multi_calib_corners.emplace(tcidtargetid, ccd_good_tmp[i]);
      }
    }
    // std::cout << "multi_calib_corners.size()=" << multi_calib_corners.size() << std::endl;
    image_timestamps.push_back(image_timestamp);
  }

  void CamCalib::startMultitargetCalibration()
  {
    std::cout << "Start multitarget camera calibration" << std::endl;

    std::cout << "start image_timestamps.size()= " << image_timestamps.size() << std::endl;

    std::sort(image_timestamps.begin(), image_timestamps.end());

    auto last = std::unique(image_timestamps.begin(), image_timestamps.end());

    image_timestamps.erase(last, image_timestamps.end());

    std::cout << "end image_timestamps.size()= " << image_timestamps.size() << std::endl;

    if (!calib_opt)
      calib_opt.reset(new PosesOptimization);
    calib_opt->resetCalib(cam_num, target_num, cam_types);
    if (!init_intrinsic_from_config)
    {
      initCamIntrinsics();
    }
    else
    {
      for (size_t cam_id = 0; cam_id < cam_num; ++cam_id)
      {
        calib_opt->calib->intrinsics[cam_id].setFromInit(cam_intrinsic_init[cam_id]);
      }
      std::cout << "Camera intrinsics initialization from config:" << std::endl;
      for (size_t cam_id = 0; cam_id < cam_num; cam_id++)
      {
        std::cout << "Cam " << cam_id << ": "
                  << calib_opt->calib->intrinsics[cam_id].getParam().transpose()
                  << std::endl;
      }
    }
    if (initMultiTargetCamPoses())
    {
      std::cout << "initMultiTargetCamPoses success" << std::endl;
    }
    else
    {
      std::cout << "initCamPoses fail" << std::endl;
    }
    initTargetExtrinsics();

    initCamExtrinsics();

    if (initOptimization())
    {
      std::cout << "initOptimization success" << std::endl;
    }
    else
    {
      std::cout << "initOptimization fail" << std::endl;
    }
    optimize();
    std::cout << "Multitarget camera calibration end" << std::endl;
  }

  void CamCalib::computeProjections()
  {
    reprojected_corners.clear();

    if (!calib_opt.get())
      return;

    constexpr int ANGLE_BIN_SIZE = 2;
    std::vector<Eigen::Matrix<double, 180 / ANGLE_BIN_SIZE, 1>> polar_sum(
        calib_opt->calib->intrinsics.size());
    std::vector<Eigen::Matrix<int, 180 / ANGLE_BIN_SIZE, 1>> polar_num(
        calib_opt->calib->intrinsics.size());

    std::vector<Eigen::Matrix<double, 360 / ANGLE_BIN_SIZE, 1>> azimuth_sum(
        calib_opt->calib->intrinsics.size());
    std::vector<Eigen::Matrix<int, 360 / ANGLE_BIN_SIZE, 1>> azimuth_num(
        calib_opt->calib->intrinsics.size());

    for (size_t i = 0; i < calib_opt->calib->intrinsics.size(); i++)
    {
      polar_sum[i].setZero();
      polar_num[i].setZero();
      azimuth_sum[i].setZero();
      azimuth_num[i].setZero();
    }

    for (size_t j = 0; j < image_timestamps.size(); ++j)
    {
      int64_t timestamp_ns = image_timestamps[j];

      for (size_t i = 0; i < calib_opt->calib->intrinsics.size(); i++)
      {
        TimeCamId tcid(timestamp_ns, i);

        ProjectedCornerData rc, rv;
        Eigen::aligned_vector<Eigen::Vector2d> polar_azimuthal_angle;

        Sophus::SE3d T_c_w_ =
            (calib_opt->getT_w_i(timestamp_ns) * calib_opt->calib->T_i_c[i])
                .inverse();

        Eigen::Matrix4d T_c_w = T_c_w_.matrix();

        calib_opt->calib->intrinsics[i].project(
            aprilgrid_corner_pos_3d, T_c_w, rc.corners_proj,
            rc.corners_proj_success, polar_azimuthal_angle);

        // calib_opt->calib->intrinsics[i].project(
        //     aprilgrid_vignette_pos_3d, T_c_w, rv.corners_proj,
        //     rv.corners_proj_success);

        reprojected_corners.emplace(tcid, rc);

        // Compute reprojection histogrames over polar and azimuth angle
        auto it = calib_corners.find(tcid);
        if (it != calib_corners.end())
        {
          for (size_t k = 0; k < it->second.corners.size(); k++)
          {
            size_t id = it->second.corner_ids[k];

            if (rc.corners_proj_success[id])
            {
              double error = (it->second.corners[k] - rc.corners_proj[id]).norm();

              size_t polar_bin =
                  180 * polar_azimuthal_angle[id][0] / (M_PI * ANGLE_BIN_SIZE);

              polar_sum[tcid.cam_id][polar_bin] += error;
              polar_num[tcid.cam_id][polar_bin] += 1;

              size_t azimuth_bin =
                  180 / ANGLE_BIN_SIZE + (180.0 * polar_azimuthal_angle[id][1]) /
                                             (M_PI * ANGLE_BIN_SIZE);

              azimuth_sum[tcid.cam_id][azimuth_bin] += error;
              azimuth_num[tcid.cam_id][azimuth_bin] += 1;
            }
          }
        }
      }
    }
  }

  void CamCalib::initCamIntrinsics()
  {
    if (multi_calib_corners.empty())
    {
      std::cerr << "No corners detected. Press detect_corners to start corner "
                   "detection."
                << std::endl;
      return;
    }

    std::cout << "Started camera intrinsics initialization" << std::endl;

    std::vector<bool> cam_initialized(cam_num, false);

    std::map<std::pair<size_t, size_t>, int> cam_target_corners_num;

    for (size_t cam_id = 0; cam_id < cam_num; cam_id++)
    {
      for (size_t target_id = 0; target_id < target_num; target_id++)
      {
        for (size_t i = 0; i < image_timestamps.size(); i++)
        {
          int64_t timestamp_ns = image_timestamps[i];

          TimeCamIdTargetId tcidtargetid(timestamp_ns, cam_id, target_id);

          auto it = multi_calib_corners.find(tcidtargetid);
          if (it != multi_calib_corners.end())
          {
            std::pair<size_t, size_t> cam_target = std::make_pair(cam_id, target_id);
            cam_target_corners_num[cam_target] += it->second.corners.size();
          }
        }
      }
    }

    int inc = 1;
    if (image_timestamps.size() > 100)
      inc = 3;

    for (size_t j = 0; j < cam_num; j++)
    {
      size_t best_target_id = -1;
      int max_corners = -1;
      for (size_t target_id = 0; target_id < target_num; target_id++)
      {
        auto it = cam_target_corners_num.find(std::make_pair(j, target_id));
        if (it != cam_target_corners_num.end())
        {
          int corners_count = it->second;
          if (corners_count > max_corners)
          {
            max_corners = corners_count;
            best_target_id = target_id;
          }
        }
      }
      std::cout << "cam " << j << " select target " << best_target_id << " to camera intrinsics initialization " << std::endl;

      for (size_t i = 0; i < image_timestamps.size();
           i += inc)
      {
        const int64_t timestamp_ns = image_timestamps[i];

        TimeCamIdTargetId tcidtargetid(timestamp_ns, j, best_target_id);

        if (multi_calib_corners.find(tcidtargetid) != multi_calib_corners.end())
        {
          CalibCornerData cid = multi_calib_corners.at(tcidtargetid);

          Eigen::Vector4d init_intr;

          bool success = CalibHelper::initializeIntrinsics(
              cid.corners, cid.corner_ids, multi_aprilgrid_corner_pos_3d[best_target_id],
              target_cols_rows[best_target_id].first, target_cols_rows[best_target_id].second, cam_resolutions[j].first,
              cam_resolutions[j].second, init_intr);

          if (success)
          {
            cam_initialized[j] = true;
            calib_opt->calib->intrinsics[j].setFromInit(init_intr);
            break;
          }
        }
      }
    }

    // Try perfect pinhole initialization for cameras that are not initalized.
    for (size_t j = 0; j < cam_num; j++)
    {
      if (!cam_initialized[j])
      {

        size_t best_target_id = -1;
        int max_corners = -1;
        for (size_t target_id = 0; target_id < target_num; target_id++)
        {
          auto it = cam_target_corners_num.find(std::make_pair(j, target_id));
          if (it != cam_target_corners_num.end())
          {
            int corners_count = it->second;
            if (corners_count > max_corners)
            {
              max_corners = corners_count;
              best_target_id = target_id;
            }
          }
        }
        std::cout << "cam " << j << " select target " << best_target_id << " to camera intrinsics initialization " << std::endl;

        std::vector<CalibCornerData *> pinhole_corners;
        int w = 0;
        int h = 0;

        for (size_t i = 0; i < image_timestamps.size();
             i += inc)
        {
          const int64_t timestamp_ns = image_timestamps[i];

          TimeCamIdTargetId tcidtargetid(timestamp_ns, j, best_target_id);

          auto it = multi_calib_corners.find(tcidtargetid);
          if (it != multi_calib_corners.end())
          {
            if (it->second.corners.size() > 8)
            {
              pinhole_corners.emplace_back(&it->second);
            }
          }

          w = cam_resolutions[j].first;
          h = cam_resolutions[j].second;
        }

        BASALT_ASSERT(w > 0 && h > 0);

        Eigen::Vector4d init_intr;

        bool success = CalibHelper::initializeIntrinsicsPinhole(
            pinhole_corners, multi_aprilgrid_corner_pos_3d[best_target_id], w, h, init_intr);

        if (success)
        {
          cam_initialized[j] = true;

          std::cout << "Initialized camera " << j
                    << " with pinhole model. You should set pinhole model for "
                       "this camera!"
                    << std::endl;
          calib_opt->calib->intrinsics[j].setFromInit(init_intr);
        }
      }
    }

    std::cout << "Done camera intrinsics initialization:" << std::endl;
    for (size_t j = 0; j < cam_num; j++)
    {
      std::cout << "Cam " << j << ": "
                << calib_opt->calib->intrinsics[j].getParam().transpose()
                << std::endl;
    }
  }

  bool CamCalib::initMultiTargetCamPoses()
  {
    if (multi_calib_corners.empty())
    {
      std::cerr << "No corners detected. Press detect_corners to start corner "
                   "detection."
                << std::endl;
      return false;
    }

    if (!calib_opt.get() || !calib_opt->calibInitialized())
    {
      std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                   "intrinsics"
                << std::endl;
      return false;
    }

    std::cout << "Started initial multitarget camera pose computation " << std::endl;

    CalibHelper::initMultiTargetCamPoses(calib_opt->calib,
                                         multi_aprilgrid_corner_pos_3d,
                                         this->multi_calib_corners, this->multi_calib_init_poses);

    std::cout << "Done initial multitarget camera pose computation. " << std::endl;

    return true;
  }

  void CamCalib::initTargetExtrinsics()
  {
    if (multi_calib_init_poses.empty())
    {
      std::cerr << "No initial multitarget camera poses. Press init_multitarget_cam_poses initialize "
                   "multitarget camera poses "
                << std::endl;
      return;
    }

    if (!calib_opt.get() || !calib_opt->calibInitialized())
    {
      std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                   "intrinsics"
                << std::endl;
      return;
    }

    // select camera to initialize target extrinsics
    size_t best_camera_id = 0;
    size_t maxCount = -1;
    std::unordered_map<size_t, size_t> cameraElementCount;
    for (size_t cam_i = 0; cam_i < cam_num; cam_i++)
    {
      for (size_t i = 0; i < image_timestamps.size(); i++)
      {
        int64_t timestamp_ns = image_timestamps[i];

        for (size_t target_i = 0; target_i < target_num; target_i++)
        {
          TimeCamIdTargetId tcidtargetid(timestamp_ns, cam_i, target_i);

          auto it = multi_calib_init_poses.find(tcidtargetid);
          if (it != multi_calib_init_poses.end())
          {
            size_t cameraId = cam_i;
            cameraElementCount[cameraId]++;
          }
        }
      }
    }
    // Iterate through the unordered_map
    for (const auto &pair : cameraElementCount)
    {
      size_t cameraId = pair.first;
      size_t count = pair.second;

      // Update maxCount and maxCameraId if a larger count is found
      if (count > maxCount)
      {
        maxCount = count;
        best_camera_id = cameraId;
      }
    }
    std::cout << " Select Camera Id: " << best_camera_id << " to init target extrinsics" << std::endl;

    // Camera graph. Stores the edge from i to j with weight w and timestamp. i
    // and j should be sorted;
    std::map<std::pair<size_t, size_t>, std::pair<int, int64_t>> target_graph;

    // Construct the graph.
    for (size_t i = 0; i < image_timestamps.size(); i++)
    {
      int64_t timestamp_ns = image_timestamps[i];

      for (size_t target_i = 0; target_i < target_num; target_i++)
      {
        TimeCamIdTargetId tcidtargetid_i(timestamp_ns, best_camera_id, target_i);

        auto it = multi_calib_init_poses.find(tcidtargetid_i);
        if (it == multi_calib_init_poses.end() || it->second.num_inliers < MIN_CORNERS)
          continue;

        for (size_t target_j = target_i + 1; target_j < target_num;
             target_j++)
        {
          TimeCamIdTargetId tcidtargetid_j(timestamp_ns, best_camera_id, target_j);

          auto it2 = multi_calib_init_poses.find(tcidtargetid_j);
          if (it2 == multi_calib_init_poses.end() ||
              it2->second.num_inliers < MIN_CORNERS)
            continue;

          std::pair<size_t, size_t> edge_id(target_i, target_j);

          int curr_weight = target_graph[edge_id].first;
          int new_weight =
              std::min(it->second.num_inliers, it2->second.num_inliers);

          if (curr_weight < new_weight)
          {
            target_graph[edge_id] = std::make_pair(new_weight, timestamp_ns);
          }
        }
      }
    }
    std::vector<bool> targets_initialized(target_num, false);
    targets_initialized[0] = true;
    size_t last_target = 0;
    calib_opt->calib->T_a_w[0] = Sophus::SE3d(); // Identity

    auto next_max_weight_edge = [&](size_t target_id)
    {
      int max_weight = -1;
      std::pair<int, int64_t> res(-1, -1);

      for (size_t i = 0; i < target_num; i++)
      {
        if (targets_initialized[i])
          continue;

        std::pair<size_t, size_t> edge_id;

        if (i < target_id)
        {
          edge_id = std::make_pair(i, target_id);
        }
        else if (i > target_id)
        {
          edge_id = std::make_pair(target_id, i);
        }

        auto it = target_graph.find(edge_id);
        if (it != target_graph.end() && max_weight < it->second.first)
        {
          max_weight = it->second.first;
          res.first = i;
          res.second = it->second.second;
        }
      }
      return res;
    };

    for (size_t i = 0; i < target_num - 1; i++)
    {
      std::pair<int, int64_t> res = next_max_weight_edge(last_target);

      std::cout << "Initializing target pair " << last_target << " " << res.first
                << std::endl;

      if (res.first >= 0)
      {
        size_t new_target = res.first;

        TimeCamIdTargetId tcidtargetid_last(res.second, best_camera_id, last_target);
        TimeCamIdTargetId tcidtargetid_new(res.second, best_camera_id, new_target);

        calib_opt->calib->T_a_w[new_target] =
            multi_calib_init_poses.at(tcidtargetid_new).T_a_c * multi_calib_init_poses.at(tcidtargetid_last).T_a_c.inverse() *
            calib_opt->calib->T_a_w[last_target];

        last_target = new_target;
        targets_initialized[last_target] = true;
      }
    }

    std::cout << "Done target extrinsics initialization:" << std::endl;
    for (size_t j = 0; j < target_num; j++)
    {
      std::cout << "T_a" << j << "_a0"
                << ":\n"
                << calib_opt->calib->T_a_w[j].matrix() << std::endl;
    }
  }

  void CamCalib::initCamExtrinsics()
  {
    if (multi_calib_init_poses.empty())
    {
      std::cerr << "No initial camera poses. Press init_cam_poses initialize "
                   "camera poses "
                << std::endl;
      return;
    }

    if (!calib_opt.get() || !calib_opt->calibInitialized())
    {
      std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                   "intrinsics"
                << std::endl;
      return;
    }

    // Camera graph. Stores the edge from i to j with weight w and timestamp. i
    // and j should be sorted; pair<cam_id,target_id>
    std::map<std::pair<size_t, size_t>, std::pair<int, int64_t>> cam_graph;
    std::map<std::pair<size_t, size_t>, std::pair<size_t, size_t>> target_graph;

    for (size_t i = 0; i < image_timestamps.size(); i++)
    {
      int64_t timestamp_ns = image_timestamps[i];

      for (size_t cam_i = 0; cam_i < cam_num; cam_i++)
      {
        for (size_t target_i = 0; target_i < target_num; target_i++)
        {
          // 构造相机和标定板索引对应的TimeCamIdTargetId
          TimeCamIdTargetId tcidtargetid_i(timestamp_ns, cam_i, target_i);

          auto it = multi_calib_init_poses.find(tcidtargetid_i);
          if (it == multi_calib_init_poses.end() || it->second.num_inliers < MIN_CORNERS)
            continue;

          for (size_t cam_j = cam_i + 1; cam_j < cam_num; cam_j++)
          {
            for (size_t target_j = 0; target_j < target_num; target_j++)
            {
              // 构造相机和标定板索引对应的TimeCamIdTargetId
              TimeCamIdTargetId tcidtargetid_j(timestamp_ns, cam_j, target_j);

              auto it2 = multi_calib_init_poses.find(tcidtargetid_j);
              if (it2 == multi_calib_init_poses.end() || it2->second.num_inliers < MIN_CORNERS)
                continue;

              std::pair<size_t, size_t> edge_id(cam_i, cam_j);
              std::pair<size_t, size_t> cam_to_target_id(target_i, target_j);

              int curr_weight = cam_graph[edge_id].first;
              int new_weight = std::min(it->second.num_inliers, it2->second.num_inliers);

              if (curr_weight < new_weight)
              {
                cam_graph[edge_id] = std::make_pair(new_weight, timestamp_ns);
                target_graph[edge_id] = cam_to_target_id;
              }
            }
          }
        }
      }
    }
    std::vector<bool> cameras_initialized(cam_num, false);
    cameras_initialized[0] = true;
    size_t last_camera = 0;
    calib_opt->calib->T_i_c[0] = Sophus::SE3d(); // Identity

    auto next_max_weight_edge = [&](size_t cam_id)
    {
      int max_weight = -1;
      std::pair<int, int64_t> res(-1, -1);

      for (size_t i = 0; i < cam_num; i++)
      {
        if (cameras_initialized[i])
          continue;

        std::pair<size_t, size_t> edge_id;

        if (i < cam_id)
        {
          edge_id = std::make_pair(i, cam_id);
        }
        else if (i > cam_id)
        {
          edge_id = std::make_pair(cam_id, i);
        }

        auto it = cam_graph.find(edge_id);
        if (it != cam_graph.end() && max_weight < it->second.first)
        {
          max_weight = it->second.first;
          res.first = i;
          res.second = it->second.second;
        }
      }
      return res;
    };

    for (size_t i = 0; i < cam_num - 1; i++)
    {
      std::pair<int, int64_t> res = next_max_weight_edge(last_camera);

      std::cout << "Initializing camera pair " << last_camera << " " << res.first
                << std::endl;

      std::pair<size_t, size_t> cam_to_target_id;
      std::pair<size_t, size_t> cam_to_cam_id;

      if (last_camera < size_t(res.first))
      {
        cam_to_cam_id = std::make_pair(last_camera, res.first);
      }
      else
      {
        cam_to_cam_id = std::make_pair(res.first, last_camera);
      }

      auto it = target_graph.find(cam_to_cam_id);

      if (res.first >= 0 && it != target_graph.end())
      {
        size_t new_camera = res.first;

        TimeCamIdTargetId tcidtargetid_last(res.second, last_camera, it->second.first);
        TimeCamIdTargetId tcidtargetid_new(res.second, new_camera, it->second.second);

        calib_opt->calib->T_i_c[new_camera] =
            calib_opt->calib->T_i_c[last_camera] *
            multi_calib_init_poses.at(tcidtargetid_last).T_a_c.inverse() * calib_opt->calib->T_a_w[it->second.first] *
            calib_opt->calib->T_a_w[it->second.second].inverse() * multi_calib_init_poses.at(tcidtargetid_new).T_a_c;

        std::cout << "Initializing camera target pair (" << last_camera << "->" << it->second.first << " ) ("
                  << res.first << " ->" << it->second.second << " )"
                  << std::endl;

        last_camera = new_camera;
        cameras_initialized[last_camera] = true;
      }
    }

    std::cout << "Done camera extrinsics initialization:" << std::endl;
    for (size_t j = 0; j < cam_num; j++)
    {
      std::cout << "T_c0_c" << j << ":\n"
                << calib_opt->calib->T_i_c[j].matrix() << std::endl;
    }

    // for (size_t cam_id = 0; cam_id < cam_num; cam_id++)
    // {
    //   std::cout << "T_c" << cam_id << "_c0"
    //             << ":\n"
    //             << calib_opt->calib->T_i_c[cam_id].matrix().inverse() << std::endl;

    //   if (cam_id > 0)
    //   {
    //     std::string tmp = "T_init";
    //     emitter << YAML::BeginMap;
    //     emitter << YAML::Key << tmp << YAML::Value << YAML::Flow << YAML::BeginSeq;
    //     for (int i = 0; i < 4; ++i)
    //     {
    //       for (int j = 0; j < 4; ++j)
    //       {
    //         emitter << calib_opt->calib->T_i_c[cam_id].matrix().inverse()(i, j);
    //       }
    //     }
    //     emitter << YAML::EndSeq;
    //     emitter << YAML::EndMap;
    //   }
    // }
  }

  bool CamCalib::initOptimization()
  {
    if (!calib_opt)
    {
      std::cerr << "Calibration is not initialized. Initialize calibration first!"
                << std::endl;
      return false;
    }

    if (multi_calib_init_poses.empty())
    {
      std::cerr << "No initial multitarget camera poses. Press init_multi_target_cam_poses initialize "
                   "multitarget camera poses "
                << std::endl;
      return false;
    }

    calib_opt->setMultiAprilgridCorners3d(multi_aprilgrid_corner_pos_3d);
    calib_opt->setAprilgridCorners3d(aprilgrid_corner_pos_3d);

    std::unordered_set<TimeCamIdTargetId> invalid_frames;
    for (const auto &kv : multi_calib_corners)
    {
      if (kv.second.corner_ids.size() < MIN_CORNERS)
        invalid_frames.insert(kv.first);
    }

    int num_1 = 0;
    int num_2 = 0;

    for (size_t j = 0; j < image_timestamps.size(); ++j)
    {
      int64_t timestamp_ns = image_timestamps[j];

      int max_inliers = -1;
      int max_inliers_cam_idx = -1;
      int max_inliers_target_idx = -1;

      for (size_t cam_id = 0; cam_id < calib_opt->calib->T_i_c.size(); cam_id++)
      {
        for (size_t target_id = 0; target_id < calib_opt->calib->T_a_w.size(); target_id++)
        {
          TimeCamIdTargetId tcidtargetid(timestamp_ns, cam_id, target_id);
          const auto cp_it = multi_calib_init_poses.find(tcidtargetid);
          if (cp_it != multi_calib_init_poses.end())
          {
            if ((int)cp_it->second.num_inliers > max_inliers)
            {
              max_inliers = cp_it->second.num_inliers;
              max_inliers_cam_idx = cam_id;
              max_inliers_target_idx = target_id;
            }
          }
        }
      }

      if (max_inliers >= (int)MIN_CORNERS)
      {
        TimeCamIdTargetId tcidtargetid(timestamp_ns, max_inliers_cam_idx, max_inliers_target_idx);
        const auto cp_it = multi_calib_init_poses.find(tcidtargetid);

        // Initial pose
        calib_opt->addPoseMeasurement(timestamp_ns, calib_opt->calib->T_a_w[max_inliers_target_idx].inverse() * cp_it->second.T_a_c *
                                                        calib_opt->calib->T_i_c[max_inliers_cam_idx].inverse());
        num_1++;
        // std::cout << "timestamp_ns= " << timestamp_ns << "max_inliers_cam_idx= " << max_inliers_cam_idx << "max_inliers_target_idx= " << max_inliers_target_idx << std::endl;
      }
      else
      {
        // Set all frames invalid if we do not have initial pose
        for (size_t cam_id = 0; cam_id < calib_opt->calib->T_i_c.size(); cam_id++)
        {
          for (size_t target_id = 0; target_id < calib_opt->calib->T_a_w.size(); target_id++)
          {
            invalid_frames.emplace(timestamp_ns, cam_id, target_id);
          }
        }
      }
    }

    for (const auto &kv : multi_calib_corners)
    {
      if (invalid_frames.count(kv.first) == 0)
      {
        calib_opt->addMultiAprilgridMeasurement(kv.first.frame_id, kv.first.cam_id, kv.first.target_id,
                                                kv.second.corners,
                                                kv.second.corner_ids);
        num_2++;
      }
    }

    std::cout << "num_1=" << num_1 << "num_2=" << num_2 << "multi_calib_corners.size()=" << multi_calib_corners.size() << std::endl;

    calib_opt->init();
    // computeProjections();

    std::cout << "Initialized optimization." << std::endl;
    return true;
  } // namespace multitargetcalib

  void CamCalib::optimize()
  {
    bool success;
    success = optimizeWithParam(true);
    if (success)
    {
      std::cout << "Optimize Success!" << std::endl;
    }
    else
    {
      std::cout << "Optimize Failed!" << std::endl;
    }
  }

  bool CamCalib::optimizeWithParam(bool print_info,
                                   std::map<std::string, double> *stats)
  {
    if (multi_calib_init_poses.empty())
    {
      std::cerr << "No initial multitarget camera poses. Press init_multi_target_cam_poses initialize "
                   "multitarget camera poses "
                << std::endl;
      return true;
    }

    if (!calib_opt.get() || !calib_opt->calibInitialized())
    {
      std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                   "intrinsics"
                << std::endl;
      return true;
    }

    bool converged = false;
    int max_iter = 10;
    auto start = std::chrono::high_resolution_clock::now();

    while (max_iter > 0 && !converged)
    {
      // calib_opt->compute_projections();
      double error;
      double reprojection_error;
      int num_points;

      converged = calib_opt->optimize(opt_intr, huber_thresh, stop_thresh, error,
                                      num_points, reprojection_error);

      if (stats)
      {
        stats->clear();

        stats->emplace("energy_error", error);
        stats->emplace("num_points", num_points);
        stats->emplace("mean_energy_error", error / num_points);
        stats->emplace("reprojection_error", reprojection_error);
        stats->emplace("mean_reprojection_error",
                       reprojection_error / num_points);
      }

      if (print_info)
      {
        std::cout << "==================================" << std::endl;

        for (size_t i = 0; i < cam_num; i++)
        {
          std::cout << "intrinsics " << i << ": "
                    << calib_opt->calib->intrinsics[i].getParam().transpose()
                    << std::endl;
          std::cout << "T_i_c" << i
                    << ":\n"
                    << calib_opt->calib->T_i_c[i].matrix() << std::endl;
        }

        for (size_t j = 0; j < target_num; j++)
        {

          std::cout << "T_a" << j << "_a0"
                    << ":\n"
                    << calib_opt->calib->T_a_w[j].matrix() << std::endl;
        }

        std::cout << "Current error: " << error << " num_points " << num_points
                  << " mean_error " << error / num_points
                  << " reprojection_error " << reprojection_error
                  << " mean reprojection " << reprojection_error / num_points << std::endl;
      }
      max_iter--;
      std::cout << "==================================" << std::endl;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << " opt_time " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count()
              << "ms." << std::endl;

    if (converged)
    {
      std::cout << "Optimization Converged !!" << std::endl;

      for (size_t cam_id = 0; cam_id < cam_num; cam_id++)
      {
        if (cam_id > 0)
        {
          std::string tmp = "T_opt";
          emitter << YAML::BeginMap;
          emitter << YAML::Key << tmp << YAML::Value << YAML::Flow << YAML::BeginSeq;
          for (int i = 0; i < 4; ++i)
          {
            for (int j = 0; j < 4; ++j)
            {
              emitter << calib_opt->calib->T_i_c[cam_id].matrix().inverse()(i, j);
            }
          }
          emitter << YAML::EndSeq;
          emitter << YAML::EndMap;
        }
      }

      std::ofstream fout("../output.yaml");
      fout << emitter.c_str();
    }

    return converged;
  }

  bool CamCalib::hasCorners() const
  {
    return !multi_calib_corners.empty();
  }

} // namespace multitargetcalib
