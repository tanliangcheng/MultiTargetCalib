/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

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

@file
@brief Calibration datatypes for muticam-IMU and motion capture calibration
*/

#pragma once

#include <memory>
#include <multitargetcalib/camera/generic_camera.hpp>

namespace multitargetcalib
{

  /// @brief Struct to store camera-IMU calibration
  template <class Scalar>
  struct Calibration
  {
    using Ptr = std::shared_ptr<Calibration>;
    using SE3 = Sophus::SE3<Scalar>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    /// @brief Default constructor.
    Calibration() {}

    /// @brief Cast to other scalar type
    template <class Scalar2>
    Calibration<Scalar2> cast() const
    {
      Calibration<Scalar2> new_cam;

      for (const auto &v : T_i_c)
        new_cam.T_i_c.emplace_back(v.template cast<Scalar2>());
      for (const auto &v : T_a_w)
        new_cam.T_a_w.emplace_back(v.template cast<Scalar2>());
      for (const auto &v : intrinsics)
        new_cam.intrinsics.emplace_back(v.template cast<Scalar2>());
      new_cam.resolution = resolution;
      return new_cam;
    }

    /// @brief Vector of transformations from camera to IMU
    ///
    /// Point in camera coordinate frame \f$ p_c \f$ can be transformed to the
    /// point in IMU coordinate frame as \f$ p_i = T_{ic} p_c, T_{ic} \in
    /// SE(3)\f$
    Eigen::aligned_vector<SE3> T_i_c;

    /// @brief Multitaget transformation from target to target
    Eigen::aligned_vector<SE3> T_a_w;

    /// @brief Vector of camera intrinsics. Can store different camera models. See
    /// \ref GenericCamera.
    Eigen::aligned_vector<GenericCamera<Scalar>> intrinsics;

    /// @brief Camera resolutions.
    Eigen::aligned_vector<Eigen::Vector2i> resolution;
  };

} // namespace multitargetcalib
