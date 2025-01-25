
#pragma once

#include <multitargetcalib/utils/sophus_utils.hpp>

#include <vector>
#include <memory>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

namespace multitargetcalib
{
  class ApriltagDetector
  {
  public:
    // target extraction options
    struct AprilgridOptions
    {
      AprilgridOptions() : doSubpixRefinement(true),
                           maxSubpixDisplacement(0),
                           minTagsForValidObs(4),
                           minBorderDistance(4.0),
                           blackTagBorder(2){};

      // options
      /// \brief subpixel refinement of extracted corners
      bool doSubpixRefinement;

      /// \brief max. displacement squarred in subpixel refinement  [px^2]
      double maxSubpixDisplacement;

      /// \brief min. number of tags for a valid observation
      unsigned int minTagsForValidObs;

      /// \brief min. distance form image border for valid points [px]
      double minBorderDistance;

      /// \brief size of black border around the tag code bits (in pixels)
      unsigned int blackTagBorder;
    };

    ApriltagDetector(size_t tagRows, size_t tagCols, double tagSize,
                     double tagSpacing, const AprilgridOptions &options = AprilgridOptions());

    ~ApriltagDetector();

    size_t getTagCols() const { return _tagCols; }
    size_t getTagRows() const { return _tagRows; }
    size_t size() { return _tagCols * _tagRows * 4; }

    Eigen::aligned_vector<Eigen::Vector4d> getAprilgridCornerPos3d() const { return _aprilgrid_corner_pos_3d; }

    // void detectTags(cv::Mat &image,
    //                 Eigen::aligned_vector<Eigen::Vector2d> &corners,
    //                 std::vector<int> &ids, std::vector<double> &radii,
    //                 Eigen::aligned_vector<Eigen::Vector2d> &corners_rejected,
    //                 std::vector<int> &ids_rejected,
    //                 std::vector<double> &radii_rejected);

    void detectTags(cv::Mat &image, Eigen::aligned_vector<Eigen::Vector2d> &corners, std::vector<int> &ids);

  private:
    /// \brief initialize the grid with the points
    void createGridPoints();

    double _tagRows;

    double _tagCols;

    /// \brief size of a tag [m]
    double _tagSize;

    /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
    double _tagSpacing;

    /// \brief target extraction options
    AprilgridOptions _options;

    // create a detector instance
    AprilTags::TagCodes _tagCodes;

    std::shared_ptr<AprilTags::TagDetector> _tagDetector;

    Eigen::aligned_vector<Eigen::Vector4d> _aprilgrid_corner_pos_3d;

    // std::mutex tagDetectorMutex;
  };

} // namespace multitargetcalib
