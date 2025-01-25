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
#include <vector>
#include <string>
#include <memory>
#include <thread>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <multitargetcalib/threadpool/threadpool.h>
#include <multitargetcalib/calibration/cam_calib.h>

int main(int argc, char **argv)
{
  // Check if the correct number of command-line arguments is provided
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <config_file>" << std::endl;
    return 1; // Return an error code
  }

  // Extract the command-line arguments
  std::string bag_file = argv[1];
  std::string config_file = argv[2];

  // List of image topics to extract
  std::vector<std::string> image_topics = {"/cam0/image_raw", "/cam1/image_raw"};
  // std::vector<std::string> image_topics = {"/cam1/image_raw"};

  multitargetcalib::CamCalib camcalib(config_file);

  // Open the bag file
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::thread> threads;

  // Process messages in the bag file
  rosbag::View view(bag, rosbag::TopicQuery(image_topics));
  for (rosbag::MessageInstance const &msg : view)
  {
    sensor_msgs::Image::ConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
    if (image_msg != NULL)
    {
      size_t cam_id = std::distance(image_topics.begin(), std::find(image_topics.begin(), image_topics.end(), msg.getTopic()));
      // Convert the ROS image message to a cv::Mat
      cv::Mat image = cv_bridge::toCvCopy(image_msg, "mono8")->image;

      if (!image.empty())
      {
        // cv::imshow("image", image);
        // cv::waitKey(10);

        // Get the image timestamp
        int64_t image_timestamp = image_msg->header.stamp.toNSec();

        // Call the detectImage function
        camcalib.detectImage(image, image_timestamp, cam_id);

        // threads.push_back(std::thread([&camcalib, &image, image_timestamp, cam_id]()
        //                               { camcalib.detectImage(image, image_timestamp, cam_id); }));
      }
      else
      {
        std::cout << "image is empty" << std::endl;
      }
    }
  }

  // for (auto &thread : threads)
  // {
  //   thread.join();
  // }

  camcalib.startMultitargetCalibration();

  // Close the bag file
  bag.close();

  return 0;
}
