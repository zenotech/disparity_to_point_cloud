/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file depth_map_fusion.hpp
 *
 * Class to perform a depth map fusion
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#include <disparity_to_point_cloud/camera_triplet.hpp>

namespace depth_map_fusion {

CameraTriplet::CameraTriplet(ros::NodeHandle &nh, int id_) 
        : hor_pair(nh, this, std::to_string(id_),   false),
          ver_pair(nh, this, std::to_string(id_+2), true) {

  std::string id_str = std::to_string(id_);

  fused_depth_pub = nh.advertise<sensor_msgs::Image>("/fused_depth_" + id_str, 1);
  fused_score_pub = nh.advertise<sensor_msgs::Image>("/fused_score_" + id_str, 1);
}


void CameraTriplet::fuseIfPossible(const sensor_msgs::ImageConstPtr &msg) {
  if (hor_pair.timestamps[0] == hor_pair.timestamps[1] &&
      hor_pair.timestamps[0] == ver_pair.timestamps[0] &&
      hor_pair.timestamps[0] == ver_pair.timestamps[1]) {

    fusePairs(msg);
  }
}


void CameraTriplet::fusePairs(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Mat cropped_depth_combined = hor_pair.depth_best_mat.clone();
  cv::Mat cropped_score_combined = hor_pair.score_best_mat.clone();

  // For each pixel, calculate a new distance
  for (int i = 0; i < cropped_depth_combined.rows; ++i) {
    for (int j = 0; j < cropped_depth_combined.cols; ++j) {
      DepthScore depth_score = getFusedPixel(i, j);
      cropped_depth_combined.at<unsigned char>(i, j) = depth_score.depth;
      cropped_score_combined.at<unsigned char>(i, j) = depth_score.score;
    }
  }
  // cv::medianBlur(cropped_depth_combined, cropped_depth_combined, 7);

  publishWithColor(msg, cropped_depth_combined, fused_depth_pub, RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_combined, fused_score_pub, GRAY_SCALE);
}


DepthScore CameraTriplet::getFusedPixel(int i, int j) {
  int hor_dist =  hor_pair.depth_best_mat.at<unsigned char>(i, j);
  int hor_score = hor_pair.score_best_mat.at<unsigned char>(i, j);
  int height = ver_pair.depth_best_mat.rows;
  int long_width  = hor_pair.depth_best_mat.cols;
  int short_width = ver_pair.depth_best_mat.cols;
  int i2 = i - 5;    // Alignment hack
  int j2 = j + 10 - ((long_width - short_width) / 2);
  if (j2 <= 0 || j2 >= short_width || i2 < 0 || i2 >= height) {
    return {hor_dist, hor_score};   // Pixel only exists on horizontal pair
  }

  DepthScore hor_best = {hor_dist, hor_score};
  DepthScore hor_sec = {hor_pair.depth_sec_mat.at<unsigned char>(i, j), 
                        hor_pair.score_sec_mat.at<unsigned char>(i, j)};
  DepthScore ver_best = {ver_pair.depth_best_mat.at<unsigned char>(i2, j2), 
                         ver_pair.score_best_mat.at<unsigned char>(i2, j2)};
  DepthScore ver_sec = {ver_pair.depth_sec_mat.at<unsigned char>(i2, j2), 
                        ver_pair.score_sec_mat.at<unsigned char>(i2, j2)};

  // return betterScore(hor_best, hor_sec, ver_best, ver_sec);
  return secondBestInv2(hor_best, hor_sec, ver_best, ver_sec);
  // return alwaysVer(hor_best, hor_sec, ver_best, ver_sec);
}

}  // depth_map_fusion