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
 * @file disparity_to_point_cloud.cpp
 *
 * Class to convert a disparity map in to a point cloud
 *
 * @author Simone Guscetti <simone@px4.io>
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

namespace d2pc {

void Disparity2PCloud::DisparityCb(const sensor_msgs::ImageConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cv_bridge::CvImageConstPtr disparity = cv_bridge::toCvShare(msg);
  cv::Size s = disparity->image.size();
  cv::Vec3f *pv;

  cv::Mat image3D(s, CV_32FC3);
  cv::reprojectImageTo3D(disparity->image, image3D, Q_);

  // cv::Mat medianFilterd = disparity->image;

  // form 40 to width-40 : to remove outlire on the border
  for (int v = 40; v < s.height - 40; v++) {
    pv = image3D.ptr<cv::Vec3f>(v);
    for (int u = 40; u < s.width - 40; u++) {
      cv::Vec3f value = pv[u];
      cloud->points.push_back(pcl::PointXYZ(value[0], value[1], value[2]));
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  //printf("Cloud size: %lu\n", cloud->points.size());
  // send point cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  // does not work on the rosbag with the tf broadcaster for the camera position
  output.header.stamp = disparity->header.stamp;
  // TODO: create ros param for this
  output.header.frame_id = "/front_optical";
  p_cloud_pub_.publish(output);
}

} /* d2pc */
