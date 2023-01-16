/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "PointCloudFilter.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace mobility_base_pointcloud_filter
{

PointCloudFilter::PointCloudFilter(ros::NodeHandle nh, ros::NodeHandle priv_nh) : nh_(nh)
{
  if (!nh.getParam("width", width_)) {
    width_ = 0.14 * M_PI;
  }
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudFilter::connectCb, this);
  pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_filtered", 10, connect_cb, connect_cb);
}

PointCloudFilter::~PointCloudFilter()
{
}

void PointCloudFilter::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (!pub_.getNumSubscribers()) {
    sub_.shutdown();
  } else if (!sub_) {
    sub_ = nh_.subscribe("points", 10, &PointCloudFilter::recvCallback, this);
  }
}

static inline float normalizeAnglePositive(float angle)
{
  return fmodf(fmodf(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

void PointCloudFilter::recvCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
#if 0
  ros::WallTime tic = ros::WallTime::now();
#endif

  if (width_ == 0.0) {
    pub_.publish(msg);
    return;
  }

  const float ANGLE_WIDTH = std::min(fabs(width_), M_PI_4);
  const float ANGLE_START = M_PI_4;
  const float ANGLE_INC = M_PI_2;

  // Remove points in the four blind spots. Assumes x, y, and z are consecutive
#if 0 // atan2() is slow
  for (sensor_msgs::PointCloud2Iterator<float> it(*msg, "x"); it != it.end(); it.operator ++()) {
    const float angle = normalizeAnglePositive(atan2f(it[1], it[0]));
    for (unsigned int j = 0; j < 4; j++) {
      const float a1 = ANGLE_START + (j * ANGLE_INC) - (ANGLE_WIDTH / 2);
      const float a2 = ANGLE_START + (j * ANGLE_INC) + (ANGLE_WIDTH / 2);
      if ((a2 > angle) && (angle > a1)) {
        it[2] = NAN; // z
        break;
      }
    }
  }
#elif 1 // unit vector is ~3x faster
  const float UNIT_X_LOW = cosf(ANGLE_START + (ANGLE_WIDTH / 2));
  const float UNIT_X_HI  = cosf(ANGLE_START - (ANGLE_WIDTH / 2));
  for (sensor_msgs::PointCloud2Iterator<float> it(*msg, "x"); it != it.end(); it.operator ++()) {
    const float x = it[0];
    const float y = it[1];
    const float unit_x = fabsf(x / sqrtf(x * x + y * y));
    if ((UNIT_X_HI > unit_x) && (unit_x > UNIT_X_LOW)) {
      it[2] = NAN; // z
    }
  }
#endif

  pub_.publish(msg);

#if 0
  ros::WallTime toc = ros::WallTime::now();
  ROS_INFO("PointCloudFilter completed in %04uus", (unsigned int)((toc - tic).toNSec() / 1000));
#endif
}

} // namespace mobility_base_pointcloud_filter
