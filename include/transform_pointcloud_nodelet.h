// Copyright (c) 2018, Tim Kambic
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// 	 notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// 	 notice, this list of conditions and the following disclaimer in the
// 	 documentation and/or other materials provided with the distribution.
// * Neither the name of the <organization> nor the
// 	 names of its contributors may be used to endorse or promote products
// 	 derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace transform_pointcloud
{
	class transformPointcloud final : public nodelet::Nodelet
	{
	public:
		transformPointcloud() = default;
        ~transformPointcloud() = default;

    private:
        void onInit() override;

        bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform);
        void pointcloudCB(const sensor_msgs::PointCloud::ConstPtr &cloud_in);
        void pointcloud2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud_in);
        
        // Publishers.
		ros::Publisher PointCloudPublisher;
		ros::Publisher PointCloud2Publisher;

        // Subscribers.
		ros::Subscriber PointCloudSubscriber;
		ros::Subscriber PointCloud2Subscriber;

        // Parameters.
		std::string ReferenceFrame;
        ros::Duration TransformTimeout;
        ros::Duration PollingTimeout;

        // Defaults.
        const std::string DEFAULT_REFERENCE_FRAME = "base_link";
        static constexpr double DEFAULT_TRANSFORM_TIMEOUT = 0.4;
        static constexpr double DEFAULT_POLLING_TIMEOUT = 0.1;
    };
}