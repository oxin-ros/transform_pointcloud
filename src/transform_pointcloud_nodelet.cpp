#include <transform_pointcloud_nodelet.h>

using namespace transform_pointcloud;

void transformPointcloud::onInit()
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nhp = getPrivateNodeHandle();
	
	// Get the frame ID.
	std::string reference_frame;
	if (!nhp.getParam("to_frame", reference_frame))
	{
		ROS_ERROR("You must specify 'to_frame' parameter !");
		exit(1);
	}
	ReferenceFrame = reference_frame;

	// Get the transform timeout.
	double transform_timeout = 0.01;
	nhp.getParam("transform_timeout", transform_timeout);
	TransformTimeout = ros::Duration(transform_timeout);

	// Get the polling timeout.
	double polling_timeout = 0.01;
	nhp.getParam("polling_timeout", polling_timeout);
	PollingTimeout = ros::Duration(polling_timeout);

	// Setup the publishers.
	PointCloudPublisher = nhp.advertise<sensor_msgs::PointCloud>("output_pcl", 1);
	PointCloud2Publisher = nhp.advertise<sensor_msgs::PointCloud2>("output_pcl2", 1);
	PointCloudSubscriber = nhp.subscribe<sensor_msgs::PointCloud>("input_pcl", 1, &transformPointcloud::pointcloudCB, this);
	PointCloud2Subscriber = nhp.subscribe<sensor_msgs::PointCloud2>("input_pcl2", 1, &transformPointcloud::pointcloud2CB, this);
};

bool transformPointcloud::getTransform(
	const std::string& reference_frame, 
	const std::string& child_frame, 
	tf::StampedTransform &transform)
{
	// Retrieve the next transform message.
	std::string errMsg;
	tf::TransformListener tf_listener;
	const auto get_next_transform = ros::Time(0);
	const bool transform_retrieved = tf_listener.waitForTransform(
		reference_frame, 
		child_frame, 
		get_next_transform, 
		TransformTimeout,
		PollingTimeout, 
		&errMsg);
	if (!transform_retrieved)
	{
		ROS_ERROR_STREAM("Pointcloud transform | Unable to get pose from TF: " << errMsg);
		return false;
	}
	
	// Retrieve the transform message from the listener.
	try
	{
		tf_listener.lookupTransform(
			reference_frame, 
			child_frame, 
			get_next_transform, 
			transform);
	}
	catch (const tf::TransformException &e)
	{
		ROS_ERROR_STREAM(
			"Pointcloud transform | Error in lookupTransform of " << child_frame << " in " << reference_frame);
		return false;
	}
	return true;
}

void transformPointcloud::pointcloudCB(const sensor_msgs::PointCloud::ConstPtr &cloud_in)
{
	// Check if there are any points in the cloud.
	if (cloud_in->points.empty())
	{
		return;
	}

	// Get the transform.
	tf::StampedTransform tf_between_frames;
	getTransform(ReferenceFrame, cloud_in->header.frame_id, tf_between_frames);

	// Convert it to geometry_msg type.
	geometry_msgs::TransformStamped tf_between_frames_geo;
	tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo); 

	// Convert from pointcloud to pointcloud2.
	sensor_msgs::PointCloud2 cloud_in2;
	sensor_msgs::convertPointCloudToPointCloud2(*cloud_in, cloud_in2); 

	// Transform the point cloud.
	sensor_msgs::PointCloud2 cloud_in_transformed;
	tf2::doTransform(cloud_in2, cloud_in_transformed, tf_between_frames_geo);

	// Check if there are any subscribers.
	if (PointCloud2Publisher.getNumSubscribers() > 0)
	{ 
		// Publish pointcloud2.
		PointCloud2Publisher.publish(cloud_in_transformed);
	}

	// Check if there are any subscribers.
	if (PointCloudPublisher.getNumSubscribers() > 0)
	{
		// Convert from pointcloud to pointcloud2.
		sensor_msgs::PointCloud cloud_out;
		sensor_msgs::convertPointCloud2ToPointCloud(cloud_in_transformed, cloud_out); 

		// Publish pointcloud.
		PointCloudPublisher.publish(cloud_out);													  
	}
}

void transformPointcloud::pointcloud2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{
	// Check if there are any points in the cloud.
	if (cloud_in->data.empty())
	{
		return;
	}

	// Get the transform.
	tf::StampedTransform tf_between_frames;
	getTransform(ReferenceFrame, cloud_in->header.frame_id, tf_between_frames);

	// Convert it to geometry_msg type.
	geometry_msgs::TransformStamped tf_between_frames_geo;
	tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo);

	// Transform the point cloud.
	sensor_msgs::PointCloud2 cloud_in_transformed;
	tf2::doTransform(*cloud_in, cloud_in_transformed, tf_between_frames_geo);

	// Check if there are any subscribers.
	if (PointCloud2Publisher.getNumSubscribers() > 0)
	{
		// Publish pointcloud2.
		PointCloud2Publisher.publish(cloud_in_transformed);
	}
	
	// Check if there are any subscribers.
	if (PointCloudPublisher.getNumSubscribers() > 0)
	{
		// Convert from pointcloud to pointcloud2.
		sensor_msgs::PointCloud cloud_out;
		sensor_msgs::convertPointCloud2ToPointCloud(cloud_in_transformed, cloud_out); 

		// Publish pointcloud.
		PointCloudPublisher.publish(cloud_out);
	}
}

PLUGINLIB_EXPORT_CLASS(transform_pointcloud::transformPointcloud, nodelet::Nodelet);
