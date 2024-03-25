#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cstdlib>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
{
	if (quaternions.size() == 0)
	{
		std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
		return Eigen::Vector4f::Zero();
	}

	// first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
	Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

	for (int q=0; q<quaternions.size(); ++q)
		A += quaternions[q] * quaternions[q].transpose();

	// normalise with the number of quaternions
	A /= quaternions.size();

	// Compute the SVD of this 4x4 matrix
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::VectorXf singularValues = svd.singularValues();
	Eigen::MatrixXf U = svd.matrixU();

	// find the eigen vector corresponding to the largest eigen value
	int largestEigenValueIndex;
	float largestEigenValue;
	bool first = true;

	for (int i=0; i<singularValues.rows(); ++i)
	{
		if (first)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
			first = false;
		}
		else if (singularValues(i) > largestEigenValue)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
		}
	}
	Eigen::Vector4f average;
	average(0) = U(0, largestEigenValueIndex);
	average(1) = U(1, largestEigenValueIndex);
	average(2) = U(2, largestEigenValueIndex);
	average(3) = U(3, largestEigenValueIndex);

	return average;
}

class SubscribeAndPublish
{
public:
    SubscribeAndPublish();
    void callback(const sensor_msgs::Image::ConstPtr& img_msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    std::string body_link;
    std::string camera_link;
    Eigen::Vector3d translation_acc;
    std::vector<Eigen::Vector4f> quaternion_acc;
    int count = 0;
};

SubscribeAndPublish::SubscribeAndPublish()
{
    //从launch文件中获取监听坐标系
    if(!(nh.getParam("/camera_link", camera_link)))
    {
        ROS_ERROR("Failed to get param 'camera_link'");
        ros::shutdown();
    }
    if(!(nh.getParam("/body_link", body_link)))
    {
        ROS_ERROR("Failed to get param 'body_link'");
        ros::shutdown();
    }
    Eigen::Vector3d translation_acc(0, 0, 0);
    sub = nh.subscribe("/tag_detections_image", 100, &SubscribeAndPublish::callback, this);

    ros::spin();
}

void SubscribeAndPublish::callback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    count++;
    //监听相机坐标系与机体坐标系之间的tf变换关系
    tf::StampedTransform tf_bc;
    try
    {
        //监听rmtt机体与相机的tf
        listener.waitForTransform(body_link, camera_link, img_msg->header.stamp, ros::Duration(0.1));
        listener.lookupTransform(body_link, camera_link, ros::Time(0), tf_bc);//transform
        //计算平移和旋转关系（RPY）
        Eigen::Vector3d t;
        t << tf_bc.getOrigin().x(), tf_bc.getOrigin().y(), tf_bc.getOrigin().z();
        Eigen::Vector4f q;
        q(0) = tf_bc.getRotation().x();
        q(1) = tf_bc.getRotation().y();
        q(2) = tf_bc.getRotation().z();
        q(3) = tf_bc.getRotation().w();
        q.normalized();

        translation_acc += t;
        Eigen::Vector3d translation_avg = translation_acc / count;

        quaternion_acc.push_back(q);
        Eigen::Vector4f quaternion_avg = quaternionAverage(quaternion_acc);
        
        std::cout << "T = " << translation_avg.transpose() << std::endl;
        std::cout << "Q = " << quaternion_avg.transpose() << std::endl;

    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rt_calibration_node");
    SubscribeAndPublish SAP;
    return 0;
}