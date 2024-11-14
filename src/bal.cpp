#include <ros/ros.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/PointStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "datatype.h"

using namespace std;
using namespace geometry_msgs;
using namespace message_filters;

Eigen::Matrix4d to_matrix(const Eigen::Vector3d T, const Eigen::Quaterniond R_)
{
    Eigen::Matrix3d R = R_.normalized().toRotationMatrix();
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = R;
    matrix.block<3,1>(0,3) = T;
    return matrix;
}

void from_matrix(const Eigen::Matrix4d& matrix, Eigen::Vector3d& T, Eigen::Quaterniond& R)
{
//     cout << matrix << endl;
    // 提取旋转矩阵部分
    Eigen::Matrix3d rotation_matrix = matrix.block<3,3>(0,0);
    
    // 提取平移向量部分
    T = matrix.block<3,1>(0,3);

    // 将旋转矩阵转换为四元数
    R = Eigen::Quaterniond(rotation_matrix);
}

Eigen::Matrix4d to_inverse(const Eigen::Matrix4d matrix)
{
    Eigen::Matrix3d R = matrix.block<3,3>(0,0);
    Eigen::Vector3d t = matrix.block<3,1>(0,3);
    Eigen::Matrix3d R_T = R.transpose();
    Eigen::Matrix4d matrix_inv = Eigen::Matrix4d::Identity();
    matrix_inv.block<3,3>(0,0) = R_T;
    matrix_inv.block<3,1>(0,3) = -R_T * t;

    return matrix_inv;
}

struct SnavelyReprojectionErrorWithQuaternions {
  // (u, v): the position of the observation with respect to the image
  // center point.
  SnavelyReprojectionErrorWithQuaternions(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,//w,x,y,z, x,y,z
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
    //
    // We use QuaternionRotatePoint as it does not assume that the
    // quaternion is normalized, since one of the ways to run the
    // bundle adjuster is to let Ceres optimize all 4 quaternion
    // parameters without a local parameterization.
    T p[3];
    ceres::QuaternionRotatePoint(camera, point, p);

    p[0] += camera[4];//x
    p[1] += camera[5];//y
    p[2] += camera[6];//z

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    const T xp = - p[0] / p[2];
    const T yp = - p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[8];
    const T& l2 = camera[9];

    const T r2 = xp*xp + yp*yp;
    const T distortion = 1.0 + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    const T& focal = camera[7];
    const T predicted_x = focal * distortion * xp;
    const T predicted_y = focal * distortion * yp;
    
    
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    
//     cout << residuals[0] << " " << residuals[1] << endl; 
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {

    return (new ceres::AutoDiffCostFunction<
            SnavelyReprojectionErrorWithQuaternions, 2, 10, 3>(
                new SnavelyReprojectionErrorWithQuaternions(observed_x,
                                                            observed_y)));
  }

  double observed_x;
  double observed_y;
};


class RosNode
{
public:
        RosNode(ros::NodeHandle &n);
        ~RosNode() {};
        void callback(const PoseArray::ConstPtr &tag_centre_msg,
                      const PoseStamped::ConstPtr &tag_pose_msg,
                      const PoseStamped::ConstPtr &drone_pose_msg);
private:
        ceres::Problem problem;
        // Eigen::Matrix<double, 3, 4> K_in_;
        Eigen::Matrix4d K_ex_; //world -> camera
        Eigen::Matrix4d K_ex1_; //body->world
        Eigen::Matrix4d K_ex2_; //camera->body

        double K1_ = K1;
        double K2_ = K2;
        double F_ = F1;

        double Tx_ = Tx;
        double Ty_ = Ty;
        double Tz_ = Tz;
        double Qx_ = Qx;
        double Qy_ = Qy;
        double Qz_ = Qz;
        double Qw_ = Qw;

        tf2_ros::TransformBroadcaster tf_bc_wc;
        tf2_ros::TransformBroadcaster tf_bc_wb;
        tf2_ros::TransformBroadcaster tf_bc_bc;
};

RosNode::RosNode(ros::NodeHandle &n) {}

void RosNode::callback(const PoseArray::ConstPtr &tag_centre_msg,
                       const PoseStamped::ConstPtr &tag_pose_msg,
                       const PoseStamped::ConstPtr &drone_pose_msg)
{    
        cout << "\n--------------------------------" << endl;

        cv::Point2d tag_centre;
        tag_centre.x = tag_centre_msg->poses[0].position.x;
        tag_centre.y = tag_centre_msg->poses[0].position.y;
        // cout << tag_centre.x << " " << tag_centre.y << endl;

        double* tag_pose = new double[3];
        tag_pose[0] = tag_pose_msg->pose.position.x;
        tag_pose[1] = tag_pose_msg->pose.position.y;
        tag_pose[2] = tag_pose_msg->pose.position.z;

        Eigen::VectorXd drone_pose(7);
        drone_pose[4] = drone_pose_msg->pose.position.x;
        drone_pose[5] = drone_pose_msg->pose.position.y;
        drone_pose[6] = drone_pose_msg->pose.position.z;
        drone_pose[0] = drone_pose_msg->pose.orientation.w;
        drone_pose[1] = drone_pose_msg->pose.orientation.x;
        drone_pose[2] = drone_pose_msg->pose.orientation.y;
        drone_pose[3] = drone_pose_msg->pose.orientation.z;

        // body -> world
        Eigen::Vector3d T1(drone_pose[4], drone_pose[5], drone_pose[6]);
        Eigen::Quaterniond Q1(drone_pose[0], drone_pose[1], drone_pose[2], drone_pose[3]);
        K_ex1_ = to_matrix(T1, Q1);

        // camera -> body
        Eigen::Vector3d T2(Tx, Ty, Tz);
        Eigen::Quaterniond Q2(Qw, Qx, Qy, Qz);
        K_ex2_ = to_matrix(T2, Q2);

        // camera -> world
        K_ex_ = K_ex1_ * K_ex2_;

        // world -> camera
        // Eigen::Matrix4d K_ex1_inv = to_inverse(this->K_ex1_);//world -> body
        // Eigen::Matrix4d K_ex2_inv = to_inverse(this->K_ex2_);//body -> camera
        // K_ex_ = K_ex2_inv * K_ex1_inv;
        

        Eigen::Vector3d T12;
        Eigen::Quaterniond Q12;
        from_matrix(K_ex_, T12, Q12);

         geometry_msgs::TransformStamped tf_wc;
        tf_wc.header.stamp = drone_pose_msg->header.stamp;
        tf_wc.header.frame_id = "world";
        tf_wc.child_frame_id = "cam_frame";
        tf_wc.transform.translation.x = T12[0];
        tf_wc.transform.translation.y = T12[1];
        tf_wc.transform.translation.z = T12[2];
        tf_wc.transform.rotation.w = Q12.w();
        tf_wc.transform.rotation.x = Q12.x();
        tf_wc.transform.rotation.y = Q12.y();
        tf_wc.transform.rotation.z = Q12.z();
        
        tf_bc_wc.sendTransform(tf_wc);
        
        // std::cout << "Camera -> World:\n";
        // std::cout << T12.transpose() << std::endl;
        // std::cout << Q12.w() << " " << Q12.x() << " " << Q12.y() << " " << Q12.z() << std::endl;
        // std::cout << "\n";

        double* camera_param = new double[10];
        camera_param[0] = Q12.w(); //w
        camera_param[1] = Q12.x(); //x
        camera_param[2] = Q12.y(); //y
        camera_param[3] = Q12.z(); //z
        camera_param[4] = T12[0]; 
        camera_param[5] = T12[1];
        camera_param[6] = T12[2];
        camera_param[7] = this->F_; //f
        camera_param[8] = this->K1_; //一阶畸变
        camera_param[9] = this->K2_; //二阶畸变
        // camera_param[0] = Qw_; //w
        // camera_param[1] = Qx_; //x
        // camera_param[2] = Qy_; //y
        // camera_param[3] = Qz_; //z
        // camera_param[4] = Tx_; 
        // camera_param[5] = Ty_;
        // camera_param[6] = Tz_;
        // camera_param[7] = this->F_; //f
        // camera_param[8] = this->K1_; //一阶畸变
        // camera_param[9] = this->K2_; //二阶畸变

        // std::cout << "Camera Parameters:\n";
        // for (int i = 0; i < 10; ++i) {
        //         std::cout << camera_param[i] << " ";
        // }
        // std::cout << "\n";

        ceres::CostFunction* cost_function =
                SnavelyReprojectionErrorWithQuaternions::Create(tag_centre.x, tag_centre.y);//2d

        ceres::LossFunction* loss_function = new ceres::TrivialLoss();

        problem.AddResidualBlock(cost_function,
                                 loss_function, /* squared loss */
                                 camera_param,//10d
                                 tag_pose);//3d
        // std::cout << "Num of Factors: " << problem.NumResidualBlocks() << std::endl;

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        // options.minimizer_progress_to_stdout = true;
        options.minimizer_progress_to_stdout = true;
        options.function_tolerance = 1e-4;
        options.gradient_tolerance = 1e-8;

        // options.initial_trust_region_radius = 1e3;
        // options.min_trust_region_radius = 1e-3;  // 防止步长太小

        // options.min_linear_solver_iterations = 10;
        

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        this->F_ = camera_param[7];
        this->K1_ = camera_param[8];
        this->K2_ = camera_param[9];
        // this->Qw_ = camera_param[0];
        // this->Qx_ = camera_param[1];
        // this->Qy_ = camera_param[2];
        // this->Qz_ = camera_param[3];
        // this->Tx_ = camera_param[4];
        // this->Ty_ = camera_param[5];
        // this->Tz_ = camera_param[6];


        //camera -> world
        Eigen::Vector3d T_wc(camera_param[4], camera_param[5], camera_param[6]);
        Eigen::Quaterniond Q_wc(camera_param[0], camera_param[1], camera_param[2], camera_param[3]);
        Q_wc.normalize();
        Eigen::Matrix4d K_wc = to_matrix(T_wc, Q_wc);      

         geometry_msgs::TransformStamped tf_wc_bal;
        tf_wc_bal.header.stamp = drone_pose_msg->header.stamp;
        tf_wc_bal.header.frame_id = "world";
        tf_wc_bal.child_frame_id = "cam_frame_bal";
        tf_wc_bal.transform.translation.x = T_wc[0];
        tf_wc_bal.transform.translation.y = T_wc[1];
        tf_wc_bal.transform.translation.z = T_wc[2];
        tf_wc_bal.transform.rotation.w = Q_wc.w();
        tf_wc_bal.transform.rotation.x = Q_wc.x();
        tf_wc_bal.transform.rotation.y = Q_wc.y();
        tf_wc_bal.transform.rotation.z = Q_wc.z();
        tf_bc_wc.sendTransform(tf_wc_bal);


        // camera <- world * world <- body
        // Eigen::Matrix4d K_cb = to_inverse(K_wc) * K_ex1_;
        // Eigen::Vector3d T_cb;
        // Eigen::Quaterniond Q_cb;
        // from_matrix(K_cb, T_cb, Q_cb);
        // std::cout << "Optimized Camera Parameters:\n";
        // std::cout << T_cb.transpose() << std::endl;
        // std::cout << Q_cb.w() << " " << Q_cb.x() << " " << Q_cb.y() << " " << Q_cb.z() << std::endl;
        // std::cout << "\n";
        
        // body <- world * world <- camera
        Eigen::Matrix4d K_bc = to_inverse(K_ex1_) * K_wc;
        Eigen::Vector3d T_bc;
        Eigen::Quaterniond Q_bc;
        from_matrix(K_bc, T_bc, Q_bc);
        // std::cout << "Optimized Camera Parameters:\n";
        // std::cout << T_bc.transpose() << std::endl;
        // std::cout << Q_bc.w() << " " << Q_bc.x() << " " << Q_bc.y() << " " << Q_bc.z() << std::endl;
        // std::cout << "\n";


        // std::cout << "Optimized Camera Parameters:\n";
        // for (int i = 0; i < 10; ++i) {
                // std::cout << camera_param[i] << " ";
        // }
        // std::cout << "\n";
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "bal_extrinsic");
        ros::NodeHandle n;

        RosNode node(n);
        Subscriber<PoseArray> tag_centre_sub(n, "/tag_bbx_centre", 10, ros::TransportHints().tcpNoDelay());
        Subscriber<PoseStamped> tag_pose_sub(n, "/vrpn_client_node/tag_vrpn/pose", 10, ros::TransportHints().tcpNoDelay());
        Subscriber<PoseStamped> drone_pose_sub(n, "/vrpn_client_node/obs_03/pose", 10, ros::TransportHints().tcpNoDelay());

        typedef sync_policies::ApproximateTime<PoseArray, PoseStamped, PoseStamped> SyncPolicy;
        Synchronizer<SyncPolicy> sync(SyncPolicy(10), tag_centre_sub, tag_pose_sub, drone_pose_sub);
        sync.registerCallback(boost::bind(&RosNode::callback, &node, _1, _2, _3));

        ros::spin();

        return 0;
}