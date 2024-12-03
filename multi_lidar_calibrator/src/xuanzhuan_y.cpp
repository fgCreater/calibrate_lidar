#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <Eigen/Dense>

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    // 将PointCloud2消息转换为PCL点云格式，包含强度信息
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    pcl::PointCloud<pcl::PointXYZI> rotated_cloud;
    rotated_cloud.header = cloud.header; // 保持头信息

    // 创建旋转矩阵（Z轴旋转180度）
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()); // 绕Z轴旋转180度

    // 进行坐标转换
    for (const auto &point : cloud.points) {
        // 将点转换为Eigen格式
        Eigen::Vector3f original_point(point.x, point.y, point.z);

        // 应用旋转矩阵
        Eigen::Vector3f rotated_point = rotation_matrix * original_point;

        // 添加到新的点云中
        pcl::PointXYZI new_point;
        new_point.x = rotated_point.x();
        new_point.y = rotated_point.y();
        new_point.z = rotated_point.z();
        new_point.intensity = point.intensity; // 复制强度信息

        rotated_cloud.points.push_back(new_point);
    }

    // 发布变换后的点云
    rotated_cloud.width = rotated_cloud.points.size();
    rotated_cloud.height = 1; // 如果是无序点云，高度为1
    rotated_cloud.is_dense = true;

    // 创建点云消息并发布
    sensor_msgs::PointCloud2 rotated_cloud_msg;
    pcl::toROSMsg(rotated_cloud, rotated_cloud_msg);

    rotated_cloud_msg.header.frame_id = cloud_msg->header.frame_id; // 同样的坐标框架

    static ros::NodeHandle pub_nh; // 静态NodeHandle以避免多次初始化
    static ros::Publisher pub = pub_nh.advertise<sensor_msgs::PointCloud2>("/rotated_points207", 10000);

    pub.publish(rotated_cloud_msg);
    // ROS_INFO("Published rotated point cloud with %zu points.", rotated_cloud.points.size());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_rotation_node");
    ros::NodeHandle nh;

    // 订阅入点云数据
    ros::Subscriber sub = nh.subscribe("/livox/lidar_192_168_1_207", 10000, pointCloudCallback);

    ros::spin(); // 循环等待回调
    return 0;
}
