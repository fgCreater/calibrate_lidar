#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <pcl/registration/ndt.h>
#include <deque>
#include <boost/shared_ptr.hpp>

// 全局变量，存储合并的点云数据
static pcl::PointCloud<pcl::PointXYZI> merged_cloud;  // 合并后的点云数据
static ros::Publisher merged_cloud_pub;                // 发布合并点云的发布者
static std::deque<pcl::PointCloud<pcl::PointXYZI>> point_cloud_queue;  // 存储接收到的点云的队列

// 点云过滤函数
void filterPointsWithinRegion(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& src_cloud_ptr,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr& dst_cloud_ptr,
                              const std::pair<double, double>& x_range,
                              const std::pair<double, double>& y_range,
                              const std::pair<double, double>& z_range,
                              bool remove) 
{
    dst_cloud_ptr->clear(); // 清空目标点云

    for (const auto& pt : src_cloud_ptr->points) {
        bool inside = (pt.x >= x_range.first && pt.x <= x_range.second &&
                       pt.y >= y_range.first && pt.y <= y_range.second &&
                       pt.z >= z_range.first && pt.z <= z_range.second);

        if (inside ^ remove) {
            dst_cloud_ptr->points.push_back(pt);
        }
    }
}

// 点云回调函数
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI> tmp;

    // Convert the ROS PointCloud2 message to PCL format
    pcl::fromROSMsg(*input, tmp);

    // 将当前点云加入队列
    point_cloud_queue.push_back(tmp);

    if (point_cloud_queue.size() > 10)
    {
        point_cloud_queue.pop_front();  // 移除最旧的一帧
    }

    if (point_cloud_queue.size() == 10)
    {
        // 合并队列中的点云
        merged_cloud.clear();  // 清空前一次的合并结果
        for (const auto& cloud : point_cloud_queue)
        {
            merged_cloud += cloud;  // 使用 '+' 运算符合并点云
        }

        // 过滤点云：定义范围
        // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        // filterPointsWithinRegion(merged_cloud.makeShared(), 
        //                           filtered_cloud_ptr,
        //                           std::make_pair(0, 5.0),    // x_range      
        //                           std::make_pair(-4, 4),    // y_range
        //                           std::make_pair(-2.0, 2.0),    // z_range
        //                           true); // false to keep points within specified range

        // 发布合并后的和过滤后的点云
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(merged_cloud, output);
        output.header = input->header; // 将输入消息头赋值给输出
        merged_cloud_pub.publish(output); // 发布合并点云
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_point_cloud_merger2");
    ros::NodeHandle nh;

    // 发布合并点云的主题
    merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_points207", 100000);

    // 订阅输入点云的主题
    ros::Subscriber points_sub = nh.subscribe("/rotated_points207", 100000, points_callback);

    ros::spin();  // 启动ROS事件循环

    return 0;
}
