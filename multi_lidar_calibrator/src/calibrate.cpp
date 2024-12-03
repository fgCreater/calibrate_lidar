#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/registration/ndt.h>
#include <deque>

// 全局变量，存储合并的点云数据
static pcl::PointCloud<pcl::PointXYZI> merged_cloud;  // 合并后的点云数据
static ros::Publisher merged_cloud_pub;                // 发布合并点云的发布者
static std::deque<pcl::PointCloud<pcl::PointXYZI>> point_cloud_queue;  // 存储接收到的点云的队列

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

        // 发布合并的点云

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(merged_cloud, output);
        output.header = input->header; // 将输入消息头赋值给输出
//        ros::Rate r(1);// 发布频率

            merged_cloud_pub.publish(output); // 发布合并点云

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_point_cloud_merger");
    ros::NodeHandle nh;

    // 发布合并点云的主题
    merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_points206", 100000);

    // 订阅输入点云的主题
    ros::Subscriber points_sub = nh.subscribe("/livox/lidar_192_168_1_206", 100000, points_callback);

    ros::spin();  // 启动ROS事件循环

    return 0;
}
