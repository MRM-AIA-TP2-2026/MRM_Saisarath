#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode()
        : Node("obstacle_detection_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 10,
            std::bind(&ObstacleDetectionNode::pointCloudBufferCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles", 10);
    }

private:
    void pointCloudBufferCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "No point cloud received yet.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        //
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel1;
        voxel1.setInputCloud(cloud);
        voxel1.setLeafSize(0.07f, 0.07f, 0.07f);
        voxel1.filter(*cloud_voxel1);

        pcl::SACSegmentation<pcl::PointXYZ> seg1;
        pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
        seg1.setOptimizeCoefficients(true);
        seg1.setModelType(pcl::SACMODEL_PLANE);
        seg1.setMethodType(pcl::SAC_RANSAC);
        seg1.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Z-axis is "up"
        seg1.setEpsAngle(10.0 * 3.141 / 180);
        seg1.setDistanceThreshold(0.1);
        seg1.setMaxIterations(2000);
        seg1.setInputCloud(cloud_voxel1);
        seg1.segment(*inliers1, *coefficients1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract1;
        extract1.setInputCloud(cloud_voxel1);
        extract1.setIndices(inliers1);
        extract1.setNegative(true);
        extract1.filter(*cloud_no_ground1);
        //
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_no_ground1);
        voxel.setLeafSize(0.07f, 0.07f, 0.07f);
        voxel.filter(*cloud_voxel);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Z-axis is "up"
        seg.setEpsAngle(10.0 * 3.141 / 180);
        seg.setDistanceThreshold(0.1);
        seg.setMaxIterations(2000);
        seg.setInputCloud(cloud_voxel);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_voxel);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_no_ground);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_no_ground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(15000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_no_ground);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Detected %ld obstacles", cluster_indices.size());

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_no_ground, output);
        output.header = msg->header;
        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
    rclcpp::shutdown();
    return 0;
}