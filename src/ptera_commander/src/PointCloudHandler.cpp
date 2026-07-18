
#include "PointCloudHandler.h"
#include <pcl/common/centroid.h>

PointCloudHandler::PointCloudHandler()
{

}

PointCloudHandler::~PointCloudHandler()
{

}

bool PointCloudHandler::fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut)
{
    // Define the point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read the PLY file
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(aFilePath, *cloud) == -1)
    {
        std::cerr << "Couldn't read file: " << aFilePath << std::endl;
        return -1;
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);

    pcl_conversions::fromPCL(pcl_pc2, aCloudOut);
    aCloudOut.header.frame_id = "map";

    return true; 
}

bool PointCloudHandler::fromFile(const std::string& aFilePath,
                                 const Eigen::Vector3d& aGlobalCentroid,
                                 sensor_msgs::msg::PointCloud2& aCloudOut)
{
    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(aFilePath, *cloud) == -1)
    {
        std::cerr << "Couldn't read file: " << aFilePath << std::endl;
        return false;
    }

    // Compute local centroid
    Eigen::Vector4f localCentroid;
    if (pcl::compute3DCentroid(*cloud, localCentroid) == 0)
    {
        std::cerr << "Failed to compute centroid for cloud: "
                  << aFilePath << std::endl;
        return false;
    }

    // Compute translation: global - local
    Eigen::Vector3f translation =
        aGlobalCentroid.cast<float>() - localCentroid.head<3>();

    // Apply translation to cloud
    for (auto& pt : cloud->points)
    {
        pt.x += translation.x();
        pt.y += translation.y();
        pt.z += translation.z();
    }

    // Convert to ROS2 PointCloud2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, aCloudOut);

    aCloudOut.header.frame_id = "map";

    return true;
}
