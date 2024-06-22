#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/convolution_3d.h>

using namespace std;
using namespace cv;
using namespace ros;

const double PI = 3.14159265358979323846;
const float range_resolution = 0.175;
const float angle_resolution = (2*PI)/400;
Publisher radar_pub;

bool intensity_compare(pcl::PointXYZI a, pcl::PointXYZI b) 
{
    return a.intensity > b.intensity; 
}

pcl::PointCloud<pcl::PointXYZI>::Ptr create_radar_pc(Mat img)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZI>);

    int height = img.rows;
    int width = img.cols;
    int channels = img.channels();

    // Initialize the vector storing the maximum pixel position and intensity
    std::vector<cv::Point> maxPixelPositions(20);
    std::vector<int> maxPixelIntensities(20, 0);

    // Define maximum and minimum thresholds for distance
    int range_max_threshold = 100;
    int range_min_threshold = 3;
    // Calculate maximum and minimum row indexes based on distance resolution
    int max_row = range_max_threshold / range_resolution;
    int min_row = range_min_threshold / range_resolution;

    for (int column = 0; column < width; ++column) {
        // Reset the maximum pixel position and intensity vector for each column
        std::fill(maxPixelPositions.begin(), maxPixelPositions.end(), cv::Point(0, 0));
        std::fill(maxPixelIntensities.begin(), maxPixelIntensities.end(), 0);

        for (int row = 4 + min_row; row < max_row; ++row) {
            // Get the intensity value of the current pixel
            uchar intensity = img.at<uchar>(row, column);
            // Find the smallest value in the intensity vector
            auto minIntensityIt = std::min_element(maxPixelIntensities.begin(), maxPixelIntensities.end());

            // If the current pixel intensity > the minimum intensity, update the position and intensity
            if (intensity > *minIntensityIt) {
                int index = std::distance(maxPixelIntensities.begin(), minIntensityIt);
                maxPixelPositions[index] = cv::Point(row, column);
                maxPixelIntensities[index] = intensity;
            }
        }

        // Iterate over the 20 maximum intensity pixels found
        for (int i = 0; i < 20; ++i) {

            float range = (range_resolution / 2) + (maxPixelPositions[i].x - 4) * range_resolution;
            float angle = (angle_resolution / 2) + (maxPixelPositions[i].y) * angle_resolution;

            // Create a new point directly in the point cloud and set its coordinates and intensity
            pcl::PointXYZI& point = new_pc->emplace_back();
            point.x = range * cos(angle);
            point.y = -range * sin(angle);
            point.z = 0;
            point.intensity = static_cast<float>(maxPixelIntensities[i]);
            new_pc -> push_back(point);
        }
    }
    ROS_INFO("%ld", new_pc->points.size());

    return new_pc;
}

void radarCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img;
    ROS_INFO("get radar"); 
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc_ptr = create_radar_pc(img);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*radar_pc_ptr, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "navtech";
    radar_pub.publish(pc_msg);
    ROS_INFO("PointCloud"); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_polar_to_pointcloud");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
    image_transport::Subscriber sub = it.subscribe("/Navtech/Polar", 1, radarCallback);
    
    ros::spin();
    return 0;
}