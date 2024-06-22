#include <string>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_broadcaster.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace ros;
using namespace std;

ros::Publisher map_pub;
string map_source_path;
string map_save_path;

void pub_all()
{
    float z_min_table[50][20];
    sensor_msgs::PointCloud2 map_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sum (new pcl::PointCloud<pcl::PointXYZI>);
    
    for(int i=-40; i<10; i++)
    {
        for(int j=-10; j<10; j++)
        {
            string map_file = map_source_path + "submap_" + to_string(i) +  "_" + to_string(j) + ".pcd";
            ifstream f(map_file);
    
            if(!f)
            {
                continue;
            }
        
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return;
            }
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("intensity");
            pass.setFilterLimits(40, 256);
            pass.setFilterLimitsNegative(false);
            pass.filter(*cloud);

            if(cloud->points.size() <= 0)   
            {
                continue;
            }

            float z_min = cloud->points[0].z;
            int z_index = 0;

            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(50);
            sor.setStddevMulThresh (1);
            sor.filter(*cloud);

            for(int i=0; i<cloud->points.size(); i++)
            {
                int temp = cloud->points[i].z;

                if(temp < z_min)
                {
                    z_min = temp;
                    z_index = i;
                }
            }

            cloud->points.erase(cloud->points.begin() + z_index);

            z_min = cloud->points[0].z;
            z_index = 0;
            for(int i=0; i<cloud->points.size(); i++)
            {
                int temp = cloud->points[i].z;

                if(temp < z_min)
                {
                    z_min = temp;
                    z_index = i;
                }
            }

            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min+2, z_min+4);
            pass.setFilterLimitsNegative(false);
            pass.filter(*cloud);
            
            cout << i << ", " << j << " : " << z_min << " " << cloud->points.size() <<  endl;            

            for(int i=0; i<cloud->points.size(); i++)
            {
                cloud->points[i].z = 0;
            }
            cloud->width = 1;
            cloud->height = cloud->points.size();
            *sum += *cloud;
            
            if(cloud->points.size() != 0)
            {
                pcl::io::savePCDFile<pcl::PointXYZI>(map_save_path + "submap_" + to_string(i) +  "_" + to_string(j) + ".pcd", *cloud);
            }
        }
    }
    cout << "output map" << endl;
    pcl::toROSMsg(*sum, map_pc);
    map_pc.header.stamp = ros::Time::now();
    map_pc.header.frame_id = "map";
    map_pub.publish(map_pc);
}

int main(int argc, char** argv) 
{
    ros::init (argc, argv, "map_modifier");
    ros::NodeHandle nh;
    nh.param<string>("/map_source_path", map_source_path, "/Default/path");
    nh.param<string>("/map_save_path", map_save_path, "/Default/path");
    
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 1);
    pub_all();
    ros:spin();
    return 0;
}