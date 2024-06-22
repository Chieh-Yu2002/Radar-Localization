#include <string>
#include <fstream>
#include <iostream>

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/transform_broadcaster.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<pcl/filters/voxel_grid.h>

using namespace ros;
using namespace std;

class map_publisher
{
private:
    ros::NodeHandle _nh;
    ros::Publisher map_pub;
    ros::Subscriber radar_pose_sub;
    ros::Subscriber gps_sub;
    string map_path;
    float pose_x, pose_y;
    bool flag;
    bool initialized;
    int last_x, last_y;

public:
    map_publisher(ros::NodeHandle nh)
    {
        initialized = false;
        _nh = nh;
        _nh.param<string>("map_path", map_path, "/Default/path");

        map_pub = _nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 1);
        radar_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/tranformed_radar_pose", 1, &map_publisher::radar_pose_callback, this);
        gps_sub = _nh.subscribe("/gps", 1, &map_publisher::gps_call_back, this);
    }

    ~map_publisher()
    {
        ROS_INFO("Exit Map Publisher");
    }

    void gps_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if(!initialized)
        {
            int x = ((int)msg->pose.position.x);
            int y = ((int)msg->pose.position.y);
            initialized = true;
            initialize(x, y);
        }
    }

    void initialize(int x, int y)
    {
        flag = false;
        sensor_msgs::PointCloud2 map_pc;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr sum (new pcl::PointCloud<pcl::PointXYZI>);

        if(x<0)
            x = (x-50)/50; 
        else
            x = x/50;

        if(y<0)
            y = (y-50)/50;
        else
            y = y/50;

        for(int i=x-2; i<x+3; i++)
        {
            for(int j=y-2; j<y+3; j++)
            {   
                string map_file = map_path + "submap_" + to_string(i) +  "_" + to_string(j) + ".pcd";
                ifstream f(map_file);
                //ROS_INFO_STREAM("Checking map file: " << map_file);
                //ROS_INFO("%s",map_file.c_str());
        
                if(!f)
                {
                    ROS_INFO("No more file");

                    continue;
                }
                if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_file, *cloud) == -1) //* load the file
                {
                    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                    return;
                }
                
                *sum += *cloud;
            }
        }
        //Filter lidar point
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_map;
        voxel_filter_map.setLeafSize (0.15f, 0.15f, 0.15f);
        voxel_filter_map.setInputCloud (sum);
        pcl::PointCloud<pcl::PointXYZI>:: Ptr filter_map(new pcl :: PointCloud<pcl::PointXYZI>);
        voxel_filter_map.filter (*filter_map);

        pcl::toROSMsg(*filter_map, map_pc);
        map_pc.header.stamp = ros::Time::now();
        map_pc.header.frame_id = "map";
        map_pub.publish(map_pc);
        ROS_WARN("Publish Map [%d, %d] : %ld points", x, y, sum->points.size());

        last_x = x;
        last_y = y;
    }

    void radar_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        int x = ((int)msg->pose.position.x);
        int y = ((int)msg->pose.position.y);
        if(!initialized)
        {
            return;
        }
        else
        {
            if(x<0)
                x = (x-50)/50; 
            else
                x = x/50;

            if(y<0)
                y = (y-50)/50;
            else
                y = y/50;
        
            if(x!=last_x || y!=last_y)
                flag = true;
        }



        if(flag)
        {
            flag = false;
            last_x = x;
            last_y = y;

            sensor_msgs::PointCloud2 map_pc;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr sum (new pcl::PointCloud<pcl::PointXYZI>);

            for(int i=x-2; i<x+3; i++)
            {
                for(int j=y-2; j<y+3; j++)
                {
                    string map_file = map_path + "submap_" + to_string(i) +  "_" + to_string(j) + ".pcd";
                    ROS_WARN("Attempting to load map file: %s", map_file.c_str());
                    ifstream f(map_file.c_str());
            
                    if(!f)
                    {
                        continue;
                    }
                    if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_file, *cloud) == -1) //* load the file
                    {
                        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                        return;
                    }
                    else{
                        ROS_WARN("Successfully loaded map file: %s", map_file.c_str());
                    }
                    *sum += *cloud;
                }

            }
            pcl::toROSMsg(*sum, map_pc);
            for(int i=0; i<3; i++)
            {
                map_pc.header.stamp = ros::Time::now();
                map_pc.header.frame_id = "map";
                map_pub.publish(map_pc);
                ROS_WARN("Publish Map [%d, %d] : %ld points", x, y, sum->points.size());   
                ros::Duration(0.1).sleep();
            }
        }

            

    } 
};

int main(int argc, char** argv) 
{
    ros::init (argc, argv, "map_publisher");
    ros::NodeHandle nh;
    map_publisher map_publisher(nh);
    ros::spin();
    return 0;
}