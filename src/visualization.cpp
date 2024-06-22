#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;
class Visualizer
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber radar_pc_sub;
    ros::Publisher map_pub;
    ros::Publisher radar_pc_pub;
    ros::Publisher path_pub;
    tf::TransformBroadcaster br;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    nav_msgs::Path path;
    
    std::string save_path;
    std::string map_path;
    std::ofstream file;

    float pose_x;
    float pose_y;
    float pose_yaw;

    bool map_ready = false;

    vector<float> x;
    vector<float> y;
    vector<float> yaw;

    int seq = 0;


public:
    Visualizer(ros::NodeHandle nh) : map_pc(new pcl::PointCloud<pcl::PointXYZI>)
    {
        map_ready = false;
        path.header.frame_id = "map";
        
        _nh = nh;
        _nh.param<string>("/save_path", save_path, "/Default/path");
        _nh.param<string>("/map_path", map_path, "/Default/path");

        radar_pc_sub = _nh.subscribe("/radar_pc", 400, &Visualizer::radar_pc_callback, this);
        radar_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/tranformed_radar_pc", 1);
        path_pub = _nh.advertise<nav_msgs::Path>("/localization_path", 1);
        map_pub = _nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 1);

        map_pub_all();
        usleep(1e6);

        vector<vector<string>> content;
	    vector<string> row;
        fstream file (save_path, ios::in);
        string line, word;
	    if(file.is_open())
	    {
		    while(getline(file, line))
		    {
			    row.clear();
 
			   stringstream str(line);
 
			    while(getline(str, word, ','))
				    row.push_back(word);
			    content.push_back(row);
		    }
	    }
	    else
        {
            ROS_WARN("Could not open the file");
        }
 
	    for(int i=1; i<content.size(); i++)
	    {
            x.push_back(stof(content[i][1]));
            y.push_back(stof(content[i][2]));
            yaw.push_back(stof(content[i][3]));
	    }
    }

    ~Visualizer()
    {
        ROS_WARN("Exit Localization");
        file.close();
    }

    void map_pub_all()
    {
        sensor_msgs::PointCloud2 map_pc;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr sum (new pcl::PointCloud<pcl::PointXYZI>);

        for(int i=-40; i<10; i++)
        {
            for(int j=-10; j<10; j++)
            {   
                string map_file = map_path + "submap_" + to_string(i) +  "_" + to_string(j) + ".pcd";
                ifstream f(map_file);
        
                if(!f)
                {
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

        pcl::toROSMsg(*sum, map_pc);
        map_pc.header.stamp = ros::Time::now();
        map_pc.header.frame_id = "map";
        map_pub.publish(map_pc);
        ROS_WARN("Publish All Map, points size = %ld", sum->points.size());
    }

    void radar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        ros::Time stamp = msg->header.stamp;
        float pose_x = x[seq];
        float pose_y = y[seq];
        float pose_yaw = yaw[seq];
        
        tf_brocaster(pose_x, pose_y, pose_yaw, stamp);
        pcl::fromROSMsg(*msg, *radar_pc);
        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(*radar_pc, radar_pc_msg);
        radar_pc_msg.header.stamp = stamp;
        radar_pc_msg.header.frame_id = "base_link";
        radar_pc_pub.publish(radar_pc_msg);

        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, pose_yaw);
        myQuaternion.normalize();

        pose.header.stamp = stamp;
        pose.header.frame_id = "map";

        pose.pose.position.x = pose_x;
        pose.pose.position.y = pose_y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();

        path.header.stamp = stamp;
        path.poses.push_back(pose);
        path_pub.publish(path);

        ROS_INFO("Publish transformed pc");
        ROS_INFO("[seq %d] x:%.3f, y:%.3f, yaw:%.3f\n", seq, pose_x, pose_y, pose_yaw);
        seq++;
    }

    void tf_brocaster(float x, float y, float yaw, ros::Time stamp)
    {  
        ROS_INFO("Update map to baselink");
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, stamp, "map", "base_link"));
    }
};

int main(int argc, char** argv) 
{
    ros::init (argc, argv, "visualize");
    ros::NodeHandle nh;
    Visualizer Visualizer(nh);

    ros::spin();
    return 0;
}