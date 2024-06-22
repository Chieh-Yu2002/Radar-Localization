#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

using namespace std;
class Localizer
{
private:
    ros::NodeHandle _nh;

    ros::Subscriber radar_pc_sub;
    ros::Subscriber map_sub;
    ros::Subscriber gps_sub;

    ros::Publisher radar_pc_pub;
    ros::Publisher radar_pose_pub;
    ros::Publisher path_pub;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    nav_msgs::Path path;
    tf::TransformBroadcaster br;
    
    std::string save_path;
    std::ofstream file;

    float pose_x;
    float pose_y;
    float pose_yaw;
    float gps_x;
    float gps_y;
    float gps_yaw;

    // To get better initial guess for bonus
    // float last_pose_x;
    // float last_pose_y;
    // float next_pose_x;
    // float next_pose_y;

    int id = 0;
    int max_iter;
    float epsilon1;
    float epsilon2;
    float correspond;

    Eigen::Matrix4f init_guess;

    bool map_ready = false;
    bool gps_ready = false;
    bool initialized = false;

    int frame = 0;


public:
    Localizer(ros::NodeHandle nh) : map_pc(new pcl::PointCloud<pcl::PointXYZI>)
    {
        map_ready = false;
        gps_ready = false;
        
        _nh = nh;
        _nh.param<string>("/save_path", save_path, "/Default/path");

        init_guess.setIdentity();
        file.open(save_path);
        file << "id,x,y,yaw\n";

        radar_pc_sub = _nh.subscribe("/radar_pc", 400, &Localizer::radar_pc_callback, this);
        map_sub = _nh.subscribe("/map_pc", 1, &Localizer::map_callback, this);
        gps_sub = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);

        radar_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/tranformed_radar_pc", 1);
        radar_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/tranformed_radar_pose", 1);
        path_pub = _nh.advertise<nav_msgs::Path>("/localization_path", 1);
    }

    ~Localizer()
    {
        ROS_WARN("Exit Localization");
        file.close();
    }

    void gps_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_WARN("Got GPS data");
        gps_x = msg->pose.position.x;
        gps_y = msg->pose.position.y;
        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double r, p, yaw;
        m.getRPY(r, p, yaw);
        gps_yaw = yaw;
        if(!gps_ready)
        {
            pose_x = gps_x;
            pose_y = gps_y;
            pose_yaw = gps_yaw;
            gps_ready = true;
        }
    }

    void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ROS_WARN("Got Map Pointcloud");
        pcl::fromROSMsg(*msg, *map_pc);
        map_ready = true;
    }

    Eigen::Matrix4d transform2D(double xt, double yt, double theta) {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

        matrix(0, 3) = xt;
        matrix(1, 3) = yt;

        matrix(0, 0) = cos(theta);
        matrix(0, 1) = -sin(theta);
        matrix(1, 0) = sin(theta);
        matrix(1, 1) = cos(theta);

        return matrix;
    }

    ros::Time prev_time;

    void radar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();

        ROS_WARN("Got Radar Pointcloud %d", frame);
        frame ++;
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *radar_pc);
        ROS_INFO("point size: %d", radar_pc->width);


        while(!(map_ready && gps_ready))
        {
            ROS_WARN("Wait for map and gps ready");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }


        if(!initialized)
        {
            // Use the GPS data to initialize the initial guess
            init_guess = transform2D(gps_x, gps_y, gps_yaw).cast<float>();
            last_pose_x = gps_x;
            last_pose_y = gps_y;
            initialized = true; // Set the initialized flag to true
        }

        // Set up the NDT object and its parameters (resolution, max iterations, etc.)
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
        ndt.setTransformationEpsilon(1e-3); // Set minimum transformation difference for termination conditions
        ndt.setStepSize(0.5);// Set maximum step size for More-Thuente line search
        ndt.setResolution(8); // Set resolution of NDT grid structure (VoxelGridCovariance)
        ndt.setMaximumIterations(10); // Set maximum number of registration iterations

        ndt.setInputSource(radar_pc);// Set point cloud to be aligned
        ndt.setInputTarget(map_pc);
        ndt.align(*output_pc, init_guess);

        int ndt_iter = ndt.getFinalNumIteration();
        ROS_INFO("ndt1_iter : %d", ndt_iter);
        ROS_INFO("radar_pc : %ld", radar_pc->points.size());// Set target point cloud to align the source cloud to
        ROS_INFO("map_pc : %ld", map_pc->points.size());
        
        Eigen::Matrix4f second_init_guess = ndt.getFinalTransformation().cast<float>();

        // Second ndt for competition and bonus
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt2;
        ndt2.setTransformationEpsilon(1e-6); 
        ndt2.setStepSize(0.01);
        ndt2.setResolution(6); 
        ndt2.setMaximumIterations(30); 
        ndt2.setInputSource(radar_pc);
        ndt2.setInputTarget(map_pc);
        ndt2.align(*output_pc, second_init_guess);

        int ndt2_iter = ndt2.getFinalNumIteration();
        ROS_INFO("ndt2_iter : %d", ndt2_iter);

        Eigen::Matrix4f third_init_guess = ndt.getFinalTransformation().cast<float>();
        Set up the ICP object and its parameters
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(0.01);
        icp.setMaximumIterations(40); 
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setInputSource(radar_pc);
        icp.setInputTarget(map_pc);
        icp.align(*output_pc, third_init_guess);


        // Check if the NDT has converged and extract the transformation
        if(ndt2.hasConverged())
        {
            Eigen::Matrix4f ndt2_pose = ndt2.getFinalTransformation();
            pcl::transformPointCloud(*radar_pc, *output_pc, ndt2_pose);

            double delta_t = (current_time - prev_time).toSec();
            // Extract 2D pose information (x, y, yaw) from the 4x4 transformation matrix
            float ndt2_yaw = std::atan2(ndt2_pose(1, 0), ndt2_pose(0, 0));
            pose_x = ndt2_pose(0, 3);
            pose_y = ndt2_pose(1, 3);
            pose_yaw = ndt2_yaw;

            // Calculate next initial guess for bonus
            // float ratio = 0.7;
            // float delta_x = pose_x - last_pose_x;
            // float delta_y = pose_y - last_pose_y;
            // float velocity = ratio * sqrt(delta_x * delta_x + delta_y * delta_y);

            // next_pose_x = pose_x + velocity * cos(pose_yaw);
            // next_pose_y = pose_y + velocity * sin(pose_yaw);
            // set_init_guess(next_pose_x, next_pose_y, pose_yaw);
            // init_guess = transform2D(next_pose_x, next_pose_y, pose_yaw).cast<float>();
            init_guess = transform2D(pose_x, pose_y, pose_yaw).cast<float>();

            // update next initial guess for bonus
            // last_pose_x = pose_x;
            // last_pose_y = pose_y;

        }  
        else
        {
            ROS_WARN("ICP did not converge");
        }
       
        tf_brocaster(pose_x, pose_y, pose_yaw);
        radar_pose_publisher(pose_x, pose_y, pose_yaw);

        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(*radar_pc, radar_pc_msg);
        radar_pc_msg.header.stamp = ros::Time::now();
        radar_pc_msg.header.frame_id = "base_link";
        radar_pc_pub.publish(radar_pc_msg);
        ROS_INFO("Publish transformed pc");
        ROS_INFO("[id %d] x:%.3f, y:%.3f, yaw:%.3f\n", id, pose_x, pose_y, pose_yaw);


        file << id << ",";
        file << pose_x << ",";
        file << pose_y << ",";
        file << pose_yaw << "\n";
        file.flush();
        ROS_WARN("save data");
        id++;
    }

    void radar_pose_publisher(float x, float y, float yaw)
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, yaw);
        myQuaternion.normalize();

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();
        radar_pose_pub.publish(pose);
        
        path.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    void tf_brocaster(float x, float y, float yaw)
    {  
        ROS_INFO("Update map to baselink");
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "localizer");
    ros::NodeHandle nh;
    Localizer Localizer(nh);

    ros::spin();
    return 0;
}