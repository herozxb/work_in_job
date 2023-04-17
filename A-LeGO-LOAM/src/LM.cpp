#include "alego/utility.h"
#include <std_srvs/Empty.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace boost::interprocess;
using namespace std;

#include <iostream>
#include <stdint.h>
#include <vector>
#include <random>
#include <cmath>

#include "../include/EKF/types.h"
#include "../include/EKF/kalman_filter.h"

static constexpr size_t DIM_X{ 1 };
static constexpr size_t DIM_Z{ 2 };

static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter;
static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter_x;
static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter_y;
static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter_z;

double x = 0;
double y = 0;
double z = 0;

class LM
{
private:

  //initilize the ros node
  ros::NodeHandle nh_;
  
  //initialize the Publisher
  ros::Publisher pub_cloud_surround_;
  ros::Publisher pub_odom_aft_mapped_;
  ros::Publisher pub_keyposes_;

  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;
  ros::Publisher pub_recent_keyframes_;

  double time_laser_odom_;

  int counter = 0;
  int counter_stable_map = 0;
  
  ros::Subscriber sub_pc_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pre;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_boxxed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_boxxed_2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_fixed;
    
  Eigen::Matrix4f Ti;
  Eigen::Matrix4f Ti_of_map;
  Eigen::Matrix4f Ti_of_map_real;
  Eigen::Matrix4f Ti_translation;
  Eigen::Matrix4f Ti_real;
  
  
  ros::Publisher pub_odom_aft_mapped_2;
  ros::Publisher pub_odom_aft_mapped_3;   
  ros::Publisher pub_odom_aft_mapped_kalman; 
    
public:

  LM(ros::NodeHandle nh) : nh_(nh)
  {
    onInit();
  }
  
  void onInit()
  {

    ROS_INFO("--------- LaserMapping init --------------");

    //all the publiser
    pub_cloud_surround_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 10);
    pub_odom_aft_mapped_ = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped", 10);
    pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 10);
    pub_recent_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/recent_keyframes", 10);
    pub_history_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/history_keyframes", 10);
    pub_icp_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/icp_keyframes", 10);

    
    sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, boost::bind(&LM::main_callback, this, _1));
    
    cloud_pre.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final_boxxed.reset(new pcl::PointCloud<pcl::PointXYZ>);    
    map_final_boxxed_2.reset(new pcl::PointCloud<pcl::PointXYZ>);   
    map_fixed.reset(new pcl::PointCloud<pcl::PointXYZ>);    
    
    Ti = Eigen::Matrix4f::Identity ();
    Ti_of_map = Eigen::Matrix4f::Identity ();
    Ti_of_map_real = Eigen::Matrix4f::Identity ();
    Ti_translation = Eigen::Matrix4f::Identity ();
    Ti_real = Eigen::Matrix4f::Identity ();
    
    
    pub_odom_aft_mapped_2 = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_2", 10);
    pub_odom_aft_mapped_3 = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_3", 10);
    pub_odom_aft_mapped_kalman = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_kalman", 10);
  }
  
  
  
  //process the input cloud
  void main_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
  
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_boxxed_for_local (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::fromROSMsg(*msg, *cloud_in);

    if( counter == 0 )
    {
        // Initialize the map
    	*map_final = *cloud_in;
    	
    }
    else
    {
        // 1.0 remove the Nan points
	// remove the NaN points and remove the points out of distance
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	
	
	// 1.1 Voxel the cloud
	
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor_input;
	sor_input.setInputCloud (cloud_in);
	sor_input.setLeafSize (0.1f, 0.1f, 0.1f);
	sor_input.filter (*cloud_in_filtered);
	
	// 1.2 boxxed the cloud
	*cloud_in_boxxed_for_local = *cloud_in_filtered;
	
        pcl::CropBox<pcl::PointXYZ> boxFilter_for_in;
	float x_min_for_in = - 50, y_min_for_in = - 50, z_min_for_in = + 0 ;
	float x_max_for_in = + 50, y_max_for_in = + 50, z_max_for_in = + 50;
	
	boxFilter_for_in.setMin(Eigen::Vector4f(x_min_for_in, y_min_for_in, z_min_for_in, 1.0));
	boxFilter_for_in.setMax(Eigen::Vector4f(x_max_for_in, y_max_for_in, z_max_for_in, 1.0));

	boxFilter_for_in.setInputCloud(cloud_in_boxxed_for_local);
	boxFilter_for_in.filter(*cloud_in_boxxed_for_local);
	

	/*
	// 2.0 gicp of the cloud_in voxeled and pre cloud
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaxCorrespondenceDistance(1.0);
	gicp.setTransformationEpsilon(0.001);
	gicp.setMaximumIterations(1000);
	
	gicp.setInputSource(cloud_in_filtered);
	gicp.setInputTarget(cloud_pre);
	pcl::PointCloud<pcl::PointXYZ> Final;
	gicp.align(Final);
	//*/
	
	pcl::PointCloud<pcl::PointXYZ> Final;
	
	if(counter < 3 )
	{
		// 1st icp for cloud_in [now] and cloud_pre [last]
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud_in_filtered);
		icp.setInputTarget(cloud_pre);
		icp.align(Final);
			
		// 2.2 get the x,y,z of the odometry of the trajectory
		Ti = icp.getFinalTransformation () * Ti;
	}
	else
	{
		// 2.0 gicp of the cloud_in voxeled and pre cloud
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
		gicp.setMaxCorrespondenceDistance(1.0);
		gicp.setTransformationEpsilon(0.001);
		gicp.setMaximumIterations(1000);
		
		gicp.setInputSource(cloud_in_filtered);
		gicp.setInputTarget(cloud_pre);
		gicp.align(Final);
		Ti = gicp.getFinalTransformation () * Ti;
	
	}
	//*/
	
	// 2.1 output the cloud_in_boxxed_for_local and cloud_pre
	//brown is the input cloud of right now frame
	sensor_msgs::PointCloud2Ptr msg_second(new sensor_msgs::PointCloud2);
	//cout<<"==================cloud_in====================="<<endl;
	pcl::toROSMsg(*cloud_in_boxxed_for_local, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_cloud_surround_.publish(msg_second);

/*
	//blue is cloud_pre, the last frame
	pcl::toROSMsg(*cloud_pre, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_recent_keyframes_.publish(msg_second); 
//*/

	//cout<<"==================Final====================="<<endl;
	//pcl::toROSMsg(Final, *msg_second);
	//msg_second->header.stamp.fromSec(0);
	//msg_second->header.frame_id = "map";
	//pub_history_keyframes_.publish(msg_second);    




	
	Ti_translation(0,3) = Ti(0,3);
	Ti_translation(1,3) = Ti(1,3);
	Ti_translation(2,3) = Ti(2,3);
	
	
	// 3.0 voxed the map_final
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (map_final);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*map_final);
	
	
	if( counter_stable_map % 20 == 0 )
	{
		// 3.1 boxxed the map_final
		pcl::CropBox<pcl::PointXYZ> boxFilter;
		float x_min = Ti(0,3) - 50, y_min = Ti(1,3) - 50, z_min = Ti(2,3) - 0;
		float x_max = Ti(0,3) + 50, y_max = Ti(1,3) + 50, z_max = Ti(2,3) + 50;

		boxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
		boxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

		boxFilter.setInputCloud(map_final);
		boxFilter.filter(*map_final_boxxed);
	}
	counter_stable_map++;
        //3.2 output the map_final boxxed
	//red is the map_final
	pcl::toROSMsg(*map_final, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_icp_keyframes_.publish(msg_second);  
	
	/*
	//cout<<"==================FinalTransformation_of_map====================="<<endl;
	// 2nd icp for cloud_in_cropbox_for_local and map_final_boxxed
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_for_map;
	icp_for_map.setInputSource(cloud_in_cropbox_for_local);
	icp_for_map.setInputTarget(map_final_boxxed);
	//pcl::PointCloud<pcl::PointXYZ> Final;
	icp_for_map.align(Final);
	//std::cout << "has converged:" << icp_for_map.hasConverged() << " score: " << icp_for_map.getFitnessScore() << std::endl;
	//std::cout << icp_for_map.getFinalTransformation() << std::endl;
	//*/
	
	//3.3 get the gicp of the cloud in boxxed and the map boxxed
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_boxxed_translate_to_near_mapboxxed (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_in_boxxed_for_local, *cloud_in_boxxed_translate_to_near_mapboxxed, Ti);   

	//blue is cloud_pre, the last frame
	pcl::toROSMsg(*cloud_in_boxxed_translate_to_near_mapboxxed, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_recent_keyframes_.publish(msg_second); 
		
	
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_for_map;
	gicp_for_map.setMaxCorrespondenceDistance(10.0);
	gicp_for_map.setTransformationEpsilon(0.01);
	gicp_for_map.setRotationEpsilon(0.01);
	gicp_for_map.setMaximumIterations(1000);
	
	gicp_for_map.setInputSource(cloud_in_boxxed_translate_to_near_mapboxxed);
	gicp_for_map.setInputTarget(map_final_boxxed);
	gicp_for_map.align(Final);
	
	Ti_of_map = gicp_for_map.getFinalTransformation (); // * Ti_of_map;

	Eigen::Matrix4f rotation_matrix = Ti_of_map;

	//double roll = atan2( rotationMatrix(2,1),rotationMatrix(2,2) )/3.1415926*180;
	//std::cout<<"roll is " << roll <<std::endl;
	//double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  )/3.1415926*180;
	//std::cout<<"pitch is " << pitch <<std::endl;
	double yaw_of_cloud_ti_to_map = atan2( rotation_matrix(1,0),rotation_matrix(0,0) )/3.1415926*180;
	std::cout<<"yaw is " << yaw_of_cloud_ti_to_map <<std::endl;
	
	Ti_real = Ti_of_map * Ti;
		
	if( abs( Ti_of_map(0,3) ) > 0.2 || abs( Ti_of_map(1,3) ) > 0.2 || abs( yaw_of_cloud_ti_to_map ) > 1 )
	{
	        cout<<"===========Ti_real=============="<<endl;
	        cout<<Ti_of_map<<endl;
		cout<<Ti<<endl;
		cout<<Ti_real<<endl;
		Ti = Ti_real;

	}

	
	
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_translate (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_in_filtered, *cloud_map_translate, Ti_real);    
	

	
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_final_and_map;
	gicp_final_and_map.setMaxCorrespondenceDistance(1.0);
	gicp_final_and_map.setTransformationEpsilon(0.001);
	gicp_final_and_map.setMaximumIterations(1000);
	
	gicp_final_and_map.setInputSource(cloud_map_translate);
	gicp_final_and_map.setInputTarget(map_final);
	gicp_final_and_map.align(Final);
	
	/*
	//3rd icp for cloud_map_translate and map_final
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_final_and_map;
	icp_final_and_map.setInputSource(cloud_map_translate);
	icp_final_and_map.setInputTarget(map_final);
	//pcl::PointCloud<pcl::PointXYZ> Final;
	icp_final_and_map.align(Final);
	//std::cout << "has converged:" << icp_final_and_map.hasConverged() << " score: " << icp_final_and_map.getFitnessScore() << std::endl;
	//std::cout << icp_final_and_map.getFinalTransformation() << std::endl;
	//*/
	
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_translate (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_map_translate, *Final_translate, gicp_final_and_map.getFinalTransformation ()); 
	//*/
	 

	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud_translate (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_in_filtered, *Final_cloud_translate, Ti_real); 
	//*/

	

	
	if( counter % 5 == 0 )
        {
        
            
	    // 3.1 boxxed the map_final
	    pcl::CropBox<pcl::PointXYZ> box_filter;
	    float x_min = Ti_real(0,3) - 50, y_min = Ti_real(1,3) - 50, z_min = Ti_real(2,3) - 50;
	    float x_max = Ti_real(0,3) + 50, y_max = Ti_real(1,3) + 50, z_max = Ti_real(2,3) + 50;

	    box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
	    box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

	    box_filter.setInputCloud(map_final);
	    box_filter.filter(*map_final_boxxed_2);
	    //*/
        
	    pcl::PointCloud<pcl::PointXYZ> Final_for_add_to_map;
	    
	    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_for_add_to_map_final;
	    gicp_for_add_to_map_final.setMaxCorrespondenceDistance(5.0);
	    gicp_for_add_to_map_final.setTransformationEpsilon(0.001);
	    gicp_for_add_to_map_final.setMaximumIterations(1000);

	    gicp_for_add_to_map_final.setInputSource(Final_cloud_translate);
	    gicp_for_add_to_map_final.setInputTarget(map_final_boxxed_2);
	    gicp_for_add_to_map_final.align(Final_for_add_to_map);
	    
            
            
            /*
	    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_for_add_to_map_final;
	    icp_for_add_to_map_final.setInputSource(Final_translate);
	    icp_for_add_to_map_final.setInputTarget(map_final);
	   //pcl::PointCloud<pcl::PointXYZ> Final;
	    icp_for_add_to_map_final.align(Final_for_add_to_map);
            //*/
            
            std::cout << "============icp==============" <<std::endl;
            std::cout << gicp_for_add_to_map_final.getFinalTransformation () <<std::endl;
            
            /*
	    //Eigen::Matrix4f rotationMatrix = gicp_for_add_to_map_final.getFinalTransformation ();
	    Eigen::Matrix4f rotationMatrix = Ti_of_map;

	    double roll = atan2( rotationMatrix(2,1),rotationMatrix(2,2) )/3.1415926*180;
	    std::cout<<"roll is " << roll <<std::endl;
	    double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  )/3.1415926*180;
	    std::cout<<"pitch is " << pitch <<std::endl;
	    double yaw = atan2( rotationMatrix(1,0),rotationMatrix(0,0) )/3.1415926*180;
	    std::cout<<"yaw is " << yaw <<std::endl;
            //*/
            //if( abs( gicp_for_add_to_map_final.getFinalTransformation ()(0,3) ) < 0.9 && abs( gicp_for_add_to_map_final.getFinalTransformation ()(1,3) ) < 0.9 && abs( gicp_for_add_to_map_final.getFinalTransformation ()(2,3) ) < 0.9 && abs(yaw) < 5 && abs(roll) < 2 && abs(pitch) < 2 )//&& abs( gicp_for_add_to_map_final.getFinalTransformation ()(2,3) ) < 0.5 )
            
            
            if( abs( gicp_for_add_to_map_final.getFinalTransformation ()(0,3) ) < 0.3 && abs( gicp_for_add_to_map_final.getFinalTransformation ()(1,3) ) < 0.3 ) //  && abs( gicp_for_add_to_map_final.getFinalTransformation ()(2,3) ) < 0.01 )
            {
            
		Ti = gicp_for_add_to_map_final.getFinalTransformation () * Ti;
		
		*map_final += Final_for_add_to_map;
		
		pcl::toROSMsg(Final_for_add_to_map, *msg_second);
		msg_second->header.stamp.fromSec(0);
		msg_second->header.frame_id = "map";
		pub_history_keyframes_.publish(msg_second);   
	    }
	    
	}
	
	/*
	if( sqrt( Ti_real(0,3)*Ti_real(0,3) + Ti_real(1,3)*Ti_real(1,3) ) > 50 )
	{
		Ti = Eigen::Matrix4f::Identity ();
		Ti_of_map = Eigen::Matrix4f::Identity ();
		Ti_real = Eigen::Matrix4f::Identity ();
		
		counter = 0;
		counter_stable_map = 0;
		
		map_final.reset(new pcl::PointCloud<pcl::PointXYZ>);
		map_final_boxxed.reset(new pcl::PointCloud<pcl::PointXYZ>);    
		map_final_boxxed_2.reset(new pcl::PointCloud<pcl::PointXYZ>); 
	
	
	}
	//*/
	
	
    }
    
    counter++;
    
    *cloud_pre = *cloud_in_filtered;
  
  //*/   
  
  }

  
  //get the odometry information
  void laserOdomHandler(const nav_msgs::OdometryConstPtr &msg)
  {

  }

  

  void publish_transform()
  {
  

    shared_memory_object shdmem(open_or_create, "Boost", read_write);
    shdmem.truncate(1024);
    mapped_region region(shdmem, read_write);
    double* i1 = static_cast<double*>(region.get_address());

    tf::Quaternion q( -0.00666253 , -0.0385051, 0.0329403, 0.998693);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    PointT this_pose_3d;

    this_pose_3d.x = (*i1)*((-45));
    this_pose_3d.y = (*(i1+1))*80;  
    this_pose_3d.z = (*(i1+2))* (-0.45);
    this_pose_3d.intensity = 1;

    
    
    //cout<<"=============================Odometry================================"<<endl;
    //cout<< Ti_real <<endl;
    
    nav_msgs::OdometryPtr msg_2(new nav_msgs::Odometry);
    msg_2->header.stamp.fromSec(time_laser_odom_);
    msg_2->header.frame_id = "map";
    msg_2->child_frame_id = "/laser";
    msg_2->pose.pose.position.x = Ti_real(0,3);
    msg_2->pose.pose.position.y = Ti_real(1,3);
    msg_2->pose.pose.position.z = Ti_real(2,3);
    msg_2->pose.pose.orientation.w = 1;
    msg_2->pose.pose.orientation.x = 0;
    msg_2->pose.pose.orientation.y = 0;
    msg_2->pose.pose.orientation.z = 0;
    pub_odom_aft_mapped_2.publish(msg_2);
    
    //cout<<"=============================Odometry_2================================"<<endl;

    nav_msgs::OdometryPtr msg_3(new nav_msgs::Odometry);
    msg_3->header.stamp.fromSec(time_laser_odom_);
    msg_3->header.frame_id = "map";
    msg_3->child_frame_id = "/laser";
    msg_3->pose.pose.position.x = Ti(0,3);
    msg_3->pose.pose.position.y = Ti(1,3);
    msg_3->pose.pose.position.z = Ti(2,3);
    msg_3->pose.pose.orientation.w = 1;
    msg_3->pose.pose.orientation.x = 0;
    msg_3->pose.pose.orientation.y = 0;
    msg_3->pose.pose.orientation.z = 0;
    pub_odom_aft_mapped_3.publish(msg_3);
    
    
    kf::Matrix<1, 1>  matrix_F{ kf::Matrix<1, 1>::Identity() };  // 1x1
    matrix_F << 1.0F;
    
    kf::Matrix<1, 1>  matrix_Q{ kf::Matrix<1, 1>::Identity() };  // 1x1
    matrix_Q << 0.05F;
    
    kf::Matrix<2, 2>  matrix_R { kf::Matrix<2, 2>::Identity() }; // 2x2
    matrix_R << 2, 0, 0, 1;
    
    kf::Matrix<2, 1>  matrix_H { kf::Matrix<2, 1>::Random(2,1) };// 2x1
    matrix_H << 1.0F, 1.0F;
    
    
    //std::cout<<"======================kalmanfilter============================="<<std::endl;
    kalmanfilter_x.vector_x() << 1.0F;
    kalmanfilter_x.matrix_P() << 1.0F;
    const kf::Vector<2> vector_z_of_x { Ti_real(0,3), this_pose_3d.x };
    kalmanfilter_x.predict(matrix_F, matrix_Q );
    x = kalmanfilter_x.correct(vector_z_of_x, matrix_R, matrix_H);


    kalmanfilter_y.vector_x() << 1.0F;
    kalmanfilter_y.matrix_P() << 1.0F;
    const kf::Vector<2> vector_z_of_y { Ti_real(1,3), this_pose_3d.y };
    kalmanfilter_y.predict(matrix_F, matrix_Q );
    y = kalmanfilter_y.correct(vector_z_of_y, matrix_R, matrix_H);
    
    //const kf::Vector<2> vector_z_of_z { Ti_real(2,3), this_pose_3d.z };
    //kalmanfilter_z.predict(matrix_F, matrix_Q );
    //double z = kalmanfilter_z.correct(vector_z_of_z, matrix_R, matrix_H);

    z = this_pose_3d.z;
    nav_msgs::OdometryPtr msg_kalman(new nav_msgs::Odometry);
    msg_kalman->header.stamp.fromSec(time_laser_odom_);
    msg_kalman->header.frame_id = "map";
    msg_kalman->child_frame_id = "/laser";
    
    msg_kalman->pose.pose.position.x = Ti_of_map_real(0,3);
    msg_kalman->pose.pose.position.y = Ti_of_map_real(1,3);
    msg_kalman->pose.pose.position.z = Ti_of_map_real(2,3);
    msg_kalman->pose.pose.orientation.w = 1;
    msg_kalman->pose.pose.orientation.x = 0;
    msg_kalman->pose.pose.orientation.y = 0;
    msg_kalman->pose.pose.orientation.z = 0;
    pub_odom_aft_mapped_kalman.publish(msg_kalman);
    
    
    
    
  }

  //main loop thread
  void mainLoop()
  {
    ros::Duration dura(0.001);
    while (ros::ok())
    {
      
      publish_transform();
      
      dura.sleep();
      ros::spinOnce();
    }
  }
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LM");
  ros::NodeHandle nh;
  LM lm(nh);
  lm.mainLoop();
  ros::spin();
  return 0;
}
