#include "alego/utility.h"
#include <std_srvs/Empty.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <iostream>
#include <stdint.h>
#include <vector>
#include <random>
#include <cmath>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using namespace std;

class LM
{
private:

  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odometryNoise;
  noiseModel::Diagonal::shared_ptr constraintNoise;
  noiseModel::Base::shared_ptr robustNoiseModel;

  //initilize the ros node
  ros::NodeHandle nh_;
  
  //initialize the Publisher
  ros::Publisher pub_cloud_surround_;
  ros::Publisher pub_odom_aft_mapped_;
  ros::Publisher pub_keyposes_;

  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;
  ros::Publisher pub_recent_keyframes_;
  ros::Publisher pub_map_all_;

  double time_laser_odom_;

  int counter = 0;
  int counter_stable_map = 0;
  int counter_all_map = 0;
  
  ros::Subscriber sub_pc_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pre;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_boxed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_boxed_2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_last;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_all;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_in_all_map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_final_in_all_map_boxed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_all_boxed;
    
  
  
    
  Eigen::Matrix4f Ti;
  Eigen::Matrix4f Ti_of_map;
  Eigen::Matrix4f Ti_of_map_real;
  Eigen::Matrix4f Ti_translation;
  Eigen::Matrix4f Ti_real;
  Eigen::Matrix4f Ti_of_map_for_all;
  
  Eigen::Matrix4f Ti_all; 
  Eigen::Matrix4f Ti_real_all; 
  Eigen::Matrix4f Ti_real_last_submap_saved;

  Eigen::Matrix4f Ti_transformLast;  
  
  
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
    pub_map_all_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_all", 10);
    
    sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, boost::bind(&LM::main_callback, this, _1));
    
    cloud_pre.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final_boxed.reset(new pcl::PointCloud<pcl::PointXYZ>);    
    map_final_boxed_2.reset(new pcl::PointCloud<pcl::PointXYZ>);  
    map_last.reset(new pcl::PointCloud<pcl::PointXYZ>);  
    map_all.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final_in_all_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_final_in_all_map_boxed.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_all_boxed.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    Ti = Eigen::Matrix4f::Identity ();
    Ti_of_map = Eigen::Matrix4f::Identity ();
    Ti_of_map_real = Eigen::Matrix4f::Identity ();
    Ti_translation = Eigen::Matrix4f::Identity ();
    Ti_real = Eigen::Matrix4f::Identity ();
    Ti_of_map_for_all = Eigen::Matrix4f::Identity ();
    
    Ti_all = Eigen::Matrix4f::Identity ();
    Ti_real_all = Eigen::Matrix4f::Identity ();
    Ti_real_last_submap_saved = Eigen::Matrix4f::Identity ();
    
    Ti_transformLast = Eigen::Matrix4f::Identity ();
    
    pub_odom_aft_mapped_2 = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_2", 10);
    pub_odom_aft_mapped_3 = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_3", 10);
    pub_odom_aft_mapped_kalman = nh_.advertise<nav_msgs::Odometry>("/odom_aft_mapped_kalman", 10);
    
    
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);
    
  }
  
  
  
  //process the input cloud
  void main_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
  
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_boxed_for_local (new pcl::PointCloud<pcl::PointXYZ>(5,1));
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
	
	// 1.2 boxed the cloud
	*cloud_in_boxed_for_local = *cloud_in_filtered;
	
        pcl::CropBox<pcl::PointXYZ> boxFilter_for_in;
	float x_min_for_in = - 50, y_min_for_in = - 50, z_min_for_in = + 0 ;
	float x_max_for_in = + 50, y_max_for_in = + 50, z_max_for_in = + 50;
	
	boxFilter_for_in.setMin(Eigen::Vector4f(x_min_for_in, y_min_for_in, z_min_for_in, 1.0));
	boxFilter_for_in.setMax(Eigen::Vector4f(x_max_for_in, y_max_for_in, z_max_for_in, 1.0));

	boxFilter_for_in.setInputCloud(cloud_in_boxed_for_local);
	boxFilter_for_in.filter(*cloud_in_boxed_for_local);
	

	pcl::PointCloud<pcl::PointXYZ> Final;
	
	if( counter < 2 )
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
	
	
	// 2.1 output the cloud_in_boxed_for_local and cloud_pre
	//brown is the input cloud of right now frame
	sensor_msgs::PointCloud2Ptr msg_second(new sensor_msgs::PointCloud2);
	//cout<<"==================cloud_in====================="<<endl;
	pcl::toROSMsg(*cloud_in_boxed_for_local, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_cloud_surround_.publish(msg_second);
	//*/

  
	
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
		// 3.1 boxed the map_final
		pcl::CropBox<pcl::PointXYZ> boxFilter;
		float x_min = Ti(0,3) - 50, y_min = Ti(1,3) - 50, z_min = Ti(2,3) - 0;
		float x_max = Ti(0,3) + 50, y_max = Ti(1,3) + 50, z_max = Ti(2,3) + 50;

		boxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
		boxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

		boxFilter.setInputCloud(map_final);
		boxFilter.filter(*map_final_boxed);
	}
	counter_stable_map++;
        //3.2 output the map_final boxed
	//red is the map_final
	pcl::toROSMsg(*map_final, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_icp_keyframes_.publish(msg_second);  
	
	//3.3 get the gicp of the cloud in boxed and the map boxed
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_boxed_translate_to_near_mapboxed (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_in_boxed_for_local, *cloud_in_boxed_translate_to_near_mapboxed, Ti);   

	//blue is cloud_pre, the last frame
	pcl::toROSMsg(*cloud_in_boxed_translate_to_near_mapboxed, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_recent_keyframes_.publish(msg_second); 
		
	
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_for_map;
	gicp_for_map.setMaxCorrespondenceDistance(10.0);
	gicp_for_map.setTransformationEpsilon(0.01);
	gicp_for_map.setRotationEpsilon(0.01);
	gicp_for_map.setMaximumIterations(1000);
	
	gicp_for_map.setInputSource(cloud_in_boxed_translate_to_near_mapboxed);
	gicp_for_map.setInputTarget(map_final_boxed);
	gicp_for_map.align(Final);
	
	Ti_of_map = gicp_for_map.getFinalTransformation (); // * Ti_of_map;

	Eigen::Matrix4f rotation_matrix = Ti_of_map;

	//double roll = atan2( rotationMatrix(2,1),rotationMatrix(2,2) )/3.1415926*180;
	//std::cout<<"roll is " << roll <<std::endl;
	//double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  )/3.1415926*180;
	//std::cout<<"pitch is " << pitch <<std::endl;
	double yaw_of_cloud_ti_to_map = atan2( rotation_matrix(1,0),rotation_matrix(0,0) )/3.1415926*180;
	//std::cout<<"yaw is " << yaw_of_cloud_ti_to_map <<std::endl;
	
	Ti_real = Ti_of_map * Ti;
	Ti_real_all = Ti_real_last_submap_saved * Ti_real ;
	

		
	if( abs( Ti_of_map(0,3) ) > 0.2 || abs( Ti_of_map(1,3) ) > 0.2 ||  abs( yaw_of_cloud_ti_to_map ) > 1)
	{
	        //cout<<"===========Ti_real=============="<<endl;
	        //cout<<Ti_of_map<<endl;
		//cout<<Ti<<endl;
		//cout<<Ti_real<<endl;
		Ti = Ti_real;
		

	}

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud_translate (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::transformPointCloud (*cloud_in_filtered, *Final_cloud_translate, Ti_real); 
	//*/

	

	
	if( counter % 5 == 0 )
        {
        
            
	    // 3.1 boxed the map_final
	    pcl::CropBox<pcl::PointXYZ> box_filter;
	    float x_min = Ti_real(0,3) - 50, y_min = Ti_real(1,3) - 50, z_min = Ti_real(2,3) - 50;
	    float x_max = Ti_real(0,3) + 50, y_max = Ti_real(1,3) + 50, z_max = Ti_real(2,3) + 50;

	    box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
	    box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

	    box_filter.setInputCloud(map_final);
	    box_filter.filter(*map_final_boxed_2);
	    //*/
        
	    pcl::PointCloud<pcl::PointXYZ> Final_for_add_to_map;
	    
	    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_for_add_to_map_final;
	    gicp_for_add_to_map_final.setMaxCorrespondenceDistance(5.0);
	    gicp_for_add_to_map_final.setTransformationEpsilon(0.001);
	    gicp_for_add_to_map_final.setMaximumIterations(1000);

	    gicp_for_add_to_map_final.setInputSource(Final_cloud_translate);
	    gicp_for_add_to_map_final.setInputTarget(map_final_boxed_2);
	    gicp_for_add_to_map_final.align(Final_for_add_to_map);
	    
            
            
            if( abs( gicp_for_add_to_map_final.getFinalTransformation ()(0,3) ) < 0.3 && abs( gicp_for_add_to_map_final.getFinalTransformation ()(1,3) ) < 0.3 ) 
            {
            
		Ti = gicp_for_add_to_map_final.getFinalTransformation () * Ti;
		
		*map_final += Final_for_add_to_map;
		
		pcl::toROSMsg(Final_for_add_to_map, *msg_second);
		msg_second->header.stamp.fromSec(0);
		msg_second->header.frame_id = "map";
		pub_history_keyframes_.publish(msg_second);   
	    }   
	}
    }
     
    counter++;
    
    *cloud_pre = *cloud_in_filtered;
  
  //*/   
  
  
    if( sqrt( Ti_real(0,3)*Ti_real(0,3) + Ti_real(1,3)*Ti_real(1,3) ) > 20 )
    {
    
        
        pcl::transformPointCloud (*map_final, *map_final_in_all_map, Ti_real_last_submap_saved);
        
	//Ti_real_last_submap_saved = Ti_real_all;
	//*map_all += *map_final_in_all_map;
	
	
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor_input;
	sor_input.setInputCloud (map_all);
	sor_input.setLeafSize (0.1f, 0.1f, 0.1f);
	sor_input.filter (*map_all);
	
	
	
	
	if( counter_all_map == 0)
	{
		Ti_real_last_submap_saved = Ti_real_all;
		*map_all += *map_final_in_all_map;
		
		
		cout<<"============Ti_real_last_submap_saved_pose[0]============="<<endl;
		cout<<Ti_real_last_submap_saved<<endl;
		
		Eigen::Matrix4f rotation_matrix = Ti_real_last_submap_saved;

		double roll = atan2( rotation_matrix(2,1),rotation_matrix(2,2) );
		//std::cout<<"roll is " << roll <<std::endl;
		double pitch = atan2( -rotation_matrix(2,0), std::pow( rotation_matrix(2,1)*rotation_matrix(2,1) +rotation_matrix(2,2)*rotation_matrix(2,2) ,0.5  )  );
		//std::cout<<"pitch is " << pitch <<std::endl;
		double yaw = atan2( rotation_matrix(1,0),rotation_matrix(0,0) );
		
		
		gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix(0,3), rotation_matrix(1,3), rotation_matrix(2,3))), priorNoise));
		
            	initialEstimate.insert(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix(0,3), rotation_matrix(1,3), rotation_matrix(2,3))));
            
            	
            	//update iSAM
		//cout<<"============gtsam_pose[0]============="<<endl;
		isam->update(gtSAMgraph, initialEstimate);
		isam->update();
		//cout<<"============gtsam_pose[1]============="<<endl;
		gtSAMgraph.resize(0);
		initialEstimate.clear();
		
		Pose3 latestEstimate;
		isamCurrentEstimate = isam->calculateEstimate();
		latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
		
		cout<<"============gtsam_pose[0]============="<<endl;
		cout<<latestEstimate<<endl;
		
		Ti_transformLast = Ti_real_last_submap_saved;
            	
		
	}
	else
	{
		Ti_real_last_submap_saved = Ti_real_all;
		*map_all += *map_final_in_all_map;
		
		
		cout<<"============Ti_real_last_submap_saved_pose============="<<endl;
		cout<<Ti_real_last_submap_saved<<endl;
	
		Eigen::Matrix4f rotation_matrix_from = Ti_transformLast;

		double roll = atan2( rotation_matrix_from(2,1),rotation_matrix_from(2,2) );
		//std::cout<<"roll is " << roll <<std::endl;
		double pitch = atan2( -rotation_matrix_from(2,0), std::pow( rotation_matrix_from(2,1)*rotation_matrix_from(2,1) +rotation_matrix_from(2,2)*rotation_matrix_from(2,2) ,0.5  )  );
		//std::cout<<"pitch is " << pitch <<std::endl;
		double yaw = atan2( rotation_matrix_from(1,0),rotation_matrix_from(0,0) );
	
		gtsam::Pose3 poseFrom = Pose3( Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix_from(0,3), rotation_matrix_from(1,3), rotation_matrix_from(2,3)));
		
		
		Eigen::Matrix4f rotation_matrix_to = Ti_real_last_submap_saved;

		roll = atan2( rotation_matrix_to(2,1),rotation_matrix_to(2,2) );
		//std::cout<<"roll is " << roll <<std::endl;
		pitch = atan2( -rotation_matrix_to(2,0), std::pow( rotation_matrix_to(2,1)*rotation_matrix_to(2,1) +rotation_matrix_to(2,2)*rotation_matrix_to(2,2) ,0.5  )  );
		//std::cout<<"pitch is " << pitch <<std::endl;
		yaw = atan2( rotation_matrix_to(1,0),rotation_matrix_to(0,0) );
		
		gtsam::Pose3 poseTo   = Pose3( Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix_to(0,3), rotation_matrix_to(1,3), rotation_matrix_to(2,3)));
		
		gtSAMgraph.add(BetweenFactor<Pose3>(counter_all_map-1, counter_all_map, poseFrom.between(poseTo), odometryNoise));
		
		initialEstimate.insert(counter_all_map, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix_to(0,3), rotation_matrix_to(1,3), rotation_matrix_to(2,3))));
	

		
		//update iSAM
		//cout<<"============gtsam_pose[2]============="<<endl;
		isam->update(gtSAMgraph, initialEstimate);
		isam->update();
		//cout<<"============gtsam_pose[3]============="<<endl;
		gtSAMgraph.resize(0);
		initialEstimate.clear();
		
		Pose3 latestEstimate;
		isamCurrentEstimate = isam->calculateEstimate();
		latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
		
		cout<<"============gtsam_pose============="<<endl;
		cout<<latestEstimate<<endl;
		
		latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-2);
		cout<<latestEstimate<<endl;
		
		Ti_transformLast = Ti_real_last_submap_saved;
		
		
		/*
		// 3.1 boxed the map submap
		pcl::CropBox<pcl::PointXYZ> box_filter_submap;
		float x_min = Ti_real_last_submap_saved(0,3) - 50, y_min = Ti_real_last_submap_saved(1,3) - 50, z_min = Ti_real_last_submap_saved(2,3) - 0;
		float x_max = Ti_real_last_submap_saved(0,3) + 50, y_max = Ti_real_last_submap_saved(1,3) + 50, z_max = Ti_real_last_submap_saved(2,3) + 50;

		box_filter_submap.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
		box_filter_submap.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

		box_filter_submap.setInputCloud(map_final_in_all_map);
		box_filter_submap.filter(*map_final_in_all_map_boxed);

				
		box_filter_submap.setInputCloud(map_all);
		box_filter_submap.filter(*map_all_boxed);
		//*/
		
		
		/*
		pcl::PointCloud<pcl::PointXYZ> Final_for_add_to_all;

		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_for_add_to_map_all;
		gicp_for_add_to_map_all.setMaxCorrespondenceDistance(10.0);
		gicp_for_add_to_map_all.setTransformationEpsilon(0.001);
		gicp_for_add_to_map_all.setMaximumIterations(1000);

		gicp_for_add_to_map_all.setInputSource(map_final_in_all_map_boxed);
		gicp_for_add_to_map_all.setInputTarget(map_all_boxed);
		gicp_for_add_to_map_all.align(Final_for_add_to_all);
		
		
		
		
		Ti_of_map_for_all = gicp_for_add_to_map_all.getFinalTransformation (); // * Ti_of_map;

		Eigen::Matrix4f rotation_matrix = Ti_of_map_for_all;

		//double roll = atan2( rotationMatrix(2,1),rotationMatrix(2,2) )/3.1415926*180;
		//std::cout<<"roll is " << roll <<std::endl;
		//double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  )/3.1415926*180;
		//std::cout<<"pitch is " << pitch <<std::endl;
		double yaw_of_cloud_ti_of_map_all_once = atan2( rotation_matrix(1,0),rotation_matrix(0,0) )/3.1415926*180;
		//std::cout<<"yaw is " << yaw_of_cloud_ti_to_map <<std::endl;
		
		double x = gicp_for_add_to_map_all.getFinalTransformation ()(0,3);
		double y = gicp_for_add_to_map_all.getFinalTransformation ()(1,3);
		
		cout<<"========================gicp_for_add_to_map_all========================"<<endl;
		cout<<"x="<<x<<endl;
		cout<<"y="<<y<<endl;
		cout<<"theta="<<yaw_of_cloud_ti_of_map_all_once<<endl;
		
		/*
		if( abs( yaw_of_cloud_ti_of_map_all_once ) < 2.0)
		{
		
			pcl::transformPointCloud (*map_final_in_all_map, *map_final_in_all_map, gicp_for_add_to_map_all.getFinalTransformation ());
		        cout<<"========================gicp========================"<<endl;
			Ti_real_last_submap_saved = gicp_for_add_to_map_all.getFinalTransformation ()  *  Ti_real_all;
			*map_all += *map_final_in_all_map;
		}
		else
		{
			cout<<"========================submap========================"<<endl;
			*map_all += *map_final_in_all_map;
		}
		*/
		//
		
		/*
		if( ( abs( x ) > 0.5 && abs( x ) < 3.9 ) || ( abs( y ) > 0.5 && abs( y ) < 3.9 ) && (abs( yaw_of_cloud_ti_of_map_all_once ) > 1  && abs( yaw_of_cloud_ti_of_map_all_once ) < 3.9)  )
		{
			cout<<"========================gicp========================"<<endl;
			pcl::transformPointCloud (*map_final_in_all_map, *map_final_in_all_map, gicp_for_add_to_map_all.getFinalTransformation ());
			
			Ti_real_last_submap_saved = gicp_for_add_to_map_all.getFinalTransformation ()  *  Ti_real_all;
			*map_all += *map_final_in_all_map;
		}
		else
		{	
			cout<<"========================submap========================"<<endl;
			Ti_real_last_submap_saved = Ti_real_all;
			*map_all += *map_final_in_all_map;
		}
	
		//*/

	}
	//*/
	
	
	if( counter_all_map == 16 )
	{
	
		Eigen::Matrix4f rotation_matrix_from = Ti_real_last_submap_saved;

		double roll = atan2( rotation_matrix_from(2,1),rotation_matrix_from(2,2) );
		//std::cout<<"roll is " << roll <<std::endl;
		double pitch = atan2( -rotation_matrix_from(2,0), std::pow( rotation_matrix_from(2,1)*rotation_matrix_from(2,1) +rotation_matrix_from(2,2)*rotation_matrix_from(2,2) ,0.5  )  );
		//std::cout<<"pitch is " << pitch <<std::endl;
		double yaw = atan2( rotation_matrix_from(1,0),rotation_matrix_from(0,0) );
		
		gtsam::Pose3 poseFrom   = Pose3( Rot3::RzRyRx(roll, pitch, yaw), Point3(rotation_matrix_from(0,3), rotation_matrix_from(1,3), rotation_matrix_from(2,3)));
		
	
	
		//gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
               gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(20.2164, 1.62224, -0.0866));
	
		gtSAMgraph.add(BetweenFactor<Pose3>(16, 0, poseFrom.between(poseTo), robustNoiseModel)); // giseop
		isam->update(gtSAMgraph);
		isam->update();
		
		Pose3 latestEstimate;
		isamCurrentEstimate = isam->calculateEstimate();
		latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
		
		cout<<"============gtsam_pose_last============="<<endl;
		cout<<latestEstimate<<endl;
		
		latestEstimate = isamCurrentEstimate.at<Pose3>(1);
		cout<<latestEstimate<<endl;
		latestEstimate = isamCurrentEstimate.at<Pose3>(2);
		cout<<latestEstimate<<endl;
		latestEstimate = isamCurrentEstimate.at<Pose3>(3);
		cout<<latestEstimate<<endl;
	
	}
	
	
	
	cout<<"====counter_all_map====="<<endl;
	cout<<counter_all_map<<endl;
	counter_all_map++;
	//*/


	
	/*
	sensor_msgs::PointCloud2Ptr msg_second(new sensor_msgs::PointCloud2);
	//cout<<"==================cloud_in====================="<<endl;
	pcl::toROSMsg(*map_final_in_all_map_boxed, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_cloud_surround_.publish(msg_second);
	//*/	
		
	sensor_msgs::PointCloud2Ptr msg_second(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*map_all, *msg_second);
	msg_second->header.stamp.fromSec(0);
	msg_second->header.frame_id = "map";
	pub_map_all_.publish(msg_second); 
	
	        
	Ti = Eigen::Matrix4f::Identity ();
	Ti_of_map = Eigen::Matrix4f::Identity ();
	Ti_real = Eigen::Matrix4f::Identity ();

	counter = 0;
	counter_stable_map = 0;

	map_final.reset(new pcl::PointCloud<pcl::PointXYZ>);
	map_final_boxed.reset(new pcl::PointCloud<pcl::PointXYZ>);    
	map_final_boxed_2.reset(new pcl::PointCloud<pcl::PointXYZ>); 


	*map_final_in_all_map_boxed = *map_final_in_all_map;

    }
    
    //*/
  
  
  
  }

  

  void publish_transform()
  {
  
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
    
    nav_msgs::OdometryPtr msg_3(new nav_msgs::Odometry);
    msg_3->header.stamp.fromSec(time_laser_odom_);
    msg_3->header.frame_id = "map";
    msg_3->child_frame_id = "/laser";
    msg_3->pose.pose.position.x = Ti_real_last_submap_saved(0,3);
    msg_3->pose.pose.position.y = Ti_real_last_submap_saved(1,3);
    msg_3->pose.pose.position.z = Ti_real_last_submap_saved(2,3);
    msg_3->pose.pose.orientation.w = 1;
    msg_3->pose.pose.orientation.x = 0;
    msg_3->pose.pose.orientation.y = 0;
    msg_3->pose.pose.orientation.z = 0;
    pub_odom_aft_mapped_3.publish(msg_3);
    

    nav_msgs::OdometryPtr msg_kalman(new nav_msgs::Odometry);
    msg_kalman->header.stamp.fromSec(time_laser_odom_);
    msg_kalman->header.frame_id = "map";
    msg_kalman->child_frame_id = "/laser";
    
    msg_kalman->pose.pose.position.x = Ti_real_all(0,3);
    msg_kalman->pose.pose.position.y = Ti_real_all(1,3);
    msg_kalman->pose.pose.position.z = Ti_real_all(2,3);
    msg_kalman->pose.pose.orientation.w = 1;
    msg_kalman->pose.pose.orientation.x = 0;
    msg_kalman->pose.pose.orientation.y = 0;
    msg_kalman->pose.pose.orientation.z = 0;
    pub_odom_aft_mapped_kalman.publish(msg_kalman);
    
    
    
    
  }

  //main loop thread
  void mainLoop()
  {
    ros::Duration dura(0.01);
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
