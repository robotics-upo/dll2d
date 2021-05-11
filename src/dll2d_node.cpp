#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dll2d/tdf2d.hpp>
#include <dll2d/dll2dsolver.hpp>
#include "ElapsedTime.hpp"
#include <time.h> 

using std::isnan;

//Class definition
class DLL2DNode
{
public:

	//!Default contructor 
	DLL2DNode(std::string &node_name) : m_solver((DF2D *)&m_grid2d)
	{		
		// Read node parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("in_scan", m_inScanTopic))
			m_inScanTopic = "/scan";	
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";	
		if(!lnh.getParam("odom_frame_id", m_odomFrameId))
			m_odomFrameId = "odom";	
		if(!lnh.getParam("map_frame_id", m_globalFrameId))
			m_globalFrameId = "map";	
		
		// Read DLL parameters
		double initX, initY, initA;
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 10.0;
		if(!lnh.getParam("initial_x", initX))
			initX = 0.0;
		if(!lnh.getParam("initial_y", initY))
			initY = 0.0;
		if(!lnh.getParam("initial_a", initA))
			initA = 0.0;	
		if(!lnh.getParam("update_min_d", m_dTh))
			m_dTh = 0.2;
		if(!lnh.getParam("update_min_a", m_aTh))
			m_aTh = 0.1;
		if(!lnh.getParam("update_min_t", m_tTh))
			m_tTh = 1.0;								
		if(!lnh.getParam("solver_max_iter", m_solverMaxIter))
			m_solverMaxIter = 50;
		if(!lnh.getParam("solver_max_threads", m_solverMaxThreads))
			m_solverMaxThreads = 1;
		
		// Init internal variables
		m_tfCache = false;
		m_init = false;
		m_doUpdate = false;
		m_mapLoaded = false;
		
		// Launch subscribers
		m_scanSub = m_nh.subscribe(m_inScanTopic, 1, &DLL2DNode::scanCallback, this);	
		m_mapSub = m_nh.subscribe("map", 1, &DLL2DNode::mapCallback, this);
		m_initialPoseSub = lnh.subscribe("initial_pose", 2, &DLL2DNode::initialPoseReceived, this);

		// Solver setup
		m_solver.setMaxIterations(m_solverMaxIter);
		m_solver.setMaxThreads(m_solverMaxThreads);

		// Time stamp for periodic update
		m_lastPeriodicUpdate = ros::Time::now();

		// Launch updater timer
		m_updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &DLL2DNode::checkUpdateThresholdsTimer, this);
		
		// Initialize TF from odom to map as identity
		m_lastGlobalTf.setIdentity();
				
		if(initX != 0 || initY != 0 || initA != 0)
		{
			tf::Pose pose;
			tf::Vector3 origin(initX, initY, 0);
			tf::Quaternion q;
			q.setRPY(0, 0, initA);

			pose.setOrigin(origin);
			pose.setRotation(q);

			std::cout << "Initializing DLL2D position..." << std::endl;
			while(!setInitialPose(pose) && ros::ok());
			std::cout << "done!" << std::endl;
	
			m_init = true;
		}
	}

	//!Default destructor
	~DLL2DNode()
	{
	}

	//! Check motion and time thresholds for position update
	bool checkUpdateThresholds()
	{
		// If the filter is not initialized then exit
		if(!m_init || !m_mapLoaded)
			return false;
					
		// Publish current TF from odom to map
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
		
		// Compute odometric translation and rotation since last update 
		ros::Time t = ros::Time::now();
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("DLL2D error: %s",ex.what());
			return false;
		}
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
		
		// Check translation threshold
		if(T.getOrigin().length() > m_dTh)
		{
            m_doUpdate = true;
			m_lastPeriodicUpdate = t;
			return true;
		}
		
		// Check yaw threshold
		double yaw, pitch, roll;
		T.getBasis().getRPY(roll, pitch, yaw);
		if(fabs(yaw) > m_aTh)
		{
			m_doUpdate = true;
			m_lastPeriodicUpdate = t;
			return true;
		}

		// Check time threshold
		if((t-m_lastPeriodicUpdate).toSec() > m_tTh)
		{
			m_doUpdate = true;
			m_lastPeriodicUpdate = t;
			return true;
		}
		
		return false;
	}
		                                   
private:
	//! Timer callback
	void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
	{
		checkUpdateThresholds();
	}

	//! Map callback
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
	{
		if(!m_mapLoaded)
		{
			// Load occupancy grid into TDF grid
			std::cout << "Map received. Computing distance function...\n";
			m_grid2d.loadGrid(*msg);
			std::cout << "done!\n";
			
			m_mapLoaded = true;
		} 
	}
	
	//! Initial pose callback
	void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		// We only accept initial pose estimates in the global frame
		if(msg->header.frame_id != m_globalFrameId)
		{
			ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
			msg->header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}
				
		// Initialize the filter
		tf::Pose pose;
		tf::poseMsgToTF(msg->pose.pose, pose);
		setInitialPose(pose);
	}

	//! Set the initial pose of the particle filter
	bool setInitialPose(tf::Pose initPose)
	{
		// Extract TFs for future updates
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("DLL2D error: %s",ex.what());
			return false;
		}

		// Get position information from pose
		double roll, pitch, yaw;
		tf::Vector3 t = initPose.getOrigin();
		initPose.getBasis().getRPY(roll, pitch, yaw);
		
		// Update global TF
		tf::Quaternion q;
		q.setRPY(0, 0, yaw);
		t.setZ(0.0);
		m_lastGlobalTf = tf::Transform(q, t)*m_lastOdomTf.inverse();
		std::cout << "Pose initialized to x: " << t.getX() << ", y: " << t.getY() << ", yaw: " << yaw << std::endl;

		// Prepare next iterations		
		m_doUpdate = false;
		m_init = true;

		return true;
	}

	//! laser scan callback
	void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
	{
		// If the filter is not initialized then exit
		if(!m_init  || !m_mapLoaded || !m_doUpdate)
			return;

		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}
		tf::Transform mapTf;
		mapTf = m_lastGlobalTf * odomTf;

		// Pre-cache transform for laser-scan to base frame
		if(!m_tfCache)
		{	
			try
			{
                m_tfListener.waitForTransform(m_baseFrameId, scan->header.frame_id, ros::Time(0), ros::Duration(2.0));
                m_tfListener.lookupTransform(m_baseFrameId, scan->header.frame_id, ros::Time(0), m_scanTf);
				m_tfCache = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}
		}

		// Transform the scan into a point-cloud in the base frame
		std::vector<Point2D> cloud;
		if(!scan2cloud(*scan, cloud, m_scanTf))
		{
			ROS_ERROR("Error transforming input scan from %s to %s frame", scan->header.frame_id.c_str(), m_baseFrameId.c_str());
			return;
		}	

		// Get estimated position and orientation into the map
		double tx, ty, roll, pitch, yaw;
		tx = mapTf.getOrigin().getX();
		ty = mapTf.getOrigin().getY();
		mapTf.getBasis().getRPY(roll, pitch, yaw);

		// Align point-cloud to current map
		bool converged = false;
		ElapsedTime t;
		converged = m_solver.solve(cloud, tx, ty, yaw);
		std::cout << "Optimization time: " << t.tock() << "\n";
		std::cout << "tx: " << tx << ", ty: " << ty << ", yaw: " <<  yaw << "\n\n";

		// Update global TF and publish it
		tf::Quaternion q;
		q.setRPY(0, 0, yaw);
		m_lastGlobalTf = tf::Transform(q, tf::Vector3(tx, ty , 0))*odomTf.inverse();
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));

		// Update time and transform information
		m_lastOdomTf = odomTf;
		m_doUpdate = false;
	}

	//! Transform scan into point-cloud applying the given transform
	bool scan2cloud(const sensor_msgs::LaserScan &scan, std::vector<Point2D> &cloud, const tf::StampedTransform &tf)
	{
		// Clear point cloud
		cloud.clear();

		// Sanity check
		if(scan.ranges.size() <= 0)
			return false;
			
		// Get transform
		double tx, ty, tz;
		tx = tf.getOrigin().getX();
		ty = tf.getOrigin().getY();
		tz = tf.getOrigin().getZ();
		tf::Matrix3x3 R = tf.getBasis(); 
		
		// Project and transform points 
		Point2D p;
		float a, r, x, y, z;
		a=scan.angle_min;
		for(uint32_t i=0; i<scan.ranges.size(); i++) 
		{
			r = scan.ranges[i];
			if(r >= scan.range_min && r<= scan.range_max)
			{
				x = r*cos(a); 
				y = r*sin(a); 
				z = 0;
				p.x = x*R[0][0] + y*R[0][1] + z*R[0][2] + tx;
				p.y = x*R[1][0] + y*R[1][1] + z*R[1][2] + ty;
				cloud.push_back(p);
			}
			a = a+scan.angle_increment;	
		}

		return true;
	}
	
	//! Indicates that the local transfrom for the pint-cloud is cached
	bool m_tfCache;
	tf::StampedTransform m_scanTf;
			
	//! Node parameters
	std::string m_inScanTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	std::string m_globalFrameId;
	
	//! ROS msgs and data
	ros::NodeHandle m_nh;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
    ros::Subscriber m_scanSub, m_mapSub, m_initialPoseSub;

	//! Thresholds and params for filter updating
	double m_dTh, m_aTh, m_tTh;
	tf::StampedTransform m_lastOdomTf;
	tf::Transform m_lastGlobalTf;
	bool m_doUpdate, m_init;
	double m_updateRate, m_mapLoaded;
	ros::Time m_lastPeriodicUpdate;
	ros::Timer m_updateTimer;
	
	//! 2D distance grid
    TDF2D m_grid2d;
	double m_tdfGridSize, m_tdfGridRes;
		
	//! Non-linear optimization solver
	DLL2DSolver m_solver;
	int m_solverMaxIter, m_solverMaxThreads;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dll2d_node");  
	
	// Particle filter instance
	std::string node_name = "dll2d_node";
	DLL2DNode node(node_name);
	
	// Process data at given rate
	ros::spin();

	return 0;
}




