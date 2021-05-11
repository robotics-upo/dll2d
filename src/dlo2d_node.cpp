#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <dll2d/tdf2d.hpp>
#include <dll2d/dll2dsolver.hpp>
#include "ElapsedTime.hpp"
#include <time.h> 

using std::isnan;

//Class definition
class DLO2DNode
{
public:

	//!Default contructor 
	DLO2DNode(std::string &node_name) : m_solver((DF2D *)&m_grid2d)
	{		
		// Read node parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("in_scan", m_inScanTopic))
			m_inScanTopic = "/scan";	
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";	
		if(!lnh.getParam("odom_frame_id", m_odomFrameId))
			m_odomFrameId = "odom";	
		
		// Read DLO parameters
		if(!lnh.getParam("keyframe_dist", m_keyFrameDist))
			m_keyFrameDist = 2.0;
		if(!lnh.getParam("keyframe_rot", m_keyFrameRot))
			m_keyFrameRot = 1.0;															
		if(!lnh.getParam("tdf_grid_size", m_tdfGridSize))
			m_tdfGridSize = 60.0;
		if(!lnh.getParam("tdf_grid_res", m_tdfGridRes))
			m_tdfGridRes = 0.05;
		if(!lnh.getParam("solver_max_iter", m_solverMaxIter))
			m_solverMaxIter = 50;
		if(!lnh.getParam("solver_max_threads", m_solverMaxThreads))
			m_solverMaxThreads = 1;
		
		// Init internal variables
		m_tfCache = false;
		m_tx = m_ty = m_ta = 0.0;
		m_keyFrameInit = false;
	 	m_kx = m_ky = m_ka = 0.0;
		
		// Launch subscribers
		m_scanSub = m_nh.subscribe(m_inScanTopic, 1, &DLO2DNode::scanCallback, this);		

		// Setup TDF grid
		m_grid2d.setup(-m_tdfGridSize/2, m_tdfGridSize/2, -m_tdfGridSize/2, m_tdfGridSize/2, m_tdfGridRes);

		// Solver setup
		m_solver.setMaxIterations(m_solverMaxIter);
		m_solver.setMaxThreads(m_solverMaxThreads);
	}

	//!Default destructor
	~DLO2DNode()
	{
	}
		                                   
private:
	
	//! laser scan callback
	void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
	{		
		// Pre-cache transform for point-cloud to base frame and transform the pc
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

		// Do we have already a key-frame?
		ElapsedTime t;
		bool converged = false;
		if(!m_keyFrameInit)
		{
			// Reset key-frame variables 
			m_kx = m_ky = m_ka = 0.0;
			m_grid2d.clear();

			// Load current scan as new key-frame
			m_grid2d.loadCloud(cloud);

			// Set flag
			m_keyFrameInit = true;
			converged = true;
		}
		else
		 	// Align point-cloud to current map
			converged = m_solver.solve(cloud, m_kx, m_ky, m_ka);

		// Check if crossed the distance thresholds for key-framing
		if(m_kx*m_kx + m_ky*m_ky > m_keyFrameDist*m_keyFrameDist || fabs(m_ka) > m_keyFrameRot || !converged)
		{
			// Add keyframe transform into odom
			m_tx += m_kx*cos(m_ta) - m_ky*sin(m_ta);
			m_ty += m_kx*sin(m_ta) + m_ky*cos(m_ta);
			m_ta += m_ka;

			// Reset key-frame variables 
			m_kx = m_ky = m_ka = 0.0;
			m_grid2d.clear();

			// Load current scan as new key-frame
			m_grid2d.loadCloud(cloud);
		}
		std::cout << "Optimization time: " << t.tock() << "\n";
		std::cout << "tx: " << m_tx + m_kx*cos(m_ta) - m_ky*sin(m_ta) << ", ty: " << m_ty + m_kx*sin(m_ta) + m_ky*cos(m_ta) << ", yaw: " <<  m_ta + m_ka << "\n\n";

		// Create and publish odom transform
		tf::Quaternion q;
		q.setRPY(0, 0, m_ta + m_ka);
		tf::Transform odomTf = tf::Transform(q, tf::Vector3(m_tx + m_kx*cos(m_ta) - m_ky*sin(m_ta), m_ty + m_kx*sin(m_ta) + m_ky*cos(m_ta), 0));
		m_tfBr.sendTransform(tf::StampedTransform(odomTf, scan->header.stamp, m_odomFrameId, m_baseFrameId));
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
		
	//! Key frame thresholds
    double m_keyFrameDist, m_keyFrameRot;
		
	//! Computed odomtery up to now
	double m_tx, m_ty, m_ta;

	//! Transform into current key-frame
	double m_keyFrameInit;
	double m_kx, m_ky, m_ka; 
		
	//! Node parameters
	std::string m_inScanTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	
	//! ROS msgs and data
	ros::NodeHandle m_nh;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
    ros::Subscriber m_scanSub;
	
	//! 2D distance grid
    TDF2D m_grid2d;
	double m_tdfGridSize, m_tdfGridRes;
		
	//! Non-linear optimization solver
	DLL2DSolver m_solver;
	int m_solverMaxIter, m_solverMaxThreads;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dlo2d_node");  
	
	// Particle filter instance
	std::string node_name = "dlo2d_node";
	DLO2DNode node(node_name);
	
	// Process data at given rate
	ros::spin();

	return 0;
}




