#ifndef __DF2D_HPP__
#define __DF2D_HPP__

#include <stdlib.h>
#include <vector>
#include <cstring>
#include <ANN/ANN.h>
#include <nav_msgs/OccupancyGrid.h>

struct Point2D
{
	float x;
	float y;

	Point2D(void)
	{
		x = 0;
		y = 0;
	}

	Point2D(const Point2D &d)
	{
		x = d.x;
		y = d.y;
	}
};

struct BilinearParams
{
	float a0, a1, a2, a3;

	BilinearParams(void)
	{
		a0 = a1 = a2 = a3 = 0.0;
	}

	BilinearParams(const BilinearParams &d)
	{
		a0 = d.a0;
		a1 = d.a1;
		a2 = d.a2;
		a3 = d.a3;
	}

	float interpolate(float x, float y)
	{
		return a0 + a1*x + a2*y + a3*x*y;
	}
};

class DF2D
{

public:

	DF2D(void) 
	{
	 	m_maxX = 50;
		m_maxY = 50;
		m_minX = -50;
		m_minY = -50;
		m_resolution = 0.05;
		m_gridDist = NULL;
		m_gridBilinear = NULL;
	}

	~DF2D(void)
	{
		if(m_gridBilinear != NULL)
			free(m_gridBilinear);
		if(m_gridDist != NULL)
			free(m_gridDist);
		m_gridDist = NULL;
		m_gridBilinear = NULL;
	}

	void virtual setup(float minX, float maxX, float minY, float maxY, float resolution)
	{
		m_maxX = maxX;
		m_maxY = maxY;
		m_minX = minX;
		m_minY = minY;
		m_resolution = resolution;

		// Free memory if needed
		if(m_gridBilinear != NULL)
			free(m_gridBilinear);
		if(m_gridDist != NULL)
			free(m_gridDist);
		
		// Memory allocation for the grid
		m_oneDivRes = 1.0/m_resolution;
		m_gridSizeX = fabs(m_maxX-m_minX)*m_oneDivRes;
		m_gridSizeY = fabs(m_maxY-m_minY)*m_oneDivRes;
		m_gridSize = m_gridSizeX*m_gridSizeY;
		m_gridBilinear = (BilinearParams *)malloc(m_gridSize*sizeof(BilinearParams));
		m_gridDist = (float *)malloc(m_gridSize*sizeof(float));
	}

	void virtual clear(void)
	{
		std::memset(m_gridBilinear, 0, m_gridSize*sizeof(BilinearParams));
		std::memset(m_gridDist, 0, m_gridSize*sizeof(float));
	}

	void virtual loadCloud(std::vector<Point2D> &cloud)
	{
		// Build the object positions array
		ANNpointArray points = annAllocPts(cloud.size(), 2);
		for(int i=0; i<cloud.size(); i++)
		{
			points[i][0] = cloud[i].x;
			points[i][1] = cloud[i].y;
		}

		// Build KDtree
		ANNkd_tree* kdTree = new ANNkd_tree(points, cloud.size(), 2);

		// Evaluate distance from all grid-cell to map-points  
		int i, j, k = 0;
		float x, y;
		ANNpoint queryPt = annAllocPt(2);
		ANNidxArray nnIdx = new ANNidx[1];            
		ANNdistArray dists = new ANNdist[1];
		for(i=0, y=m_minY; i<m_gridSizeY; i++, y+=m_resolution)
		{
			for(j=0, x=m_minX; j<m_gridSizeX; j++, x+=m_resolution, k++)
			{
				queryPt[0] = x;
				queryPt[1] = y;
				kdTree->annkSearch(queryPt, 1, nnIdx, dists);
				m_gridDist[k] = dists[0];
			}
		}
		delete kdTree;

		// Computes bilinear interpolation parameters
		computeBilinearInterpolation();
	}

	void loadCloud(std::vector<Point2D> &cloud, float tx, float ty, float yaw)
	{
		std::vector<Point2D> out;

		float c = cos(yaw);
		float s = sin(yaw);
		out.resize(cloud.size());
		for(uint32_t i=0; i<out.size(); i++)
		{
			out[i].x = c*cloud[i].x - s*cloud[i].y + tx;
			out[i].y = s*cloud[i].x + c*cloud[i].y + ty; 
		}
		loadCloud(out);
	}

	void loadGrid(const nav_msgs::OccupancyGrid &occGrid, float probThrehold = 0.5)
	{
		std::vector<Point2D> out;

		// Setup TDF grid parameters
		setup(occGrid.info.origin.position.x, occGrid.info.origin.position.x + occGrid.info.width*occGrid.info.resolution, 
		      occGrid.info.origin.position.y, occGrid.info.origin.position.y + occGrid.info.height*occGrid.info.resolution,
			  occGrid.info.resolution);

		// Compute cloud from grid
		Point2D p;
		uint32_t i, j, k;
		for(i=0, k=0; i < occGrid.info.height; i++)
			for(j=0; j < occGrid.info.width; j++, k++)
				if(occGrid.data[k] > probThrehold)
				{
					p.x = occGrid.info.origin.position.x + 	j*occGrid.info.resolution;
					p.y = occGrid.info.origin.position.y + 	i*occGrid.info.resolution;
					out.push_back(p);
				}

		loadCloud(out);
	}
	
	inline bool isIntoGrid(const float &x, const float &y)
	{
		return (x > m_minX && y > m_minY && x < m_maxX && y < m_maxY);
	}

	inline bool isIntoGrid(const uint64_t &index)
	{
		return (index >= 0 && index < m_gridSize);
	}

	double getDist(double x, double y)
	{
		float r = 0.0;
		if(isIntoGrid(x, y))
			r = m_gridDist[pointToGrid(x, y)];
		return r;
	}

	BilinearParams getDistInterpolation(double x, double y)
	{
		BilinearParams r;
		if(isIntoGrid(x, y))
			r = m_gridBilinear[pointToGrid(x, y)];
		return r;
	}

	void buildGridMsg(nav_msgs::OccupancyGrid &gridMsg)
	{
		static int seq = 0;
		
		// Setup grid msg
		gridMsg.header.stamp = ros::Time::now();
		gridMsg.header.seq = seq++;
		gridMsg.info.map_load_time = ros::Time::now();
		gridMsg.info.resolution = m_resolution;
		gridMsg.info.width = m_gridSizeX;
		gridMsg.info.height = m_gridSizeY;
		gridMsg.info.origin.position.x = m_minX;
		gridMsg.info.origin.position.y = m_minY;
		gridMsg.info.origin.position.z = 0.0;
		gridMsg.info.origin.orientation.x = 0.0;
		gridMsg.info.origin.orientation.y = 0.0;
		gridMsg.info.origin.orientation.z = 0.0;
		gridMsg.info.origin.orientation.w = 1.0;
		gridMsg.data.resize(m_gridSize);

		// Get max value into the grid
		float max = -1;
		for(int i=0; i<m_gridSize; i++)
			if(m_gridDist[i] > max)
				max = m_gridDist[i];

		// Copy data into grid msg and scale the probability to [0,100]
		float maxProb = 100/max;
		for(int i=0; i<m_gridSize; i++)
			gridMsg.data[i] = (int8_t)( 100 - m_gridDist[i]*maxProb );
	}

protected:

	bool computeBilinearInterpolation(void)
	{
		int ix, iy;
		float x1, y1, x2, y2;
		float div = 1.0/(m_resolution*m_resolution);
		float *p11, *p12, *p21, *p22;
		BilinearParams *pParams;

		for(iy=0, y1=m_minY, y2=m_minY+m_resolution; iy<m_gridSizeY-1; iy++, y1+=m_resolution, y2+=m_resolution)
		{
			p11 = m_gridDist+m_gridSizeX*iy;
			p12 = p11 + m_gridSizeX;
			p21 = p11 + 1;
			p22 = p12 + 1;
			pParams = m_gridBilinear+m_gridSizeX*iy;

			for(ix=0, x1=m_minX, x2=m_minX+m_resolution; ix<m_gridSizeX-1; ix++, x1+=m_resolution, x2+=m_resolution)
			{
				BilinearParams p;
				float f11 = *(p11++), f12 = *(p12++), f21 = *(p21++), f22 = *(p22++); 
				
				p.a0 = (+f11*x2*y2 - f12*x2*y1 - f21*x1*y2 + f22*x1*y1)*div;
				p.a1 = (-f11*y2    + f12*y1    + f21*y2    - f22*y1   )*div;
				p.a2 = (-f11*x2    + f12*x2    + f21*x1    - f22*x1   )*div;
				p.a3 = (+f11       - f12       - f21       + f22      )*div;
				
				*(pParams++) = p;
			}
		}

		return true;
	}
	
	inline uint64_t pointToGrid(const float &x, const float &y)
	{
		return (uint64_t)((x-m_minX)*m_oneDivRes) + (uint64_t)((y-m_minY)*m_oneDivRes)*m_gridSizeX;
	}

	// 2D grid information
	BilinearParams *m_gridBilinear;		// Bilinear parameters interpolation
	float *m_gridDist;					// Manhatan distance grid
	uint64_t m_gridSize; 
	uint32_t m_gridSizeX, m_gridSizeY;
	float m_maxDist;					// Maximum distance into the grid

	// Grid parameters
	float m_maxX, m_maxY;
	float m_minX, m_minY;
	float m_resolution, m_oneDivRes;	
};	


#endif
