#ifndef __TDF2D_HPP__
#define __TDF2D_HPP__

#include <algorithm>  
#include <bitset>
#include <dll2d/df2d.hpp>

#define TDF2D_MAX(a,b) ((a)>(b)?(a):(b)) 

class TDF2D : public DF2D
{
	
public:

	TDF2D(void) 
	{
		m_gridMask = NULL;
	}

	~TDF2D(void)
	{
		if(m_gridMask != NULL)
			free(m_gridMask);
		m_gridMask = NULL;
	}

	void setup(float minX, float maxX, float minY, float maxY, float resolution)
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
		if(m_gridMask != NULL)
			free(m_gridMask);
		
		// Memory allocation for the grid
		m_oneDivRes = 1.0/m_resolution;
		m_gridSizeX = fabs(m_maxX-m_minX)*m_oneDivRes;
		m_gridSizeY = fabs(m_maxY-m_minY)*m_oneDivRes;
		m_gridSize = m_gridSizeX*m_gridSizeY;
		m_gridBilinear = (BilinearParams *)malloc(m_gridSize*sizeof(BilinearParams));
		m_gridDist = (float *)malloc(m_gridSize*sizeof(float));
		m_gridMask = (uint32_t *)malloc(m_gridSize*sizeof(uint32_t));

		// Clear buffers
		clear();
	}

	void clear(void)
	{
		std::memset(m_gridBilinear, 0, m_gridSize*sizeof(BilinearParams));
		std::memset(m_gridDist, 0, m_gridSize*sizeof(float));
		std::memset(m_gridMask, -1, m_gridSize*sizeof(uint32_t));
	}

	void loadCloud(std::vector<Point2D> &cloud)
	{
		// Creates the manhatan distance mask kernel
		// The point is centered into 3D kernel
		int k = 0;
		uint32_t kernel[33*33];
		for(int y=-16; y<=16; y++)
			for(int x=-16; x<=16; x++)
				kernel[k++] = ((uint32_t)0xffffffff) >> TDF2D_MAX(31 - abs(x) - abs(y), 0);
 
		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = 16*m_resolution;
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step) || !isIntoGrid(cloud[i].x+step, cloud[i].y+step))
				continue;

			int xi, yi, k;
			uint64_t idx, idy = pointToGrid(cloud[i].x-step, cloud[i].y-step);
			for(yi=0, k=0; yi<33; yi++, idy+=m_gridSizeX)
				for(xi=0, idx=idy; xi<33; xi++)
					m_gridMask[idx++] &= kernel[k++];  
		}

		// Computes manhatan distance based on the mask value
		for(uint64_t i=0; i<m_gridSize; i++)
			m_gridDist[i] = std::bitset<32>(m_gridMask[i]).count();			

		// Computes bilinear interpolation parameters
		computeBilinearInterpolation();
	}

protected:

	uint32_t *m_gridMask;				// Binary mask for Manhatan distance

};	


#endif
