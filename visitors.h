#pragma once

#include "octree.h"

using T = float;
using DP = typename Octree<T>::DP;

struct ColorPrinter {
	std::ofstream ofs;
	std::size_t color;
	
	const DP&	pts;
	
	ColorPrinter(const DP& dp) : color{0}, pts{dp}
	{
		std::ostringstream oss;
		oss << "colored-pc-" << getpid() << ".csv";
		ofs.open(oss.str());
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file .csv"));
		
		ofs<<"x,y,z,color\n";
	}
	
	~ColorPrinter()
	{
		ofs.close();
	}
	
	bool operator()(Octree<float>& oc)
	{
		if(oc.isLeaf() and not oc.isEmpty())
		{			
			auto* data = oc.getData();
			
			for(auto& d : *data)
			{
				ofs << pts(0,d) << ","
				    << pts(1,d) << ","
				    << pts(2,d) << ","
				    << color << "\n";
			}
			++color;
		}
		
		return true;
	}
};
