#pragma once

#include <cstdlib> 
#include <vector>

#include "Eigen/Core"

#include "utils.h"

template < typename T, std::size_t dim = 3>
class Octree_ 
{
public:	
	using DP = typename Eigen::Matrix<T,dim,Eigen::Dynamic>; /*add +1 if homegeneous coordinates*/
	using Index =  std::size_t;
	
	using Data = Index; /*point's index*/
	using DataContainer = std::vector<Data>;
	
	using Point = Eigen::Matrix<T,dim,1>;
	
//private:
	static constexpr std::size_t nbCells = utils::pow(2, dim);
	
private:
	struct BoundingBox 
	{
			Point center;
			T 	radius;
	};
	
	Octree_* parent;
	Octree_* cells[nbCells];
	
	/******************************************************
	 *	Cells id are assigned as their position 
	 *   from the center (+ greater than center, - lower than center)
	 *
	 *		for 3D case									for 2D case
	 *
	 *	  	0	1	2	3	4	5	6	7		  	0	1	2	3
	 * 	x:	-	+	-	+	-	+	-	+		x:	-	+	-	+
	 * 	y:	-	-	+	+	-	-	+	+		y:	-	-	+	+	
	 * 	z:	-	-	-	-	+	+	+	+
	 *
	 *****************************************************/
	
	BoundingBox bb;
	
	DataContainer data;	
	
	std::size_t depth;
		
public:
	Octree_();
	Octree_(const Octree_<T,dim>& o); //Deep-copy
	Octree_(Octree_<T,dim>&& o);
	
	virtual ~Octree_();
	
	Octree_<T,dim>& operator=(const Octree_<T,dim>& o);//Deep-copy
	Octree_<T,dim>& operator=(Octree_<T,dim>&& o);
	
	bool isLeaf() const;
	bool isRoot() const;
	bool isEmpty()const;
	
	inline std::size_t idx(const Point& pt) const;
	inline std::size_t idx(const DP& pts, const Data d) const;
	
	std::size_t getDepth() const;
	
	T getRadius() const;
	Point getCenter() const;
	
	DataContainer * getData();
	Octree_<T, dim>* operator[](std::size_t idx);
	
	// Build tree from DataPoints with a specified stop parameter
	bool build(const DP& pts, size_t maxDataByNode=1, T maxSizeByNode=T(0.), bool parallelBuild=false);

protected:
	//real build function
	bool build(const DP& pts, DataContainer&& datas, BoundingBox && bb, size_t maxDataByNode=1, T maxSizeByNode=T(0.), bool parallelBuild=false);
	
	inline DataContainer toData(const DP& pts, const std::vector<Index>& ids);
	
public:	
	template < typename Callback >
	bool visit(Callback& cb);
};
	
#include "octree.hpp"

template<typename T> using Quadtree = Octree_<T,2>;
template<typename T> using Octree = Octree_<T,3>;



