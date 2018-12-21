#include <iostream>
#include <iterator>
#include <future>

#define DEBUG 0

#if defined(DEBUG) && DEBUG
	#define DBG_PRINT(...) (std::cerr <<  __VA_ARGS__ << std::endl)
#else
	#define DBG_PRINT(...)
#endif

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(): parent{nullptr},	depth{0}
{
	for(size_t i=0; i< nbCells; ++i) cells[i]=nullptr;
	DBG_PRINT("Building empty octree...");
}

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(const Octree_<T,dim>& o): 
	bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	DBG_PRINT("Copying octree ("<<depth<<")...");
	
	if (!o.parent) 
		parent = nullptr;	
		
	if(o.isLeaf()) //Leaf case
	{
		//nullify childs
		for(size_t i=0; i<nbCells; ++i)
			cells[i]= nullptr;
		//Copy data
		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else //Node case
	{
		//Create each child recursively
		for(size_t i=0; i<nbCells;++i)
		{
			DBG_PRINT("("<<i<<")");
			cells[i] = new Octree_<T,dim>(*(o.cells[i]));	
			//Assign parent  	
			cells[i]->parent = this;
		}	
	}
}
template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(Octree_<T,dim>&& o): 
	parent{nullptr}, bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	//only allow move of root node
	assert(o.isRoot());
	
	DBG_PRINT("Moving octree...");
	
	if(o.isLeaf()) //Leaf case
	{
		//Copy data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	//copy child ptr
	for(size_t i=0; i<nbCells; ++i)
	{
		cells[i] = o.cells[i];
		//Nullify ptrs
		o.cells[i]=nullptr;
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>::~Octree_()
{
	DBG_PRINT("Deleting octree ("<<depth<<")...");
	
	//delete recursively childs
	if(!isLeaf())
		for(size_t i=0; i<nbCells; ++i)
		{
			DBG_PRINT("-- ("<<depth<<","<<i<<")." );
			delete cells[i];
		}
		
	DBG_PRINT("Deleted ("<<depth<<").");
}

template<typename T, std::size_t dim>	
Octree_<T,dim>& Octree_<T,dim>::operator=(const Octree_<T,dim>&o)
{
	DBG_PRINT("Copying/affecting octree ("<<depth<<")...");
	
	if (!o.parent) 
		parent = nullptr;
		
	depth=o.depth;
	
	if(o.isLeaf()) //Leaf case
	{
		//nullify childs
		for(size_t i=0; i<nbCells; ++i)
			cells[i]= nullptr;
  		//Copy data
  		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else //Node case
	{
		//Create each child recursively
  		for(size_t i=0; i<nbCells; ++i)
  		{
  			DBG_PRINT("("<<i<<")");
  			cells[i] = new Octree_<T,dim>(*(o.cells[i]));	
			//Assign parent  	
  			cells[i]->parent = this;
  		}	
	}
	return *this;
}

template<typename T, std::size_t dim>	
Octree_<T,dim>& Octree_<T,dim>::operator=(Octree_<T,dim>&&o)
{
	//only allow move of root node
	assert(o.isRoot());
	
	DBG_PRINT("Moving/affecting octree...");
	
	parent = nullptr;
	bb.center = o.bb.center;
	bb.radius = o.bb.radius;
	
	depth = o.depth;
	
	if(o.isLeaf()) //Leaf case
	{
		//Copy data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	//copy childs ptrs
	for(size_t i=0; i<nbCells; ++i)
	{
		cells[i] = o.cells[i];
		//Nullify ptrs
		o.cells[i]=nullptr;
	}
	
	return *this;
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::isLeaf() const
{
	return (cells[0]==nullptr);
}
template<typename T, std::size_t dim>
bool Octree_<T,dim>::isRoot() const
{
	return (parent==nullptr);
}
template<typename T, std::size_t dim>
bool Octree_<T,dim>::isEmpty() const
{
	return (data.size() == 0);
}
template<typename T, std::size_t dim>
size_t Octree_<T,dim>::idx(const Point& pt) const
{
	size_t id = 0;
	for(size_t i=0; i<dim; ++i)
		id|= ((pt(i) > bb.center(i)) << i);

	return id;
}

template<typename T, std::size_t dim>
size_t Octree_<T,dim>::idx(const DP& pts, const Data d) const
{
	return idx(pts.col(d).head(dim));
}
template<typename T, std::size_t dim>
size_t Octree_<T,dim>::getDepth() const
{
	return depth;
}
template<typename T, std::size_t dim>
T Octree_<T,dim>::getRadius() const
{
	return bb.radius;
}
template<typename T, std::size_t dim>
typename Octree_<T,dim>::Point Octree_<T,dim>::getCenter() const
{
	return bb.center;
}
template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer * Octree_<T,dim>::getData()
{
	return &data;
}
template<typename T, std::size_t dim>
Octree_<T,dim>* Octree_<T,dim>::operator[](size_t idx)
{
	assert(idx<nbCells);
	return cells[idx];
}

template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer Octree_<T,dim>::toData(const DP& pts, const std::vector<Index>& ids)
{
	return DataContainer{ids.begin(), ids.end()};
}

// Build tree from DataPoints with a specified number of points by node
template<typename T, std::size_t dim>
bool Octree_<T,dim>::build(const DP& pts, size_t maxDataByNode, T maxSizeByNode, bool parallelBuild)
{
	DBG_PRINT("Building octree from DataPoints...");
	
	assert(pts.cols()>0);
	
	//Build bounding box
	DBG_PRINT("Building BoundingBox...");
	BoundingBox box;
	
	auto minValues = pts.rowwise().minCoeff();
	auto maxValues = pts.rowwise().maxCoeff();
	
	Point min = minValues.head(dim);
	Point max = maxValues.head(dim);
	
	Point radii = max - min;
	box.center = min + radii * 0.5;
	
	box.radius = radii(0);
	for(size_t i=1; i<dim; ++i)
		if (box.radius < radii(i)) box.radius = radii(i);
	
	box.radius*=0.5;
	
	DBG_PRINT("BoundingBox: radius="<<box.radius
	        <<", center=("<<box.center(0)<<","<<box.center(1)<<","<<box.center(2)<<").");
	
	//Transform pts in data	
	const size_t nbpts = pts.cols();
	std::vector<Index> indexes;
	indexes.reserve(nbpts);
		
	for(size_t i=0; i<nbpts; ++i)
		indexes.emplace_back(Index(i));
	
	DBG_PRINT("Building Datas...");
	DataContainer datas = toData(pts, indexes);
	
	//build
	return this->build(pts, std::move(datas), std::move(box), maxDataByNode, maxSizeByNode, parallelBuild);
}

//Offset lookup table
template<typename T, std::size_t dim>
struct OctreeHelper;

template<typename T>
struct OctreeHelper<T,3>
{
	static const typename Octree_<T,3>::Point offsetTable[Octree_<T,3>::nbCells];
};
template<typename T>
const typename Octree_<T,3>::Point OctreeHelper<T,3>::offsetTable[Octree_<T,3>::nbCells] = 
		{
			{-0.5, -0.5, -0.5},
			{+0.5, -0.5, -0.5},
			{-0.5, +0.5, -0.5},
			{+0.5, +0.5, -0.5},
			{-0.5, -0.5, +0.5},
			{+0.5, -0.5, +0.5},
			{-0.5, +0.5, +0.5},
			{+0.5, +0.5, +0.5}
		};

template<typename T>
struct OctreeHelper<T,2>
{
	static const typename Octree_<T,2>::Point offsetTable[Octree_<T,2>::nbCells];
};
template<typename T>
const typename Octree_<T,2>::Point OctreeHelper<T,2>::offsetTable[Octree_<T,2>::nbCells] = 
		{
			{-0.5, -0.5},
			{+0.5, -0.5},
			{-0.5, +0.5},
			{+0.5, +0.5}
		};


template<typename T, std::size_t dim>
bool Octree_<T,dim>::build(const DP& pts, DataContainer&& datas, BoundingBox && bb, 
	size_t maxDataByNode, T maxSizeByNode, bool parallelBuild)
{
	DBG_PRINT("Building octree ("<<depth<<") recursively...");
	
	//Assign bounding box
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;
	
	//Check stop condition
	if((bb.radius*2.0 <= maxSizeByNode) or (datas.size() <= maxDataByNode))
	{
		DBG_PRINT("-- Leaf ("<<depth<<").");			
		//insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), make_move_iterator(datas.end()));
		return (isLeaf());
	}
	
	//Split datas
	DBG_PRINT("Spliting datas...");
	const std::size_t nbData = datas.size();
	
	DataContainer sDatas[nbCells];
	for(size_t i=0; i<nbCells; ++i)
		sDatas[i].reserve(nbData);
		
	for(auto&& d : datas)
		(sDatas[idx(pts, d)]).emplace_back(d);
	
	for(size_t i=0; i<nbCells; ++i)
		sDatas[i].shrink_to_fit();
	
	//Compute new bounding boxes
	DBG_PRINT("Computing bounding boxes...");
	BoundingBox boxes[nbCells];
	const T half_radius = this->bb.radius * 0.5;
	for(size_t i=0; i<nbCells; ++i)
	{
		const Point offset = OctreeHelper<T,dim>::offsetTable[i] * this->bb.radius;
		boxes[i].radius = half_radius;
		boxes[i].center = this->bb.center + offset;
	}
	
	//For each child build recursively
	DBG_PRINT("Building childs...");
	bool ret = true;

	std::vector<std::future<void>> futures;

	for(size_t i=0; i<nbCells; ++i)
	{
		DBG_PRINT("-- Child("<<i<<")");
		
		auto compute = [maxDataByNode, maxSizeByNode, i, &pts, &sDatas, &boxes, this](){
				this->cells[i] = new Octree_<T,dim>();
				//Assign depth
				this->cells[i]->depth = this->depth+1;
				//Assign parent
				this->cells[i]->parent = this;
				//next call is not parallelizable
				this->cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxDataByNode, maxSizeByNode, false);	
			};
		
		if(parallelBuild)
			futures.push_back( std::async( std::launch::async, compute ));
		else
			compute();
	}

	for(auto& f : futures) f.get();

	return (!isLeaf() and ret);
}

//------------------------------------------------------------------------------
template<typename T, std::size_t dim>
template<typename Callback>
bool Octree_<T,dim>::visit(Callback& cb)
{
	// Call the callback for this node (if the callback returns false, then
	// stop traversing.
	if (!cb(*this)) return false;

	// If I'm a node, recursively traverse my children
	if (!isLeaf())
		for (size_t i=0; i<nbCells; ++i)
			if (!cells[i]->visit(cb)) return false;

	return true;
}
