# Octree/Quadtree implementation for decomposing point cloud

The current implementation use the data structures of _Eigen::Matrix_. It ensures that each node have either (8/4) or no 0 childs. 

Can create an octree with the 2 following crieterions:
- max number of data by node (0)
- max size of a node (1) (or stop when only one or zero data element is available)

After construction, we can apply a process by creating a _Visitor_ implementing the following method:
```cpp
template<typename T, std::size_t dim>
struct Visitor {
	bool operator()(Octree_<T,dim>& oc);
};
```

**Remark:**
- Current implementation only store the _indexes_ of the points from the pointcloud.
- Data element are exclusively contained in _Leaves_ node. To know if the current octants is a _Leaf_ use the function `bool Octree_<T>::isLeaf() const`.
- Some _Leaf_ node contains no data (to ensure 8 or 0 childs), they're _Empty Node_. To know if the current octants is _empty_ use the function `bool Octree_<T>::isEmpty() const`.

## Parametrization

To enable parallel construction of the octree, please pass `true` as parameter of the build function. Only the first step of the build process is parallelized, creating 8 threads (one for each octants) due to the recursive construction.

Please, either give a number of points to test octree on synthetic data or give a point cloud file.

## Compile

```
mkdir build; cd build
cmake ..
make
```

## Execute

For a max number of data by node (`0`), use parameter of type `size_t` and for max size of a node (`1`), use a parameter of type `float` or `double`.

```
./build/octree nbPts type_criterion parameter_criterion
```
```
./build/octree 60000 0 25
```
## Results

The program implements a _Visitor_ (`ColorPrinter`) that color the point cloud according to the repartition in the octree, and create a csv file `colored-pc-<pid>.csv` that can be exploited.
