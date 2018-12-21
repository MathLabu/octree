#include <cstdlib>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>

#include "octree.h"
#include "visitors.h"

int main(int argc, char* argv[])
{		
	constexpr std::size_t dim = 3;

	using T = float;
	using DP = typename Octree_<T,dim>::DP;
	
	assert(argc>=3);
	
  const int nbPts = std::atoi(argv[1]);
     
	DP pts = DP::Random(dim, nbPts);
	
	std::cout << "==== Start of execution ====" << std::endl;
	auto t_start = std::chrono::high_resolution_clock::now();
	Octree_<T,dim> oc{};
	
	switch(atoi(argv[2]))
	{
		case 0: //max number of data
			oc.build(pts, size_t(atoi(argv[3])), 0., true);
			break;
		case 1: //max size of node
			oc.build(pts, 1, T(atof(argv[3])), true);
			break;
	}
	
	auto t_end = std::chrono::high_resolution_clock::now();
	std::cout << "==== Building take ("<< std::chrono::duration<double>(t_end-t_start).count() <<" s) ====" << std::endl;
	
	t_start = std::chrono::high_resolution_clock::now();
	Octree<T> oc2{oc};
	t_end = std::chrono::high_resolution_clock::now();
	std::cout << "==== Copying take ("<< std::chrono::duration<double>(t_end-t_start).count() <<" s) ====" << std::endl;
	
	t_start = std::chrono::high_resolution_clock::now();
	Octree<T> oc3{std::move(oc2)};
	t_end = std::chrono::high_resolution_clock::now();
	std::cout << "==== Moving take ("<< std::chrono::duration<double>(t_end-t_start).count() <<" s) ====" << std::endl;
	
	//-----------------------------------------
	ColorPrinter printer(pts);
	
	t_start = std::chrono::high_resolution_clock::now();
	oc.visit(printer);
	t_end = std::chrono::high_resolution_clock::now();
	std::cout << "==== Visiting (Color) take ("<< std::chrono::duration<double>(t_end-t_start).count() <<" s) ====" << std::endl;
	
	
	std::cout << "==== End of execution ====" << std::endl;
	return 0;
}
