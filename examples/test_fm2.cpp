/* Runs different versions of FM2 and FM2* over grid map generated from a text file. */

#include <iostream>
#include <array>
#include <string>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>
#include <fast_methods/console/console.h>

#include <fast_methods/fm2/fm2.hpp>

#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridplotter.hpp>
#include <fast_methods/io/gridwriter.hpp>

using std::string;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    console::info("Parsing input arguments.");
    string filename;
    if (argc > 2)
        console::parseArguments(argc, argv, "-map", filename);
    else {
        console::info("No enough arguments given. Use as ./test_fm2 -map path_to_file.txt");
        exit(1);
    }

    // Establish Grid & Path Dimensionality to 2
    constexpr unsigned int ndims2 = 2;
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef typename std::vector< std::array<double, ndims2> > Path2D;

    // Loading grid.
    FMGrid2D grid_fm2;

    // Solver declaration.
    Solver<FMGrid2D> * s = new FM2<FMGrid2D>("FM2_Dary");
   
    // For FM2 and its variations, it is better to completely reinitialize the grid.
    // sets occupancy of each cell in grid based on image
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2); // Loading from image.

    // print grid size
    std::array<unsigned int, ndims2> dims = grid_fm2.getDimSizes();
    std::cout << "Grid dimension: " << dims[0] << " x " << dims[1] << "\n";
    std::cout << "leaf size: " << grid_fm2.getLeafSize() << "\n";

    // establishes grid type as grid_fm2 and "cleans" each cell in grid
    // "cleaning" sets default value of -1 for each cell, does not change its occupancy
    s->setEnvironment(&grid_fm2);

    // sets starting and end points given their coords on grid
    s->setInitialAndGoalPoints({150, 330}, {720, 220}); // Init and goal points directly set.
    
    // computes the distances map by calling computeInternal
    // compute internal implements actual FM2 method
    // FM2 method computes velocities map then calls FMM's compute
    s->compute();

    std::cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

    Path2D path;
    std::vector<double> path_vels;
    // determines the path & path velocities based on the minimum of the times of
    // the arrival map
    s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
    
    GridPlotter::plotArrivalTimesPath(grid_fm2, path);

    /*
    //std::cout << "Path coordinates:\n";
    std::cout << "Path Values:\n";    
    for (int i = 0; i < path_vels.size(); i++) {
        //std::cout << "{" << path[i][0] << ", " << path[i][1] << "}, ";
        std::cout << "[" << path_vels[i] << "], ";    
    }
    std::cout << "\n";
    */

    // Preventing memory leaks.
    delete s;

    return 0;
}
