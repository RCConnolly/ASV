/* Runs different versions of FM2 and FM2* over grid map generated from a text file. */

#include <iostream>
#include <array>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>
#include <fast_methods/console/console.h>

#include <fast_methods/fm2/fm2.hpp>
#include <fast_methods/fm2/fm2star.hpp>
#include <fast_methods/datastructures/fmfibheap.hpp>
#include <fast_methods/datastructures/fmpriorityqueue.hpp>

#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridplotter.hpp>
#include <fast_methods/io/gridwriter.hpp>

using namespace std;
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

    // A bit of shorthand.
    constexpr unsigned int ndims2 = 2; // Setting two dimensions.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef typename std::vector< std::array<double, ndims2> > Path2D; // A bit of short-hand.

    // Loading grid.
    FMGrid2D grid_fm2;

    // Solvers declaration.
    Solver<FMGrid2D> * s = new FM2<FMGrid2D>("FM2_Dary");
   
    // Executing every solver individually over the same grid.
    // For FM2 and its variations, it is better to completely reinitialize the grid.
    //if(!MapLoader::loadMapFromText(filename.c_str(), grid_fm2)) // Loading from text file.
    //exit(1);
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2); // Loading from image.
    s->setEnvironment(&grid_fm2);
    s->setInitialAndGoalPoints({30, 20}, {375, 280}); // Init and goal points directly set.
    s->compute();
    cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

    Path2D path;
    vector<double> path_vels;
    s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
    GridPlotter::plotArrivalTimesPath(grid_fm2, path);

    // Preventing memory leaks.
    delete s;

    return 0;
}
