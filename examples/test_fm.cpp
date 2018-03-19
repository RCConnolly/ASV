/* Runs different versions of FMM over an empty, generated grid. */

#include <iostream>
#include <array>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>

#include <fast_methods/fm/fmm.hpp>
#include <fast_methods/fm2/fm2.hpp>

#include <fast_methods/io/gridplotter.hpp>

using std::string;
using namespace std::chrono;

int main()
{
    // A bit of shorthand.
    typedef nDGridMap<FMCell, 2> FMGrid2D;
    typedef std::array<unsigned int, 2> Coord2D;
    typedef typename std::vector< std::array<double, 2> > Path2D;


    // Grid, start and goal definition.
    Coord2D dimsize {300,300};
    FMGrid2D grid_fmm (dimsize);
    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};

    // Solvers declaration.
    Solver<FMGrid2D> * s = new FMM<FMGrid2D>("FMM");

    s->setEnvironment(&grid_fmm);
    s->setInitialPoints(init_point); // If no goal_idx is set. Comment next line if this one is uncommented.
    //s->setInitialAndGoalPoints(init_point, goal_point);
    s->compute();
    std::cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';
    GridPlotter::plotArrivalTimes(grid_fmm, s->getName());
    
    Path2D path;
    std::vector<double> path_vels;
    s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
    GridPlotter::plotArrivalTimesPath(grid_fmm, path);

    // Preventing memory leaks.
    delete s;

    return 0;
}
