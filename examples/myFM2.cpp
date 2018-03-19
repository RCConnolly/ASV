/* Runs different versions of FM2 and FM2* over grid map generated from a text file. */

#include <iostream>
#include <array>
#include <string>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>
#include <fast_methods/console/console.h>

#include <fast_methods/fm/fmm.hpp>
#include <fast_methods/fm2/fm2.hpp>

#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridplotter.hpp>
#include <fast_methods/io/gridwriter.hpp>

using std::string;
using namespace std::chrono;

// Establish Grid & Path Dimensionality to 2
constexpr unsigned int nDims = 2;
typedef nDGridMap<FMCell, nDims> FMGrid2D;
typedef typename std::vector< std::array<double, nDims> > Path2D;

std::array<unsigned int, nDims> start_coord = {150, 330};
std::array<unsigned int, nDims>  goal_coord = {720, 220};

void computePath(Solver<FMGrid2D> * s, Path2D * p, std::vector<double> * p_vel, double step = 1) {
    Path2D * path = p;
    GradientDescent<FMGrid2D> grad;
    unsigned int idx;
    s->getGrid()->coord2idx(goal_coord, idx);
    grad.apply(*s->getGrid(),idx, *path, *p_vel, step);
}

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

  
    /////////////////////////
    /// Compute Safety Map //
    /////////////////////////

    // Create safety solver
    Solver<FMGrid2D> * ss = new FMM<FMGrid2D>("FMM_Safe");
    FMGrid2D safe_grid;
    MapLoader::loadMapFromImg(filename.c_str(), safe_grid); //sets occupancy

    // establishes grid type as dist_grid and "cleans" each cell in grid
    // "cleaning" sets default value of -1 for each cell, does not change its occupancy
    ss->setEnvironment(&safe_grid);

    // sets starting points as each occupied cell in grid
    std::vector<unsigned int> obs_indices;
    safe_grid.getOccupiedCells(obs_indices);
    ss->setInitialPoints(obs_indices);
    
    // computes the safety map
    ss->compute();


    // plot safety map
    //GridPlotter::plotArrivalTimes(safe_grid, "Safety Grid");

    ///////////////////////////
    /// Compute Distance Map //
    ///////////////////////////

    // Create distance solver
    Solver<FMGrid2D> * sd = new FMM<FMGrid2D>("FMM_Dist");
    FMGrid2D dist_grid;
    MapLoader::loadMapFromImg(filename.c_str(), dist_grid);
    // establishes grid type as dist_grid and "cleans" each cell in grid
    // "cleaning" sets default value of -1 for each cell, does not change its occupancy
    sd->setEnvironment(&dist_grid);

    // sets starting and end points given their coords on grid
    sd->setInitialAndGoalPoints(start_coord, goal_coord);
    //sd->setInitialPoints(start_coord);

    // computes the distances map
    sd->compute();

    std::cout << "Initial Distance Goal Pt\n";
    unsigned int goal_idx;
    dist_grid.coord2idx(goal_coord, goal_idx);
    std::cout << dist_grid[goal_idx];

    // plot distance map
    //GridPlotter::plotArrivalTimes(dist_grid, "Distance Grid");

    ////////////////////////////////////
    // Adjust Distance Map by weights //
    ////////////////////////////////////

    for (unsigned int i = 0; i < dist_grid.size(); i++) {
        if(safe_grid[i].getVelocity() != 0) {     
            dist_grid[i].setArrivalTime(((1-safe_grid[i].getArrivalTime()) + dist_grid[i].getArrivalTime())/2.0);
        }    
    }

    std::cout << "Weighted Distance Goal Pt\n";
    std::cout << dist_grid[goal_idx];

    // plot weighted map
    GridPlotter::plotArrivalTimes(dist_grid, "Weighted Grid");

    ////////////////////
    // Calculate Path //
    ////////////////////
    Path2D path;
    std::vector<double> path_vels;
    //computePath(ss, &path, &path_vels);
    //GridPlotter::plotArrivalTimesPath(dist_grid, path);

    /*
    // print grid size
    std::array<unsigned int, nDims> dims = dist_grid.getDimSizes();
    std::cout << "Grid dimension: " << dims[0] << " x " << dims[1] << "\n";
    std::cout << "leaf size: " << dist_grid.getLeafSize() << "\n";
    */

    //std::cout << "\tElapsed "<< sd->getName() <<" time: " << sd->getTime() << " ms" << '\n';

    
    std::cout << "Path coordinates:\n";
    std::cout << "Path Values:\n";    
    for (int i = 0; i < path_vels.size(); i++) {
        std::cout << "{" << path[i][0] << ", " << path[i][1] << "}, ";
        //std::cout << "[" << path_vels[i] << "], ";    
    }
    std::cout << "\n";
    

    // Preventing memory leaks.
    delete sd;
    delete ss;

    return 0;
}
