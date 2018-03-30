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

void computeDistGrid(FMGrid2D & g) {
    // Create distance solver
    Solver<FMGrid2D> * sd = new FMM<FMGrid2D>("FMM_Dist");

    // establishes grid type as dist_grid and "cleans" each cell in grid
    // "cleaning" sets default value of -1 for each cell, does not change its occupancy
    sd->setEnvironment(&g);

    // sets starting and end points given their coords on grid
    sd->setInitialAndGoalPoints({150, 330}, {720, 220}); // Init and goal points directly set.
    
    // computes the distances map
    sd->compute();
    
    delete sd;
}

void computeSafeGrid(FMGrid2D & g) {
    // Create safety solver
    Solver<FMGrid2D> * sd = new FMM<FMGrid2D>("FMM_Safe");

    // establishes grid type as dist_grid and "cleans" each cell in grid
    // "cleaning" sets default value of -1 for each cell, does not change its occupancy
    sd->setEnvironment(&g);

    // sets starting points as each occupied cell in grid
    std::vector<unsigned int> obs_indices;
    g.getOccupiedCells(obs_indices);
    sd->setInitialPoints(obs_indices);
    
    // computes the safety map
    sd->compute();
    
    delete sd;
}

/*
void computeWeightedGrid(FMGrid2D & g, double safe_w, double dist_w) {

}
*/

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

    FMGrid2D safe_grid;
    MapLoader::loadMapFromImg(filename.c_str(), safe_grid); //sets occupancy
    computeSafeGrid(safe_grid);

    // plot safety map
    //GridPlotter::plotArrivalTimes(safe_grid, "Safety Grid");

    ///////////////////////////
    /// Compute Distance Map //
    ///////////////////////////

    FMGrid2D dist_grid;
    MapLoader::loadMapFromImg(filename.c_str(), dist_grid);
    computeDistGrid(dist_grid);

    //////////////////////////
    // Compute Weighted Map //
    //////////////////////////
    FMGrid2D w_grid;
    MapLoader::loadMapFromImg(filename.c_str(), w_grid);
    for (unsigned int i = 0; i < safe_grid.size(); i++) {
        w_grid[i].setArrivalTime((safe_grid[i].getArrivalTime() + dist_grid[i].getArrivalTime())/2.0);
    }    

    // plot distance map
    //GridPlotter::plotArrivalTimes(dist_grid, "Distance Grid");

    // plot weighted map
    GridPlotter::plotArrivalTimes(w_grid, "Distance Grid");

    std::cout << "Max Safe Value: " << safe_grid.getMaxValue() << "\n";
    std::cout << "Max Safe Speed (Occ Value): " << safe_grid.getMaxSpeed() << "\n";
    std::cout << "Average Safety: " << safe_grid.getAvgSpeed() << "\n";

    std::cout << "Max Dist Value: " << dist_grid.getMaxValue() << "\n";
    std::cout << "Max Dist Speed (Occ Value): " << dist_grid.getMaxSpeed() << "\n";
    std::cout << "Average Dist: " << dist_grid.getAvgSpeed() << "\n";


    /*
    // print grid size
    std::array<unsigned int, nDims> dims = dist_grid.getDimSizes();
    std::cout << "Grid dimension: " << dims[0] << " x " << dims[1] << "\n";
    std::cout << "leaf size: " << dist_grid.getLeafSize() << "\n";
    */

    //std::cout << "\tElapsed "<< sd->getName() <<" time: " << sd->getTime() << " ms" << '\n';


    //Path2D path;
    //std::vector<double> path_vels;
    // determines the path & path velocities based on the minimum of the times of
    // the arrival map
    //s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
    
    //GridPlotter::plotArrivalTimesPath(dist_grid, path);

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
    //delete sd;

    return 0;
}
