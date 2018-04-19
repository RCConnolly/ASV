/* Main script for running wFM2 for ASV */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>
#include <string>
#include <cmath>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>
#include <fast_methods/console/console.h>

#include <fast_methods/fm2/fm2.hpp>

#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridplotter.hpp>
#include <fast_methods/io/gridwriter.hpp>

#include <GeographicLib/GeoCoords.hpp>

#include "gdal_priv.h"
#include "cpl_conv.h"

using namespace GeographicLib;

class Image {
    public:
    const double EASTING;
    const double NORTHING;
    const int WIDTH;
    const int HEIGHT;
    const int XRES;
    const int YRES;
    const int ZONE;
    const bool isNORTH;
    Image(double e, double n, int w, int h, int xres, int yres, int zone, bool is_n) :
            EASTING(e), NORTHING(n), WIDTH(w), HEIGHT(h), XRES(xres), YRES(yres), ZONE(zone), isNORTH(is_n) {}
};

// Establish Grid & Path Dimensionality to 2
constexpr unsigned int nDims = 2;
typedef nDGridMap<FMCell, nDims> FMGrid;
typedef typename std::array<double, nDims> Point;
typedef typename std::array<unsigned int, nDims> Coord;
typedef typename std::vector<Point> Path;
//typedef typename std::vector<GeoCoords> Path;

// Establish global constants
const int SAFE_DIST = 60; // meters
const int SAFE_CELL_DIST = ceil((2*(SAFE_DIST/10))/sqrt(2));
const char MAV_VERSION[] = "110";
const int  MAV_FRAME = 0; // 0 = MAV_FRAME_GlOBAL, WGS84 coordinate system https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
const int MAV_CMD = 16; // 16 = MAV_CMD_NAV_WAYPOINT, same message definition url as above
const double HOLD_TIME = 0; // in seconds
const double WP_RADIUS = 5; // in meters
const double PASS_BY_DIST = 0; // distance in meters that ASV should pass by the waypoint
const float ROTARY_WING_YAW = 0; // or NAN?
const double ALTITUDE = 0;
long unsigned int POINT_DIST = 2;


/* Converts point coordinates from UTM northing & easting in decimal degrees
 * to image (x,y) coordinates with the bottom left corner of the image being
 * (0,0) and the x & y axes increasing to the right & upward respectively.
 * Parameters:
 *      utm_pt = {northing, easting} coordinates of point
 * Returns: none
*/
Point utm2image(GeoCoords & utm_pt, Image & img) {
    
    double  x_img = (utm_pt.Easting() - img.EASTING)/img.XRES;
    // If y-resolution is negative, the image origin is in top left.
    // Image coordinates must be in bottom left.    
    double y_img;    
    if(img.YRES < 0) {
        std::cout << "Pt n: " << utm_pt.Northing() << "\n";
        std::cout << "img n: " << img.NORTHING << "\n";
        std::cout << "img height: " << img.HEIGHT << "\n";
        std::cout << "yres: " << img.YRES << "\n";
        y_img = (utm_pt.Northing() - (img.NORTHING - img.HEIGHT*(-img.YRES)))/(-img.YRES);
    }
    else {
        y_img = (utm_pt.Northing() - img.NORTHING)/img.YRES;
    }

    assert(x_img > 0);
    assert(y_img > 0);

    if(x_img >= img.WIDTH) {
        std::cout << "Converted image x-coordinate" << x_img;
        std::cout << "exceeds image width " << img.WIDTH << "\n";
        exit(EXIT_FAILURE);
    }

    if(y_img >= img.HEIGHT) {
        std::cout << "Converted image y-coordinate " << y_img;
        std::cout << "exceeds image height " << img.HEIGHT << "\n";
        exit(EXIT_FAILURE);
    }

    Point pt = {x_img, y_img};

    return pt;
}


/* Computes a 2D path connecting the start & goal points. Each point in the path is
 * is in image coordinates.
 * Paramerters:
 *      start = initial point in image coordinates
 *      goal = goal point in image coordinates
 *      filename = name of obstacle image for the path to be computed on
 *      step = number of image pixels between each point in the path
*/
void computePath(Path & path, const Point & start, const Point & goal, const std::string filename) {
    // Loading grid.
    FMGrid grid_fm2;

    // Solver declaration.
    Solver<FMGrid> * s = new FM2<FMGrid>("FM2_Dary", SAFE_CELL_DIST);
   
    // fills in occupancy grid    
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2);
    std::array<unsigned int, 2> dims = grid_fm2.getDimSizes();
    for(unsigned int i = 0; i < dims[0]; i++) {
        for(unsigned int j = 0; j < dims[1]; j++) {
            unsigned int idx;
            grid_fm2.coord2idx({i,j}, idx);
            if(grid_fm2.getCell(idx).getOccupancy() > 0) {
                std::cout << "Occupancy(" << idx << "): " << grid_fm2.getCell(idx).getOccupancy() << "\n";
                grid_fm2.getCell(idx).setOccupancy(1);
            }
        }
    }
    s->setEnvironment(&grid_fm2);

    std::vector<unsigned int> occ;
    grid_fm2.getOccupiedCells(occ);
    std::cout << "Num occupied cells: " << occ.size() << "\n";

    std::cout << "Total Cells: " << dims[0]*dims[1] << "\n";
    
    // computes velocities map 
    Coord start_coord = {(unsigned int) start[0], (unsigned int) start[1]};
    Coord goal_coord  = {(unsigned int)  goal[0],  (unsigned int) goal[1]};   
    std::cout << "goal coord: " << goal_coord[0] << ", " << goal_coord[1] << "\n";
    std::cout << "start coord: " << start_coord[0] <<  ", " << start_coord[1] << "\n";
    s->setInitialAndGoalPoints(start_coord, goal_coord);
    std::cout << "set initial and goal\n";    
    s->compute();

    std::vector<double> path_vels;
    s->as<FM2 <FMGrid> >()->computePath(&path, &path_vels);
    //GridPlotter::plotArrivalTimesPath(grid_fm2, path);

    // Preventing memory leaks.
    delete s;

    return;
}


// Prints each path point a file, with coordinates being in Lat/Long
void printUTMPath(Path & path, const char * fname, Image & img) {
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        for(size_t i = 0; i < path.size(); i++) {
            //convert to utm before printing
            ofs << "18n "; // zone
            ofs.precision(10);
            ofs << (((path[i][1])*img.YRES)+img.NORTHING) << " "; // northing
            ofs << (((path[i][0])*img.XRES)+img.EASTING) << "\n";  // easting
        }
    }
    else {
        std::cout << "Unable to open file\n";
        exit(EXIT_FAILURE);
    }
}

// Prints each path point a file, with coordinates being in Lat/Long
void printImgPath(Path & path, const char * fname) {
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        for(size_t i = 0; i < path.size(); i++) {
            ofs << "y: " << path[i][1] << ", ";
            ofs << "x: " << path[i][0] << "\n";
        }
    }
    else {
        std::cout << "Unable to open file\n";
        exit(EXIT_FAILURE);
    }
}


void printWaypoints(Path & path, const char * fname, Image & img) {
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        ofs << "QGC WPL " << MAV_VERSION << "\n";
        int idx = 0;
        bool current_wp = 1;
        bool continue_auto = 1;

        ofs.precision(10);
        GeoCoords curr_wp;
        for(size_t i = 0; i < path.size(); i++) {
            if(i%POINT_DIST == 0) {            
                ofs << idx << "\t";
                ofs << current_wp << "\t";
                ofs << MAV_FRAME << "\t";
                ofs << MAV_CMD << "\t";
                ofs << HOLD_TIME << "\t"; // PARAM 1
                ofs << WP_RADIUS << "\t"; // PARAM 2
                ofs << PASS_BY_DIST << "\t"; // PARAM 3
                ofs << ROTARY_WING_YAW << "\t"; // PARAM 4
                
                curr_wp.Reset(img.ZONE, img.isNORTH, (path[i][0]*img.XRES)+img.EASTING, (path[i][1]*img.YRES)+img.NORTHING);
                ofs << curr_wp.Latitude() << "\t";
                ofs << curr_wp.Longitude() << "\t";
                ofs << ALTITUDE << "\t";
                ofs << continue_auto << "\n";
                idx++;
                current_wp = 0;
            }
        }
    }
    else {
        std::cout << "Unable to open file\n";
        exit(EXIT_FAILURE);
    }

    ofs.close();

}

GeoCoords getGeoCoords(std::ifstream * f) {
    char latitude[50];
    char longitude[50];

    f->getline(latitude, 50, ' ');
    f->getline(longitude, 50);

    GeoCoords pt(atof(latitude), atof(longitude));

    return pt;
}


// Input format:
// IMAGE_FILENAME POINTS_FILENAME
int main(int argc, const char ** argv)
{
    // Check command line arguments
    if(argc < 3) {
        std::cout << "Not enough arguments given. Use as ./run IMAGE_FILENAME POINTS_FILENAME\n";
        exit(EXIT_FAILURE);
    }
    
    // Read POINTS_FILENAME for zone+hemisphere, easting, northing of start & goal
    std::ifstream points_file(argv[2]);
    GeoCoords start;
    GeoCoords goal;
    if(points_file.is_open()) {
        start = getGeoCoords(&points_file);
        goal  = getGeoCoords(&points_file);
    }
    else {
        std::cout << "Could not open file\n";
        return EXIT_FAILURE;
    }
    points_file.close();

    // Extract geospatial data from GeoTiff
    std::string image_name = argv[1];
    GDALDataset * poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen(image_name.c_str(), GA_ReadOnly );
    if( poDataset == NULL )
    {
       std::cout << "Unable to open GDALDataset from filename " << image_name << "\n";
       exit(EXIT_FAILURE);
    }

    double gt[6];  
    if(poDataset->GetGeoTransform(gt) != CE_None) {
        std::cout << "Unable to obtain GeoTiff transform\n";
        exit(EXIT_FAILURE);
    }

    assert(start.Zone() == goal.Zone());
    assert(start.Northp() == goal.Northp());

    Image img(gt[0], gt[3], poDataset->GetRasterXSize(), poDataset->GetRasterYSize(), 
              gt[1], gt[5], start.Zone(), start.Northp());

    std::cout << "Size: " << img.WIDTH << ", " << img.HEIGHT << "\n";
    std::cout << "Easting/Northing " << img.EASTING << ", " << img.NORTHING << "\n";
    std::cout << "Resolution: " << img.XRES << ", " << img.YRES << "\n";
    std::cout << "Zone, north? " << img.ZONE << ", " << img.isNORTH << "\n";

    // Convert start & goal coordinates to image coordinates
    Point startImg = utm2image(start, img);
    Point goalImg  = utm2image(goal, img);
   
    // Compute path
    Path path;
    computePath(path, startImg, goalImg, image_name);

    // Print path to a file
    printWaypoints(path, "path.txt", img);

    return EXIT_SUCCESS;
}
