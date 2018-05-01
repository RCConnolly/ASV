/* Main script for running wFM2 for ASV */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>
#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

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

using namespace GeographicLib; // For coordinate system conversions

// Class containing fields GeoTiff data necessary for coordinate convertions
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


/* Converts point coordinates from UTM northing & easting in decimal degrees
 * to image (x,y) coordinates with the origin at the image's bottom,left corner.
*/
Point utm2image(GeoCoords & utm_pt, Image & img) {
    
    double  x_img = (utm_pt.Easting() - img.EASTING)/img.XRES;

    // If y-resolution is negative, the image origin is in top left.
    // Image coordinates must be in bottom left.    
    double y_img;    
    if(img.YRES < 0) {
        y_img = (utm_pt.Northing() - (img.NORTHING + img.HEIGHT*(img.YRES)))/(-img.YRES);
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
 *      img = Image object that the path is being generated for
*/
void computePath(Path & path, const Point & start, const Point & goal, const std::string filename,
                 const Image & img, const int safe_dist) {
    // Loading grid.
    FMGrid grid_fm2;

    // convert "safety distance" from meters to number of image pixels
    int safe_cell_dist = ceil((2*(safe_dist/img.XRES))/sqrt(2)); 

    // Solver declaration.
    Solver<FMGrid> * s = new FM2<FMGrid>("FM2_Dary", safe_cell_dist);
   
    // fills in occupancy grid    
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2);
    s->setEnvironment(&grid_fm2);

    
    // computes velocities map 
    Coord start_coord = {(unsigned int) round(start[0]), (unsigned int) round(start[1])};
    Coord goal_coord  = {(unsigned int) round(goal[0]),  (unsigned int) round(goal[1])};   
    s->setInitialAndGoalPoints(start_coord, goal_coord);
    s->compute();

    // generate paths
    std::vector<double> path_vels;
    s->as<FM2 <FMGrid> >()->computePath(&path, &path_vels);

    /// uncomment for plotting generated path ///    
    //GridPlotter::plotArrivalTimesPath(grid_fm2, path);

    // Preventing memory leaks.
    delete s;

    return;
}

/* Prints path points to a waypoint path file formatted for uploading to Ardupilot Mission Planner.
 * Parameters:
 *     path: image coordinates of each waypoint
 *     fname: name of file to print to
 *     img: image that path was generated on
*/
void printWaypoints(Path & path, const char * fname, Image & img, const int pt_dist) {

    // Establish MAVLink Constants
    const char MAV_VERSION[] = "110";
    const int  MAV_FRAME = 0; // 0 = MAV_FRAME_GlOBAL, WGS84 coordinate system https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
    const int MAV_CMD = 16; // 16 = MAV_CMD_NAV_WAYPOINT, same message definition url as above
    const double HOLD_TIME = 0; // in seconds
    const double WP_RADIUS = 5; // in meters
    const double PASS_BY_DIST = 0; // distance in meters that ASV should pass by the waypoint
    const float ROTARY_WING_YAW = 0; // or NAN?
    const double ALTITUDE = 0;
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        ofs << "QGC WPL " << MAV_VERSION << "\n";
        int idx = 0;
        bool current_wp = 1;
        bool continue_auto = 1;

        ofs.precision(10);
        GeoCoords curr_wp;
        for(size_t i = 0; i < path.size(); i++) {
            if(i%((int)(pt_dist/img.XRES)) == 0) {            
                ofs << idx << "\t";
                ofs << current_wp << "\t";
                ofs << MAV_FRAME << "\t";
                ofs << MAV_CMD << "\t";
                ofs << HOLD_TIME << "\t"; // PARAM 1
                ofs << WP_RADIUS << "\t"; // PARAM 2
                ofs << PASS_BY_DIST << "\t"; // PARAM 3
                ofs << ROTARY_WING_YAW << "\t"; // PARAM 4
                if(img.YRES < 0) {
                    curr_wp.Reset(img.ZONE, img.isNORTH,
                                 (path[i][0]*img.XRES)+img.EASTING,
                                 (path[i][1]*(-img.YRES) + (img.NORTHING+img.HEIGHT*(img.YRES))));
                }
                else {
                    curr_wp.Reset(img.ZONE, img.isNORTH,
                                 (path[i][0]*img.XRES)+img.EASTING,
                                 (path[i][1]*img.YRES)+img.NORTHING);
                }
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

/* Reads position coordinates from a filestream, and returns them as a GeoCoords object.
 * Each line in the filestream must have the format: LATITUDE LONGITUDE
*/
GeoCoords getGeoCoords(std::ifstream * f) {
    char latitude[50];
    char longitude[50];

    f->getline(latitude, 50, ' ');
    f->getline(longitude, 50);

    GeoCoords pt(atof(latitude), atof(longitude));

    return pt;
}


/* Command line argument format: IMAGE_PATH POINTS_PATH
 *     IMAGE_PATH: relative path name of the GeoTiff obstacle image
 *    POINTS_PATH: relative path name of txt file containing the start and goal positions
 *                 in latitude/longitude format
*/
int main(int argc, const char ** argv)
{
    // Check command line arguments
    if(argc != 5 ) {
        std::cout << "Incorrect number of arguments given. Use as ./FM2 IMAGE_PATH POINTS_FILENAME SAFE_DIST PT_DIST\n";
        exit(EXIT_FAILURE);
    }
    
    // Read POINTS_FILENAME for latitude/longitude coordinate of start and goal
    std::cout << "Reading lat/long of start and goal positions... ";
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
    std::cout << "finished.\n";

    // Extract geospatial data from GeoTiff
    std::cout << "Parsing GeoTiff image... ";
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
    GDALClose((GDALDatasetH) poDataset);
    std::cout << "finished.\n";

    // Convert start & goal coordinates to image coordinates
    Point startImg = utm2image(start, img);
    Point goalImg  = utm2image(goal, img);
   
    // Compute path
    std::cout << "Computing path... ";
    Path path;
    const int safe_dist = atoi(argv[3]);
    computePath(path, startImg, goalImg, image_name.substr(0, image_name.length()-3) += "png", img, safe_dist);
    std::cout << "finished.\n";

    // Print path to a file
    const int pt_dist = atoi(argv[4]);
    printWaypoints(path, "path.txt", img, pt_dist);
    std::cout << "Printed waypoint file to path.txt\n";

    return EXIT_SUCCESS;
}
