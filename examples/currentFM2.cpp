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
//#include <fast_methods/fm2/wFM2.hpp>

#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridplotter.hpp>
#include <fast_methods/io/gridwriter.hpp>

#include <GeographicLib/GeoCoords.hpp>


// Establish Grid & Path Dimensionality to 2
constexpr unsigned int nDims = 2;
typedef nDGridMap<FMCell, nDims> FMGrid;
typedef typename std::array<double, nDims> Point;
typedef typename std::array<unsigned int, nDims> Coord;
typedef typename std::vector<Point> Path;

// Establish global constants
const double IMG_NORTHING = 3836014;
const double IMG_EASTING  = 340154;
const int IMG_WIDTH = 1316;
const int IMG_HEIGHT = 1023;
const char IMG_ZONE_STR[] = "18n";
const int IMG_ZONE = 18;
const bool IMG_isNORTH = true;
const char MAV_VERSION[] = "110";
const int  MAV_FRAME = 0; // 0 = MAV_FRAME_GlOBAL, WGS84 coordinate system https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
const int MAV_CMD = 16; // 16 = MAV_CMD_NAV_WAYPOINT, same message definition url as above
const double HOLD_TIME = 0; // in seconds
const double WP_RADIUS = 10; // in meters
const double PASS_BY_DIST = 0; // distance in meters that ASV should pass by the waypoint
const float ROTARY_WING_YAW = NAN;
const double ALTITUDE = 0;
long unsigned int POINT_DIST = 2;


/* Converts point coordinates from UTM northing & easting in decimal degrees
 * to image (x,y) coordinates with the bottom left corner of the image being
 * (0,0) and the x & y axes increasing to the right & upward respectively.
 * Parameters:
 *      utm_pt = {northing, easting} coordinates of point
 * Returns: none
*/
Point utm2image(Point & utm_pt) {
    
    assert(utm_pt[0] > IMG_NORTHING);
    assert(utm_pt[1] > IMG_EASTING);

    double  y_img = utm_pt[0] - IMG_NORTHING;
    double  x_img = utm_pt[1] - IMG_EASTING;

    if(x_img >= IMG_WIDTH) {
        //std::cout << "Converted image x-coordinate" << x_img << " out of range\n";
        //std::cout << "utm[1]: " << utm_pt[1] << "\n";
        //std::cout << "IMG_EASTING: " << IMG_EASTING << "\n";
        exit(EXIT_FAILURE);
    }

    if(y_img >= IMG_HEIGHT) {
        //std::cout << "Converted image y-coordinate out of range\n";
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
    // HARD CODED MAX DISTANCE
    // MODIFY SO THAT IT IS 60m based on image resolution
    double maxDist = 8; // maxDist = 2*(60m/pixel_res)/sqrt(2)
    Solver<FMGrid> * s = new FM2<FMGrid>("FM2_Dary", maxDist);
   
    // fills in occupancy grid    
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2);
    s->setEnvironment(&grid_fm2);
    
    // computes velocities map 
    Coord start_coord = {(unsigned int) start[0], (unsigned int) start[1]};
    Coord goal_coord  = {(unsigned int)  goal[0],  (unsigned int) goal[1]};   
    //std::cout << "goal coord: " << goal_coord[0] << ", " << goal_coord[1] << "\n";
    //std::cout << "start coord: " << start_coord[0] <<  ", " << start_coord[1] << "\n";
    s->setInitialAndGoalPoints(start_coord, goal_coord);
    s->compute();

    std::vector<double> path_vels;
    s->as<FM2<FMGrid>>()->computePath(&path, &path_vels);
    GridPlotter::plotArrivalTimesPath(grid_fm2, path);

    // Preventing memory leaks.
    delete s;

    return;
}


// Prints each path point a file, with coordinates being in Lat/Long
void printUTMPath(Path & path, const char * fname) {
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        for(size_t i = 0; i < path.size(); i++) {
            //convert to utm before printing
            ofs << "18n "; // zone
            ofs.precision(10);
            ofs << (path[i][1]+IMG_NORTHING) << " "; // northing
            ofs << (path[i][0]+IMG_EASTING) << "\n";  // easting
        }
    }
    else {
        std::cout << "Unable to open file\n";
        exit(EXIT_FAILURE);
    }
}


void printWaypoints(Path & path, const char * fname) {
    
    std::ofstream ofs(fname);

    if(ofs.is_open()) {
        ofs << "QGC WPL " << MAV_VERSION << "\n";
        int idx = 0;
        bool current_wp = 1;
        bool continue_auto = 1;

        ofs.precision(10);
        typename GeographicLib::GeoCoords curr_wp;
        for(size_t i = 0; i < path.size(); i++) {
            if(i%POINT_DIST == 0) {            
                ofs << idx << "\t" << current_wp << "\t";
                ofs << HOLD_TIME << "\t"; // PARAM 1
                ofs << WP_RADIUS << "\t"; // PARAM 2
                ofs << PASS_BY_DIST << "\t"; // PARAM 3
                ofs << ROTARY_WING_YAW << "\t"; // PARAM 4
                
                curr_wp.Reset(IMG_ZONE, IMG_isNORTH, path[i][0]+IMG_EASTING, path[i][1]+IMG_NORTHING);
                ofs << curr_wp.Latitude() << "\t";
                ofs << curr_wp.Longitude() << "\t";
                ofs << ALTITUDE << "\t";
                ofs << continue_auto << "\n";
            }
        }
    }
    else {
        std::cout << "Unable to open file\n";
        exit(EXIT_FAILURE);
    }

    ofs.close();

}




Point getPoint(std::ifstream * f) {
    char northing[100];
    char easting[100];

    f->getline(northing, 100, ' ');
    f->getline(easting, 100, '\n');

    double iNorthing = atof(northing);
    double iEasting  = atof(easting);
    //std::cout << "northing: " << iNorthing;
    //std::cout << ", easting: " << iEasting << "\n";

    Point pt = {iNorthing, iEasting};

    return pt;
}

void checkZone(const char * pt_zone, const char * img_zone) {
    if(strcmp(pt_zone, img_zone)) {
        std::cout << "ERROR: Point UTM zone (" << pt_zone << ")";
        std::cout << " does not match image zone(" << img_zone << ")\n";
        exit(EXIT_FAILURE);
    }
    
    return;
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

    // Parse GeoTiff image
    ///////////////////////////
    // NEED TO IMPLEMENT
    //////////////////////////
    // hard coding values for channels_updated.png image
    std::string image_name = argv[1];

    
    // Read POINTS_FILENAME for zone+hemisphere, easting, northing of start & goal
    std::ifstream points_file(argv[2]);
    Point start;
    Point goal;
    if(points_file.is_open()) {
        // Set starting point & check its UTM zone
        char start_zone[4];
        points_file.get(start_zone, 4); // reads in zone (2 digits, 1 letter)
        checkZone(start_zone, IMG_ZONE_STR);
        points_file.get(); // moves passed white space after zone
        //std::cout << "start point -- ";
        start = getPoint(&points_file);

        // Set goal point & check its UTM zone
        char goal_zone[4];
        points_file.get(goal_zone, 4);
        checkZone(goal_zone, IMG_ZONE_STR);
        points_file.get(); // moves passed white space after zone
        //std::cout << "end point -- ";
        goal  = getPoint(&points_file);
    }
    else {
        std::cout << "Could not open file\n";
        return EXIT_FAILURE;
    }
    points_file.close();

    // Convert start & goal coordinates to image coordinates
    Point startImg = utm2image(start);
    Point goalImg  = utm2image(goal);
    //std::cout << "start: {" << start_coord[0] << "," << start_coord[1] << "}\n";
    //std::cout << "goal: {" << goal_coord[0] << "," << goal_coord[1] << "}\n";

    // Compute path
    Path path;
    computePath(path, startImg, goalImg, image_name);

    // Print path to a file
    printWaypoints(path, "path.txt");


    return EXIT_SUCCESS;
}
