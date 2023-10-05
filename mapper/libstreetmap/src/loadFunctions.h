//Function declarations and data structures for preloading the map
//These functions and data structures are called in loadMap()
#pragma once


#include "StreetsDatabaseAPI.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <stdbool.h>
#include <iostream>
#include <filesystem>
#include "streetAttributeData.h"
#include "ezgl/graphics.hpp"
#include <curl/curl.h>
#include <regex>

// namespace fs = std::filesystem;

#ifndef LOAD_FUNCTIONS_H
#define LOAD_FUNCTIONS_H

struct featureInfo{
    std:: string name;
    FeatureType type;
    TypedOSMID OSMID;
    int numPoints;
    std::vector<ezgl::point2d> points;
    float xMin, xMax, yMin, yMax;
    float area;

    ~featureInfo() {
        points.clear();
    }
};

struct IntersectionInfo {
    std::string name; // Name of the intersection
    LatLon position; // Geographical location
    OSMID osmNodeId; // Corresponding OSMID

    int numStreetSegments; // Number of street segments connected to it
    std::vector<StreetSegmentIdx> streetSegments; // Idxs of all the street segments
                                                    // connected to it
    
    // Idxs of all the intersections adjacent to this intersection
    std::vector<IntersectionIdx> adjacentIntersections; 
    std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>> connectingSegments; // Segments that connect the adjacent intersection
    bool highlight = false;

    ~IntersectionInfo() {
        adjacentIntersections.clear();
        connectingSegments.clear();
    }

};

struct SegmentInfo {
    OSMID wayOSMID;   // OSM ID of the source way
                      // NOTE: Multiple segments may match a single OSM way ID

    IntersectionIdx from, to;  // intersection ID this segment runs from/to
    bool oneWay;        // if true, then can only travel in from->to direction

    int numCurvePoints;      // number of curve points between the ends
    float speedLimit;        // in m/s
    float travelTime;       // Time taken to travel across this segment

    StreetIdx streetID;     // index of street this segment belongs to
    std::string streetName;

    std::string roadType;
    StreetAttribute attributes;
};

// Maps StreetIdx to all it's StreetSegmentIdx and IntersectionIdx
typedef std::pair<std::vector<StreetSegmentIdx>, std::set<IntersectionIdx>> SegIntPair;
typedef std::unordered_map<StreetIdx, SegIntPair> StreetTable;

// Make a hash table that maps partial strings to street names starting with that string
typedef std::vector<std::pair<StreetIdx, std::string>> StreetPairs;

//define structure for OSMID hash table
typedef std::pair<int, const OSMEntity*> ePair;

//define structure for street segment info
struct streetsegment{
    double distance;
    float speedLimit;

    double time;
};

//define struct to hold location information
struct LocationInfo {
    double min_lat;
    double min_lon;

    double max_lat;
    double max_lon;
};

struct StreetNameSegment {
    std::string streetName; // Name of the street
    
    ezgl::point2d p1, p2; // Starting and ending point

    double thickness; // Name size
    ezgl::color color;
};

struct HeatmapPoint {
    int id; // corresponds to the id in StreetsDatabase
    LatLon location;
    std::string name;
    std::string address;
    std::vector<std::vector<int>> popular_times;
};

struct XYCoordinates{
    double x;
    double y;
};

typedef std::vector<std::vector<std::vector<StreetSegmentIdx>>> StreetZones;

/*

The class MapData holds all the information for a specific map in custom datastructures.
These datastructures are created to more efficiently answer queries. The data is strucutred in
a way that allows easy access to all Intersections/Streets/Street segments in a map.

*/

class MapData {

public:

    int zoneWidth = 100;

    // Used to check if map was loaded correctly
    bool mapLoaded;
    LocationInfo locationInfo;

    // Maximum speed limit in city
    double maxSpeed;

    // Maps IntersectionIdx to all it's data
    std::vector<IntersectionInfo> intersectionInfoTable;
    // Maps StreetSegmentIdx to it's information Struct
    std::vector<SegmentInfo> streetSegmentTable;
    // Maps a street id to all it's intersections and street segments
    StreetTable streetInfoTable;

    // Weather Icons Data
    std::vector<std::string> weatherIcons;
    std::unordered_map<std::string, ezgl::surface*> weatherIconPath;

    // Vectors that contain the zoned street segments
    StreetZones highwaySSegments;
    StreetZones largeSSegments;
    StreetZones mediumSSegments;
    StreetZones smallSSegments;

    std::unordered_map<std::string, int> streetSizeMap = {
        {"motorway", 4}, {"trunk", 3},
        {"primary", 3}, {"secondary", 3},
        {"motorway_link", 2}, {"trunk_link", 2},
        {"primary_link", 2}, {"secondary_link", 2},
        {"tertiary", 1}, {"tertiary_link", 1}, 
        {"unclassified", 1}, {"residential", 1},
        {"default", 1}};

    //make hash table for OSMIDs
    std::unordered_map<OSMID, ePair> eTable;

    //make vector for street segments
    std::vector<streetsegment> streetSegmentVector;

    // Map partial strings to all the streets starting with that string
    std::unordered_map<std::string, StreetPairs> partialNameTable;

    //Store Features By Feature Area
    //std::vector<featureInfo> buildings;
    std::vector<featureInfo> smallFeatures;
    std::vector<featureInfo> mediumFeatures;
    std::vector<featureInfo> largeFeatures;
    std::vector<featureInfo> xLargeFeatures;
    std::vector<featureInfo> openFeatures;

    // Used for storing Heatmap Points
    std::vector<std::vector<std::vector<HeatmapPoint>>> heatmapData;

    //Maps poiidx to all its zones
    std::vector<std::vector<std::vector<POIIdx>>> poiZone;
    
    // Load all the icons png
    std::vector<std::string> poiType;

    //load all the path surfaces
    std::unordered_map<std::string, ezgl::surface*> surfacePaths;

    // load the lat lons of zones and their respective surface

    std::vector<std::vector<std::pair<double, ezgl::surface*>>> weatherSurface;

    ezgl::surface* weather_icon_surface;
    int temperature;
    
    //Maps building's featureIDX to all its zones
    std::vector<std::vector<std::vector<featureInfo>>> buildingZone; 

    MapData(); // Populate all private data strucutres
    ~MapData(); // Clear all private data structures

    // Load the street segments
    void loadStreetSegmentInfo();

    // Maps a street id to all it's intersections and street segments
    void loadStreetInfo ();


    // Loads all the adjacent interesections to a specific intersection
    std::vector<IntersectionIdx> loadFindAdjacentIntersections(IntersectionIdx intersection_id);

    // Populates the Intersection Info Table
    void loadIntersectionData ();

    //populates feature info table
    void loadFeatureInfoTable();

    //fills the OSMID hash table
    void fillOSMIDTable();

    std::string getOSMEntityTagValue (OSMID OSMid, std::string key);

    // load POIData
    void loadPoiData ();

    //load weather data
    void loadWeather();

    //fills the street segment vector
    void loadStreetSegmentVector();

    //compute distance between two points
    double loadFindDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2);

    //compute street segment length
    double loadFindStreetSegmentLength(StreetSegmentIdx street_segment_id);

    double  loadFindFeatureArea(FeatureIdx feature_id);
    

    double x_from_lon(float lon);
    double y_from_lat(float lat);
    float lon_from_x(double x);
    float lat_from_y(double y);

    void loadHeatmapData();


};

MapData* getCurrentMap();

bool turn_penality_check(StreetSegmentIdx &cur, StreetSegmentIdx &next);
extern std::unordered_map<std::string, std::string> CityPaths;
extern std::string directory_path;


#endif // LOAD_FUNCTIONS_H