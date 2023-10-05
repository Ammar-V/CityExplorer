/* 
 * Copyright 2023 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>
#include "m1.h"
#include <unordered_set>
#include <set>
#include <algorithm> 
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <OSMDatabaseAPI.h>
#include "loadFunctions.h"
#include "loadMaps.h"


#include <limits>
#include <fstream>
#include <chrono>


// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.

//Declare global variables

MapData* map; // Object to access all the custom data structures

bool loadSuccessful = false; // Used to check if map was loaded before using API


MapData* getCurrentMap() {
    return map;
}

bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    // Start timer
    std::chrono::time_point<std::chrono::system_clock> start, end;

    start = std::chrono::system_clock::now();

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    //loading the map
    load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    if (load_successful == false){
        loadSuccessful = false;
        return false;
    }


    //load OSM Database
    std::string toReplace(".streets");
    size_t position = map_streets_database_filename.find(toReplace);
    map_streets_database_filename.replace(position, toReplace.length(), ".osm");

    bool load_OSMbin = loadOSMDatabaseBIN(map_streets_database_filename);

    //return false if OSM database did not load successfully
    if (load_OSMbin == false) {
        loadSuccessful = false;
        return false;
    }

    // Load data into custom data structures
    map = new MapData();
    
    // End time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
   
    std::cout << "Map loaded in: " << elapsed_seconds.count() << " seconds." << std::endl;

    loadSuccessful = true;
    return load_successful;
}

void closeMap() {

    if (!loadSuccessful) {return;}
    
    delete map;
    //Close databases
    closeStreetDatabase();
    closeOSMDatabase();

    std::cout << "Closing APIs..." << std::endl;
}

double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){
    if (loadSuccessful == false){
        return 0;
    }

    return map->loadFindDistanceBetweenTwoPoints(point_1, point_2);  

}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    if (loadSuccessful == false){
        return 0;
    }
    return map->streetSegmentVector[street_segment_id].distance;
}


double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    if (loadSuccessful == false){
        return 0;
    }
    
    double distance = map->streetSegmentVector[street_segment_id].distance;
    double speedLimit = map->streetSegmentVector[street_segment_id].speedLimit;

    return distance/speedLimit;
}


std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    if (loadSuccessful == false){
        std::vector<IntersectionIdx> intersection;
        return intersection;
    }

    return map->intersectionInfoTable[intersection_id].adjacentIntersections;
}


IntersectionIdx findClosestIntersection(LatLon my_position){

    IntersectionIdx closestIdx = -1;

    if (loadSuccessful == false){
        return closestIdx;
    }

    // Start off with the largest distance as the closestDistance
    double closestDistance = std::numeric_limits<double>::infinity();

    for (int idx = 0; idx < getNumIntersections(); idx++) {
        LatLon &position = map->intersectionInfoTable[idx].position; // Get position of intersection
        double distance = findDistanceBetweenTwoPoints(my_position, position);

        if (distance < closestDistance) { // Smaller distance found
            closestDistance = distance;
            closestIdx = idx;
        }
    }
    
    return closestIdx;
}


std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection (IntersectionIdx intersection_id ) {
    if (loadSuccessful == false){
        std::vector<IntersectionIdx> intersection;
        return intersection;
    }

	return map->intersectionInfoTable[intersection_id].streetSegments;
}


std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    if (loadSuccessful == false){
        std::vector<IntersectionIdx> intersection;
        return intersection;
    }

    std::set<IntersectionIdx> intersections = map->streetInfoTable[street_id].second;

    return std::vector<IntersectionIdx>(intersections.begin(), intersections.end());
}


std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(StreetIdx street_id1, StreetIdx street_id2){
    if (loadSuccessful == false){
        std::vector<IntersectionIdx> intersection;
        return intersection;
    }

    std::set<IntersectionIdx> &intersectionsA = map->streetInfoTable[street_id1].second;
    std::set<IntersectionIdx> &intersectionsB = map->streetInfoTable[street_id2].second;

    std::vector<IntersectionIdx> result;

    // The intersection of two sets results in a vector with elements that belong to both sets
    set_intersection(intersectionsA.begin(), intersectionsA.end(),
                     intersectionsB.begin(), intersectionsB.end(),
                     std::inserter(result, result.end()));

    return result;
}


std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
    //creating vectors needed
    std::unordered_set<StreetIdx> allStreetIdx;
    StreetSegmentInfo StreetSegmentInformation;
    std::vector<StreetIdx> allStreetId;

    if(street_prefix.length() == 0 || loadSuccessful == false){
        return allStreetId;
    }

    // Check for string length
    if (street_prefix.length() == 1) {

        //We also change the given street_prefix to small case and remove spaces 
        // Eg: if given "BlOOrSt" -> "bloorst", which is clearly a prefix to "bloorstreetwest"
        std::transform(street_prefix.begin(), street_prefix.end(),street_prefix.begin(), ::tolower);
        street_prefix.erase(std::remove_if(street_prefix.begin(), street_prefix.end(), ::isspace),street_prefix.end());

        // Find all combinations of two characters
        std::string alphabet = "abcdefghijklmnopqrstuvwxyz";

        // After adding loop through those and check the unodered map->
        // Eg: we get the prefix "A", then we check in key "aa", "ab" .... "az"
        for (int i = 0; i < 26; i++) {
            std::string partialName = street_prefix + alphabet[i];
            StreetPairs &streetPairs = map->partialNameTable[partialName];
            for (int j = 0; j < streetPairs.size(); j++) {
                allStreetIdx.insert(streetPairs[j].first);
            }
        }

    } else if (street_prefix.length() > 1) { 
        // Prefix is of length at least 2
        //We also change the given street_prefix to small case and remove spaces 
        // Eg: if given "BlOOrSt" -> "bloorst", which is clearly a prefix to "bloorstreetwest"
        std::transform(street_prefix.begin(), street_prefix.end(),street_prefix.begin(), ::tolower);
        street_prefix.erase(std::remove_if(street_prefix.begin(), street_prefix.end(), ::isspace),street_prefix.end());
        std::string partialName = street_prefix.substr(0, 2);

        StreetPairs &streetPairs = map->partialNameTable[partialName];

        // traverse throught the vector using the key which is the fist two letters
        // Eg: we are given prefix - "Bloor", then we use the key "bl" and then value is a vector 
        // the vector is all the street segments that begin with that prefix
        for (int i = 0; i < streetPairs.size(); i++) {
            std::pair<StreetIdx, std::string> &streetPair = streetPairs[i];
            if (streetPair.second.rfind(street_prefix, 0) == 0) {
                allStreetIdx.insert(streetPair.first);
            }
        }
    }

    return std::vector<StreetIdx>(allStreetIdx.begin(), allStreetIdx.end());;

}


double findStreetLength(StreetIdx street_id){
    if (loadSuccessful == false){
        return 0;
    }
    double StreetLength = 0;

    // declaring the variable to use in the for loop
    std::vector<StreetSegmentIdx> streetSegments = map->streetInfoTable[street_id].first;

    // traverse through the street segments and find whichever correspond as a part of the street
    // then find the length of those segments and add it to the total street length
    for (int i = 0; i < streetSegments.size(); i++){
        StreetLength += findStreetSegmentLength(streetSegments[i]);
    }

    return StreetLength;
}


POIIdx findClosestPOI(LatLon my_position, std::string POItype){

    POIIdx closestPOI = 0;
    double distanceBetweenPoints = 0.0;
    double closestDistance = std::numeric_limits<double>::infinity();

    // find the closest posiiton, and if it matches the type of POItype, compare with previous value
    // if its closer then make that the new POI
    for (int i = 0; i < getNumPointsOfInterest(); i++){
        if (POItype == getPOIType(i)){
            // we find the distance between our position and the point being traversed through
            distanceBetweenPoints = map->loadFindDistanceBetweenTwoPoints(my_position, getPOIPosition(i));

            //compare distances, if the found distance is less than current distance, update
            if (closestDistance > distanceBetweenPoints){
                closestPOI = i;
                closestDistance = distanceBetweenPoints;
            }
        }
    }
    
    return closestPOI;
}

double findFeatureArea(FeatureIdx feature_id){
    if(loadSuccessful == false){
        return 0;
    }
    std::vector<std::pair<double, double>> allCoordinates;

    int NumFeaturePoints = getNumFeaturePoints(feature_id);
    double xCoordinate = 0.0;
    double yCoordinate = 0.0;
    double latAvg = 0.0;
    double Area = 0.0;
    int j = 0; // used in loop

    // Check if it has to have atleast 4 points - because a minimum polygon would be a trinagle
    // It has 3 points and the 4th point signifies if its closed
    // if not, its a polyline and hence return 0
    // if it is, add the coordinates to vector

    if(NumFeaturePoints <= 3){
        return 0;
    }

    // check if the polygon is closed 
    LatLon firstCoordinate = getFeaturePoint(feature_id, 0);
    LatLon lastCoordinate = getFeaturePoint(feature_id, NumFeaturePoints - 1);
    if(firstCoordinate.longitude() == lastCoordinate.longitude() && firstCoordinate.latitude() == lastCoordinate.latitude()){
        // calculating the average latitude
        for(int i = 0; i < (NumFeaturePoints - 1); i++){
            latAvg += getFeaturePoint(feature_id, i).latitude();
        }
    }

    else{
        return 0;
    }

    latAvg = kDegreeToRadian * (latAvg/double(NumFeaturePoints - 1));


    // adding the first coordinate
    xCoordinate = (kEarthRadiusInMeters * firstCoordinate.longitude() * kDegreeToRadian * cos(latAvg));
    yCoordinate = (kEarthRadiusInMeters * firstCoordinate.latitude() * kDegreeToRadian);
    allCoordinates.push_back(std::make_pair(xCoordinate, yCoordinate));

    // Here we add the x and y coordinates as a pair to the vector allCoordinates
    // We then use the shoelace formula to calculate the area of any simple polygon 
    // The shoelace formula is area = 1/2 * |(x1*y2 + x2*y3 + ... * xn*y1) - (y1*x2 + y2*x3 + ... + yn*x1)|
    
    for(int i = 1; i < (NumFeaturePoints); i++){
        LatLon coordinate = getFeaturePoint(feature_id, i);
        // calculating the x and y coordinates based on the longitudes and latitudes
        xCoordinate = (kEarthRadiusInMeters * coordinate.longitude() * kDegreeToRadian * cos(latAvg));
        yCoordinate = (kEarthRadiusInMeters * coordinate.latitude() * kDegreeToRadian);
        allCoordinates.push_back(std::make_pair(xCoordinate, yCoordinate));
        j = (i - 1);
        // using the shoe lace formula here
        Area += abs(allCoordinates[j].first * allCoordinates[i].second);
        Area -= abs(allCoordinates[i].first * allCoordinates[j].second);
    }

    return abs(Area/2.0);

}


std::string getOSMNodeTagValue (OSMID OSMid, std::string key){
    if(loadSuccessful == false){
        std::string empty = "";
        return empty;
    }

    std::string value;

    auto it = map->eTable.find(OSMid);//get the OSMNode correpsonding to the OSMID

    //loop through all Tag Pairs
    //if key found, return the correspondng value
    //if key isn't found, return an empty string
    for(int i = 0; i < getTagCount((it->second).second); i++){
   		std::string temp;
        std::tie(temp,value) = getTagPair((it->second).second,i);
   		if(temp == key){
            return value;
        }
   }
    
    return "";

}
