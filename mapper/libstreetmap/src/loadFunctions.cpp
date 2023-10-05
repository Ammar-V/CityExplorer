//Implementations of the helper functions declared in loadFunction.h

#include <iostream>
#include <unordered_set>
#include "StreetsDatabaseAPI.h"
#include "loadFunctions.h"
#include <OSMDatabaseAPI.h>
#include <cmath>
#include "m1.h"

// #include "streetAttributeData.h"
#include <stdlib.h>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// std::string data = std::filesystem::current_path();
// std::string path = data.substr(0, data.length() - 14);
std::string directory_path = "/nfs/ug/homes-3/v/voraamma/ece297/work/mapper";

MapData::MapData() {

    //fill table of OSMID and (idx, entities)
    fillOSMIDTable();
    
    // Load the intersection street segments information
    loadIntersectionData();
    
    // Loading Street segments into table
    loadStreetSegmentInfo();

    // Find all the adjacent Intersection Idxs
    for (IntersectionIdx i = 0; i < getNumIntersections(); i++) {
        intersectionInfoTable[i].adjacentIntersections = loadFindAdjacentIntersections(i);

    }

    //load feature information
    loadFeatureInfoTable();

    // Load Street information
    loadStreetInfo();

    //load vector of street segment distances
    loadStreetSegmentVector();
    
    // load POI data
    loadPoiData();

    // load weather data
    loadWeather();

    

}

double MapData::loadFindFeatureArea(FeatureIdx feature_id){

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


void MapData::loadFeatureInfoTable(){

    //initialize buildingZone table
    double max_x = x_from_lon(locationInfo.max_lon);
    double max_y = y_from_lat(locationInfo.max_lat);
    double min_x = x_from_lon(locationInfo.min_lon);
    double min_y = y_from_lat(locationInfo.min_lat);

    int colNum = abs(max_x - min_x)/zoneWidth;
    int rowNum = abs(max_y - min_y)/zoneWidth;

    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<featureInfo>> col;
        buildingZone.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<featureInfo> row;
            buildingZone[i].push_back(row);
        }
    }

    for (int id = 0; id < getNumFeatures(); id++) {
        featureInfo temp;
        temp.xMin = std::numeric_limits<double>::infinity();
        temp.xMax = 0;
        temp.yMin = std::numeric_limits<double>::infinity();
        temp.yMax = 0;

        //Store feature data
        temp.name = getFeatureName(id);
        temp.type = getFeatureType(id);
        temp.OSMID = getFeatureOSMID(id);
        temp.numPoints = getNumFeaturePoints(id);

        for(int j = 0; j < temp.numPoints; j++){
            LatLon point = getFeaturePoint(id,j);
            float x = x_from_lon(point.longitude());
            float y = y_from_lat(point.latitude());

            temp.points.push_back({x,y});
            if(x < temp.xMin){
                temp.xMin = x;
            }
            if(x > temp.xMax){
                temp.xMax = x;
            }
            if(y < temp.yMin){
                temp.yMin = y;
            }
            if(y > temp.yMax){
                temp.yMax = y;
            }
        }

        temp.area = loadFindFeatureArea(id);

        //store closed features by area
        if(temp.area == 0){
            openFeatures.push_back(temp);
        }
        if(temp.area > 1000000){
            xLargeFeatures.push_back(temp);
        }
        else if(temp.area > 500000){
            largeFeatures.push_back(temp);
        }
        else if(temp.area > 200000){
            mediumFeatures.push_back(temp);
        }
        else if(temp.type == 6){ //store buildings by zone

            int col = abs(min_x - temp.xMin ) / zoneWidth;
            int row = abs(min_y - temp.yMin ) / zoneWidth;


            if ( (col < colNum) && (col >= 0) && (row < rowNum) && (row >= 0)){

                buildingZone[col][row].push_back(temp);

            }
        }
        else{
            smallFeatures.push_back(temp);
        }

    }

}

void MapData::loadPoiData(){

    double max_x = x_from_lon(locationInfo.max_lon);
    double max_y = y_from_lat(locationInfo.max_lat);
    double min_x = x_from_lon(locationInfo.min_lon);
    double min_y = y_from_lat(locationInfo.min_lat);

    // std::cout << "lonmax = " << max_x << "latmax = " << max_y << "lonmin = " << min_x << "latmin = " << min_y << std::endl;

    int colNum = abs(max_x - min_x)/zoneWidth;
    int rowNum= abs(max_y - min_y)/zoneWidth;
    
    // std::cout << "the col_max = " << colNum << " the row_max = " << rowNum << std::endl;
    
    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<POIIdx>> col;
        poiZone.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<POIIdx> row;
            poiZone[i].push_back(row);
        }
    }

    for (int i = 0; i < getNumPointsOfInterest(); i++){
        LatLon latLon = getPOIPosition(i);
        int col = abs(min_x - x_from_lon(latLon.longitude())) / zoneWidth;
        int row = abs(min_y - y_from_lat(latLon.latitude())) / zoneWidth;
        
        //std::cout << "crashed" << std::endl;
         
        if ( (col < colNum) && (col >= 0) && (row < rowNum) && (row >= 0)){
            //std::cout << "the col = " << col << " the row = " << row << std::endl;
            poiZone[col][row].push_back(i);
        }
    }
    
    poiType = {
        directory_path + "/libstreetmap/resources/icons/bank-min.png",
        directory_path + "/libstreetmap/resources/icons/fuel-min.png",
        directory_path + "/libstreetmap/resources/icons/prep_school-min.png",
        directory_path + "/libstreetmap/resources/icons/place_of_worship-min.png",
        directory_path + "/libstreetmap/resources/icons/restaurant-min.png",
        directory_path + "/libstreetmap/resources/icons/doctors-min.png",
        directory_path + "/libstreetmap/resources/icons/cafe-min.png",
        directory_path + "/libstreetmap/resources/icons/bar-min.png",
        directory_path + "/libstreetmap/resources/icons/dentist-min.png",
        directory_path + "/libstreetmap/resources/icons/college-min.png",
        directory_path + "/libstreetmap/resources/icons/hotel-min.png",
        directory_path + "/libstreetmap/resources/icons/gym-min.png",
        directory_path + "/libstreetmap/resources/icons/bicycle_rental-min.png",
        directory_path + "/libstreetmap/resources/icons/fast_food-min.png",
        directory_path + "/libstreetmap/resources/icons/beach-min.png",
        directory_path + "/libstreetmap/resources/icons/ice_cream-min.png",
    };
        
        
    ezgl::renderer *g;
    surfacePaths.clear();
    surfacePaths.insert({
    {"bank", g->load_png(poiType[0].c_str()) },
    {"doctors", g->load_png(poiType[5].c_str()) },
    {"restaurant", g->load_png(poiType[4].c_str()) },
    {"hospital", g->load_png(poiType[5].c_str()) },
    {"fast_food", g->load_png(poiType[13].c_str()) },
    {"place_of_worship", g->load_png(poiType[3].c_str())},
    {"ice_cream", g->load_png(poiType[15].c_str()) },
    {"pub", g->load_png(poiType[7].c_str()) },
    {"bar", g->load_png(poiType[7].c_str()) },
    {"cafe", g->load_png(poiType[6].c_str()) },
    {"pharmacy", g->load_png(poiType[5].c_str()) },
    {"bicycle_rental", g->load_png(poiType[12].c_str()) },
    {"college", g->load_png(poiType[9].c_str()) },
    {"university", g->load_png(poiType[9].c_str()) },
    {"gym", g->load_png(poiType[11].c_str()) },
    {"fuel", g->load_png(poiType[1].c_str()) },
    {"hotel", g->load_png(poiType[10].c_str()) },
    {"beach", g->load_png(poiType[14].c_str()) },
    {"dentist", g->load_png(poiType[8].c_str()) },
    {"prep_school", g->load_png(poiType[2].c_str()) },
    {"child_care", g->load_png(poiType[2].c_str()) },
    {"kindergarten", g->load_png(poiType[2].c_str()) },
    {"school", g->load_png(poiType[9].c_str()) },
    {"atm", g->load_png(poiType[0].c_str()) },
    });
    
}

void MapData::loadWeather(){
    weatherIcons = {
        directory_path + "/libstreetmap/resources/weather-icons/01d.png",
        directory_path + "/libstreetmap/resources/weather-icons/01n.png",
        directory_path + "/libstreetmap/resources/weather-icons/02d.png",
        directory_path + "/libstreetmap/resources/weather-icons/02n.png",
        directory_path + "/libstreetmap/resources/weather-icons/03d.png",
        directory_path + "/libstreetmap/resources/weather-icons/03n.png",
        directory_path + "/libstreetmap/resources/weather-icons/04d.png",
        directory_path + "/libstreetmap/resources/weather-icons/04n.png",
        directory_path + "/libstreetmap/resources/weather-icons/09d.png",
        directory_path + "/libstreetmap/resources/weather-icons/09n.png",
        directory_path + "/libstreetmap/resources/weather-icons/10d.png",
        directory_path + "/libstreetmap/resources/weather-icons/10n.png",
        directory_path + "/libstreetmap/resources/weather-icons/11d.png",
        directory_path + "/libstreetmap/resources/weather-icons/11n.png",
        directory_path + "/libstreetmap/resources/weather-icons/13d.png",
        directory_path + "/libstreetmap/resources/weather-icons/13n.png",
        directory_path + "/libstreetmap/resources/weather-icons/50n.png",
        directory_path + "/libstreetmap/resources/weather-icons/50n.png",
    };

    ezgl::renderer *g;
    weatherIconPath.clear();
    weatherIconPath.insert({
        {"01d", g->load_png(weatherIcons[0].c_str()) },
        {"01n", g->load_png(weatherIcons[1].c_str()) },
        {"02d", g->load_png(weatherIcons[2].c_str()) },
        {"02n", g->load_png(weatherIcons[3].c_str()) },
        {"03d", g->load_png(weatherIcons[4].c_str()) },
        {"03n", g->load_png(weatherIcons[5].c_str())},
        {"04d", g->load_png(weatherIcons[6].c_str()) },
        {"04n", g->load_png(weatherIcons[7].c_str()) },
        {"09d", g->load_png(weatherIcons[8].c_str()) },
        {"09n", g->load_png(weatherIcons[9].c_str()) },
        {"10d", g->load_png(weatherIcons[10].c_str()) },
        {"10n", g->load_png(weatherIcons[11].c_str()) },
        {"11d", g->load_png(weatherIcons[12].c_str()) },
        {"11n", g->load_png(weatherIcons[13].c_str()) },
        {"13d", g->load_png(weatherIcons[14].c_str()) },
        {"13n", g->load_png(weatherIcons[15].c_str()) },
        {"50d", g->load_png(weatherIcons[16].c_str()) },
        {"50n", g->load_png(weatherIcons[17].c_str()) },
    });

    std::string icon;
    double temp = -1;

    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    
    float lat = round(locationInfo.max_lat + locationInfo.min_lat)/2.0;
    float lon = round(locationInfo.max_lon + locationInfo.min_lon)/2.0;
    
    std::string lat_s = std::to_string(lat);
    std::string lon_s = std::to_string(lon);

    std::string URL = "https://api.openweathermap.org/data/2.5/weather?lat=" + lat_s + "&lon=" + lon_s + "&appid=635275d13710e16053c9a25f45c0d70b";

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, URL.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        
        std::regex iconRegex(R"("icon":"(\w+))");
        std::regex tempRegex(R"("temp":([\d.]+))");

        std::smatch match;

        if (std::regex_search(readBuffer, match, iconRegex)) {
            icon = match[1];
        }

        if (std::regex_search(readBuffer, match, tempRegex)) {
            temp = std::stod(match[1]);
        }

    
        weather_icon_surface = weatherIconPath[icon];
        temperature = temp;

    }

    // To remove warnings
    if (res) {

    }
}

void MapData::loadStreetSegmentInfo () {
    double max_x = x_from_lon(locationInfo.max_lon);
    double max_y = y_from_lat(locationInfo.max_lat);
    double min_x = x_from_lon(locationInfo.min_lon);
    double min_y = y_from_lat(locationInfo.min_lat);

    // Set max speed
    maxSpeed = -9999;

    // std::cout << "lonmax = " << max_x << "latmax = " << max_y << "lonmin = " << min_x << "latmin = " << min_y << std::endl;

    int colNum = abs(max_x - min_x)/zoneWidth;
    int rowNum = abs(max_y - min_y)/zoneWidth;
    
    // std::cout << "the col_max = " << colNum << " the row_max = " << rowNum << std::endl;

    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<StreetSegmentIdx>> col;
        highwaySSegments.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<StreetSegmentIdx> row;
            highwaySSegments[i].push_back(row);
        }
    }

    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<StreetSegmentIdx>> col;
        largeSSegments.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<StreetSegmentIdx> row;
            largeSSegments[i].push_back(row);
        }
    }

    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<StreetSegmentIdx>> col;
        mediumSSegments.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<StreetSegmentIdx> row;
            mediumSSegments[i].push_back(row);
        }
    }

    for (int i = 0; i <= colNum; i++){
        std::vector<std::vector<StreetSegmentIdx>> col;
        smallSSegments.push_back(col);
        for (int j = 0; j <= rowNum; j++){
            std::vector<StreetSegmentIdx> row;
            smallSSegments[i].push_back(row);
        }
    }

    for (int ssIdx = 0; ssIdx < getNumStreetSegments(); ssIdx++) {
        const StreetSegmentInfo &ogInfo = getStreetSegmentInfo(ssIdx);

        SegmentInfo info;
        info.wayOSMID = ogInfo.wayOSMID;
        info.from = ogInfo.from;
        info.to = ogInfo.to;
        info.oneWay = ogInfo.oneWay;
        info.numCurvePoints = ogInfo.numCurvePoints;
        info.speedLimit = ogInfo.speedLimit;
        info.streetID = ogInfo.streetID;

        // Update max speed
        maxSpeed = maxSpeed < info.speedLimit ? info.speedLimit : maxSpeed;
        
        // More info
        info.streetName = getStreetName(info.streetID);
        
        // Organize the streets segments by size
        info.roadType = getOSMEntityTagValue(info.wayOSMID, "highway");
        if (streetSizeMap.find(info.roadType) == streetSizeMap.end()) {
            info.roadType = "default";
        }
        int streetSize = streetSizeMap[info.roadType];

        // INSERT STREET ATTRIBUTES
        info.attributes = StreetAttributesTable[info.roadType];

        LatLon latLon = LatLon((intersectionInfoTable[info.from].position.latitude() + intersectionInfoTable[info.to].position.latitude()) / 2.0, 
                                (intersectionInfoTable[info.from].position.longitude() + intersectionInfoTable[info.to].position.longitude()) / 2.0);
        int col = abs(min_x - x_from_lon(latLon.longitude())) / zoneWidth;
        int row = abs(min_y - y_from_lat(latLon.latitude())) / zoneWidth;

        // std::cout << col << " " << row << std::endl;

        switch (streetSize) {
        case 1:
            smallSSegments[col][row].push_back(ssIdx);
            break;
        case 2:
            mediumSSegments[col][row].push_back(ssIdx);
            break;
        case 3:
            largeSSegments[col][row].push_back(ssIdx);
            break;   
        case 4:
            highwaySSegments[col][row].push_back(ssIdx);
            break; 
        default:
            smallSSegments[col][row].push_back(ssIdx);
            break;
        }
        
        streetSegmentTable.push_back(info);
    }


}

void MapData::loadStreetInfo () {
    for (int ssIdx = 0; ssIdx < getNumStreetSegments(); ssIdx++) {
        SegmentInfo &segmentInfo = streetSegmentTable[ssIdx];



        // Check if Street exists
        if (streetInfoTable.find(segmentInfo.streetID) == streetInfoTable.end()) {
            std::vector<StreetSegmentIdx> _segments = {ssIdx};
            std::set<IntersectionIdx> _intersections = {segmentInfo.to, segmentInfo.from};

            SegIntPair _pair = {_segments, _intersections};
            
            // Insert the data
            streetInfoTable.insert({segmentInfo.streetID, _pair});
        } else { // Street exists in table
            
            SegIntPair &_pair = streetInfoTable[segmentInfo.streetID];

            // Insert the data
            _pair.first.push_back(ssIdx);
            _pair.second.insert(segmentInfo.to);
            _pair.second.insert(segmentInfo.from);
        }

        // Fill the partial name table

        // Get the name of the current street and transform it to lowercase without spaces
        std::string name = getStreetName(segmentInfo.streetID);
        std::transform(name.begin(), name.end(),name.begin(), ::tolower);
        name.erase(std::remove_if(name.begin(), name.end(), ::isspace),name.end());

        // Get the first two characters, which will be the key for the hash table
        std::string partialName = name.substr(0, 2);
               
        
        // If the key doesn't exists, create one
        if (partialNameTable.find(partialName) == partialNameTable.end()) {

            // Vector that stores a pair of street id and street name
            StreetPairs streetPairs;
            streetPairs.push_back({segmentInfo.streetID, name});    

            partialNameTable.insert({partialName, streetPairs});
        } else { // Key already exists
            partialNameTable[partialName].push_back({segmentInfo.streetID, name});
        }

    }
}


std::vector<IntersectionIdx> MapData::loadFindAdjacentIntersections(IntersectionIdx intersection_id){

    std::unordered_set<IntersectionIdx> intersectionIdxs;
    std::vector<StreetSegmentIdx> &streetSegmentIdxs = intersectionInfoTable[intersection_id].streetSegments;
    std::unordered_map<IntersectionIdx, std::vector<StreetSegmentIdx>> &connectingSegments = intersectionInfoTable[intersection_id].connectingSegments ;


    for (StreetSegmentIdx& streetSegmentIdx: streetSegmentIdxs) {
        SegmentInfo &segmentInfo = streetSegmentTable[streetSegmentIdx];

        // Handle one-way segements
        if (segmentInfo.oneWay && segmentInfo.to == intersection_id) {
            continue;
        } else {

            // Find if "to" is the new intersection to add or if "from" is
            IntersectionIdx addIdx = segmentInfo.from == intersection_id ? segmentInfo.to : segmentInfo.from;

            intersectionIdxs.insert(addIdx);
            // connectingSegments.insert({addIdx, streetSegmentIdx});

            // Add the shortest segment connecting the intersections
            if (connectingSegments.find(addIdx) == connectingSegments.end()) {
                connectingSegments.insert({addIdx, {streetSegmentIdx}});
            } else {
                // SegmentInfo &curSegment = streetSegmentTable[connectingSegments[addIdx]];
                // SegmentInfo &newSegment = streetSegmentTable[streetSegmentIdx];

                // if (newSegment.travelTime < curSegment.travelTime) {
                std::vector<StreetSegmentIdx> &segments = connectingSegments[addIdx];
                // connectingSegments.insert({addIdx, streetSegmentIdx});
                segments.push_back(streetSegmentIdx);
                // }
            }
        }
    }
    
    return std::vector<IntersectionIdx>(intersectionIdxs.begin(), intersectionIdxs.end());
}



void MapData::loadIntersectionData () {
    // Set the min and max latitude and longitudes
    locationInfo.min_lat = 99999;
    locationInfo.min_lon = 99999;
    locationInfo.max_lat = -99999;
    locationInfo.max_lon = -99999;


    for (int i = 0; i < getNumIntersections(); i++) {
        IntersectionInfo intersectionInfo;
        intersectionInfo.name = getIntersectionName(i);
        intersectionInfo.position = getIntersectionPosition(i);
        intersectionInfo.osmNodeId = getIntersectionOSMNodeID(i);
        intersectionInfo.numStreetSegments = getNumIntersectionStreetSegment(i);
        intersectionInfo.highlight = false;

        std::vector<StreetSegmentIdx> &streetSegmentIdxs = intersectionInfo.streetSegments;
        for (int j = 0; j < intersectionInfo.numStreetSegments; j++) {
            streetSegmentIdxs.push_back(getIntersectionStreetSegment(i, j));
        }

        intersectionInfoTable.push_back(intersectionInfo);

        // Find the min and max latitude and longitude
        double lat = intersectionInfo.position.latitude();
        double lon = intersectionInfo.position.longitude();


        locationInfo.min_lat = lat < locationInfo.min_lat ? lat : locationInfo.min_lat;
        locationInfo.min_lon = lon < locationInfo.min_lon ? lon : locationInfo.min_lon;

        locationInfo.max_lat = lat > locationInfo.max_lat ? lat : locationInfo.max_lat;
        locationInfo.max_lon = lon > locationInfo.max_lon ? lon : locationInfo.max_lon;

    }

}

double MapData::loadFindDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){
    double distance;
    
    //this is the avg lat used to compute distance
    const double cosLatAvg = cos(((point_1.latitude() + point_2.latitude())/2) * kDegreeToRadian);

    //convert to Cartesian coordinates with the formula
    double y2 = kEarthRadiusInMeters * point_2.latitude() * kDegreeToRadian;
    double y1 = kEarthRadiusInMeters * point_1.latitude() * kDegreeToRadian;
    double x2 = kEarthRadiusInMeters * point_2.longitude() * kDegreeToRadian * cosLatAvg;
    double x1 = kEarthRadiusInMeters * point_1.longitude() * kDegreeToRadian * cosLatAvg;

    double y2y1 = y2 - y1;
    double x2x1 = x2 - x1;

    distance = sqrt((y2y1)*(y2y1) + (x2x1)*(x2x1));

    return distance;  

}

double MapData::loadFindStreetSegmentLength(StreetSegmentIdx street_segment_id){
    double length;

    //get the from position, to position, and number of curve points
    LatLon from = getIntersectionPosition(getStreetSegmentInfo(street_segment_id).from);
    LatLon to = getIntersectionPosition(getStreetSegmentInfo(street_segment_id).to);
    int numCP = getStreetSegmentInfo(street_segment_id).numCurvePoints;

    //if no curve points, calculate distance between from and to
    if(numCP == 0){
        length = loadFindDistanceBetweenTwoPoints(from, to);
    }
    //if there are curve points, calculate sum of distances between each point
    else{
        length = loadFindDistanceBetweenTwoPoints(from, getStreetSegmentCurvePoint(street_segment_id, 0));
        for(int i = 0; i < numCP - 1; i++){
            length += loadFindDistanceBetweenTwoPoints(
                        getStreetSegmentCurvePoint(street_segment_id, i), 
                        getStreetSegmentCurvePoint(street_segment_id, i+1));
        }
        length += loadFindDistanceBetweenTwoPoints(
                        getStreetSegmentCurvePoint(street_segment_id, numCP - 1), to);
    }

    return length;

}

void MapData::loadStreetSegmentVector(){
    //loop from 0 to getNumStreetSegments - 1
    //fill the vector with info from each segment
    for(int idx = 0; idx < getNumStreetSegments(); idx++){

        float distance = loadFindStreetSegmentLength(idx);
        float speed = getStreetSegmentInfo(idx).speedLimit;
        float time = distance / speed;

        streetsegment temp = {distance, speed, time};
        streetSegmentVector.push_back(temp);
    }
    return;
}

void MapData::fillOSMIDTable(){
    //loop from 0 to getNumNodes - 1 
    //store each node in the hash table with its OSMID as the key
    for(int i = 0; i < getNumberOfNodes(); i++){
        const OSMNode* e = getNodeByIndex(i);
        eTable.insert( {e->id(),std::make_pair(i, static_cast<const OSMEntity*>(e))} );
    }

    for(int i = 0; i < getNumberOfWays(); i++){
        const OSMWay* e = getWayByIndex(i);
        eTable.insert( {e->id(),std::make_pair(i, static_cast<const OSMEntity*>(e))} );
    }



    return;
}

std::string MapData::getOSMEntityTagValue (OSMID OSMid, std::string key){

    std::string value;

    auto it = eTable.find(OSMid);//get the OSMNode correpsonding to the OSMID

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

double MapData::x_from_lon(float lon) {
    return kEarthRadiusInMeters * lon * kDegreeToRadian * cos(((locationInfo.max_lat + locationInfo.min_lat) / 2.0 )* kDegreeToRadian);
}

double MapData::y_from_lat(float lat) {
    return kEarthRadiusInMeters * lat * kDegreeToRadian;
}

float MapData::lat_from_y(double y){
    return y / (kEarthRadiusInMeters * kDegreeToRadian);
}

float MapData::lon_from_x(double x){
    return x / (kEarthRadiusInMeters * kDegreeToRadian * cos(((locationInfo.max_lat + locationInfo.min_lat) / 2.0 )* kDegreeToRadian));
}


MapData::~MapData() {

    // Clear datastructures
    intersectionInfoTable.clear();
    streetSegmentTable.clear();
    streetInfoTable.clear();
    eTable.clear();
    streetSegmentVector.clear();
    partialNameTable.clear();
    buildingZone.clear();
    smallFeatures.clear();
    mediumFeatures.clear();
    largeFeatures.clear();
    xLargeFeatures.clear();
    openFeatures.clear();
    smallSSegments.clear();
    mediumSSegments.clear();
    largeSSegments.clear();
    highwaySSegments.clear();
    poiZone.clear();
    heatmapData.clear();
    poiType.clear();
    weatherSurface.clear();

    // Free POI surfaces
    for(auto it = surfacePaths.begin(); it != surfacePaths.end(); it++){
        ezgl::renderer *g;
        g->free_surface(it->second);
    }

    // Free weather surfaces
    for(auto it = weatherIconPath.begin(); it != weatherIconPath.end(); it++){
        ezgl::renderer *g;
        g->free_surface(it->second);
    }
    

}

std::unordered_map <std::string, std::string> CityPaths = {
    {"Toronto", "/nfs/ug/fast1/ece297s/.public_v366/maps/toronto_canada.streets.bin"},
    {"New York", "/nfs/ug/fast1/ece297s/.public_v366/maps/new-york_usa.streets.bin"},
    {"Tokyo", "/nfs/ug/fast1/ece297s/.public_v366/maps/tokyo_japan.streets.bin"},
    {"Saint Helena", "/nfs/ug/fast1/ece297s/.public_v366/maps/saint-helena.streets.bin"},
    {"Beijing", "/nfs/ug/fast1/ece297s/.public_v366/maps/beijing_china.streets.bin"},
    {"Cairo", "/nfs/ug/fast1/ece297s/.public_v366/maps/cairo_egypt.streets.bin"},
    {"Cape Town", "/nfs/ug/fast1/ece297s/.public_v366/maps/cape-town_south-africa.streets.bin"},
    {"Golden Horseshoe", "/nfs/ug/fast1/ece297s/.public_v366/maps/golden-horseshoe_canada.streets.bin"},
    {"Hamilton", "/nfs/ug/fast1/ece297s/.public_v366/maps/hamilton_canada.streets.bin"},
    {"Hong Kong", "/nfs/ug/fast1/ece297s/.public_v366/maps/hong-kong_china.streets.bin"},
    {"Iceland", "/nfs/ug/fast1/ece297s/.public_v366/maps/iceland.streets.bin"},
    {"Interlaken", "/nfs/ug/fast1/ece297s/.public_v366/maps/interlaken_switzerland.streets.bin"},
    {"Kyiv", "/nfs/ug/fast1/ece297s/.public_v366/maps/kyiv_ukraine.streets.bin"},
    {"London", "/nfs/ug/fast1/ece297s/.public_v366/maps/london_england.streets.bin"},
    {"New Delhi", "/nfs/ug/fast1/ece297s/.public_v366/maps/new-delhi_india.streets.bin"},
    {"Rio De Janerio", "/nfs/ug/fast1/ece297s/.public_v366/maps/rio-de-janeiro_brazil.streets.bin"},
    {"Singapore", "/nfs/ug/fast1/ece297s/.public_v366/maps/singapore.streets.bin"},
    {"Tehran", "/nfs/ug/fast1/ece297s/.public_v366/maps/tehran_iran.streets.bin"},
};


