#include "pathfinding.h"
#include "loadFunctions.h"
#include <queue>
#include <list>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"


#include <omp.h>
#include <stdlib.h>
#include <time.h> 
#include <cmath>
#include <random>

// Globals
std::vector<IntersectionIdx> locations;
std::unordered_map<IntersectionIdx, int> locationsMapping;
std::vector<std::vector<SubPathInfo>> SubPathMatrix;
MapData* currentMap;
std::chrono::high_resolution_clock::time_point globalStart;

std::unordered_map<IntersectionIdx, int> pickupIdxs;
std::unordered_map<IntersectionIdx, int> dropoffIdxs;


#define INITIAL_TEMP 10
#define ALPHA 1

// Data structure to verify route
std::vector<DeliveryInf> deliveryList = {};

// Keep track of the best route
std::vector<IntersectionIdx> bestRoute;

void computeMatrix (const std::vector<DeliveryInf>& deliveries, const double turn_penalty){

    // Store all pickup and dropoff locations (no duplicates)
    std::set<IntersectionIdx> locs;
    int totalCount = 0;
    for(int i = 0; i < deliveries.size(); i++){
        locs.insert(deliveries[i].pickUp);
        locs.insert(deliveries[i].dropOff);

        pickupIdxs.insert({deliveries[i].pickUp, 1});
        dropoffIdxs.insert({deliveries[i].dropOff, 1});


        totalCount+=2;
    }

    // Replace contents of current global locations with new set
    locations.clear();
    locations.assign(locs.begin(), locs.end());

    // Get the mapping between intersections and the index in "locations"
    locationsMapping.clear();
    for (int idx = 0; idx < locations.size(); idx++) {
        locationsMapping.insert({locations[idx], idx});
    }



    // Resize the SubPathMatrix for pre-computations
    SubPathMatrix.resize(locations.size());

    // Compute travel time matrix
    #pragma omp parallel for
    for(int row = 0; row < locations.size(); row++){
        IntersectionIdx srcIntersection = locations[row];

        findPathBetweenIntersectionsMulti(
            {srcIntersection, locations},
            turn_penalty,
            SubPathMatrix,
            row
        );
    }


    // std::cout << SubPathMatrix[0][1].time << std::endl;  

}


bool SubPathInfo::operator() (const SubPathInfo &a, const SubPathInfo &b) { 
        return a.time > b.time;  // Sort in descending
}


std::vector<CourierSubPath> travelingCourier( const std::vector<DeliveryInf>& deliveries,
                                                const std::vector<IntersectionIdx>& depots,
                                                    const float turn_penalty){
   
    std::vector<CourierSubPath> paths = {};
    
    srand (time(NULL));

    //set time for clock-out
    globalStart= std::chrono::high_resolution_clock::now();

   
    // Set the global delivery pointer
    deliveryList.clear();
    deliveryList = deliveries;

    // Clear current subpath matrix
    SubPathMatrix.clear();
    // Pre-compute the subpaths for all intersections
    computeMatrix(deliveries, turn_penalty);

    // Get the current map obejct
    currentMap = getCurrentMap();

    // Dummy route

    // first is to multi-start with different start amd end points
    int numSplits = locations.size(); // Maximum of 8 threads

    // Vector to store the initial group
    std::vector<RouteInfo> initialRouteList = {};
    initialRouteList.resize(numSplits);
    
    #pragma omp parallel for
    for (int num = 0; num < numSplits; num++) {
        
        // now each of these multi-starts implements

        std::vector<IntersectionIdx> curRoute = {}; 
        
        // Get the first random pickup intersection (MAKE SURE IT IS VALID)
        IntersectionIdx startIntersection;
        while (true) {
            startIntersection = deliveries[getRandIdx(deliveries.size() - 1)].pickUp;
            curRoute.push_back(startIntersection);

            if (checkRouteValid(curRoute)) {
                break;
            } else {
                curRoute.pop_back();
            }
        }
         

        // Create a valid path based on a greedy approach
        while (curRoute.size() < locations.size()) {
            // Get the nodes closest to the node you are currently on in the route
            std::vector<SubPathInfo> remainingNodes = SubPathMatrix[locationsMapping[curRoute.back()]];

            // Sort greedily
            std::sort(remainingNodes.begin(), remainingNodes.end(), 
                [](const SubPathInfo &a, const SubPathInfo &b) -> bool{ 
                    return a.time > b.time;  // Sort in descending
                });
            // Remove the last node, which is the same as the node you are on currently
            remainingNodes.pop_back();
            

            /*
                Randomize for multi-start
            */
            // 90% chance of getting the closest, 10% chance of getting the second best
            bool secondBest = getRandIdx(10) + 1 > 8 ? true : false;
            // Handle the case when only one index remaining
            if (remainingNodes.size() == 1) secondBest = false;
            // 90% get the last, 10% get the second last
            int curIndex = secondBest ? remainingNodes.size() - 2 : remainingNodes.size() - 1; 
            
            SubPathInfo curNode = remainingNodes[curIndex];

            // The intersection at this node
            IntersectionIdx curIntersection = curNode.idx;

            // Only add this node if it does not exist already
            bool exists = std::find(curRoute.begin(), curRoute.end(), curIntersection) != curRoute.end();

            curRoute.push_back(curIntersection);
            // Check if the new route is still valid
            if(!exists && checkRouteValid(curRoute)) {
                if (secondBest) {
                    remainingNodes.erase(remainingNodes.begin() + curIndex);
                } else {
                    remainingNodes.pop_back(); // Remove it from the remaining nodes
                }
            }
            else { // If route is not valid, find the next best node that makes a valid route
                curRoute.pop_back();

                bool found = false;

                /*
                Randomize for multi-start
                */
                // 90% chance of getting the closest, 10% chance of getting the second best
                bool secondBest_ = getRandIdx(10) + 1 > 8 ? true : false;
                // Handle the case when only one index remaining
                if (remainingNodes.size() < 3) secondBest_ = false;
                // 90% get the last, 10% get the second last
                int index = secondBest_ ? remainingNodes.size() - 3 : remainingNodes.size() - 2;

                while (!found && index >= 0) {
                    
                    SubPathInfo &curLastNode = remainingNodes[index];

                    // The intersection is the last intersection of the subpath
                    IntersectionIdx curLastIntersection = curLastNode.idx;

                    // Only add this node if it does not exist already
                    bool stillExists = std::find(curRoute.begin(), curRoute.end(), curLastIntersection) != curRoute.end();

                    curRoute.push_back(curLastIntersection);
                
                    // Next best, valid node
                    if (!stillExists && checkRouteValid(curRoute)) {
                        found = true;
                        remainingNodes.erase(remainingNodes.begin() + index);
                    } else { // keep finding
                        curRoute.pop_back();
                        index--;
                    }
                }
            }
        }


        if (curRoute.size() != locations.size()) {
            std::cout << "NOT THE SAME SIZE" << std::endl;
        }

        // Get the route length
        double time = computeRouteTime(curRoute);

        // Add to the initial group
        initialRouteList[num] = {time, curRoute};
    }


    // Do iterative 3 opt to better initial conditions
     while(deliveryList.size() != 1) { 

        // get current time and duration from start
        auto current = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = (current - globalStart);
        // std::cout << "Ran" << std::endl;



        if(duration.count() > GLOBAL_TIMEOUT * 0.75) {
            std::cout << "TIME:::::: " << duration.count() << std::endl;
            break;
        }

        #pragma omp parallel for
        for (int index = 0; index < initialRouteList.size(); index++) {
            twoOptSingle(initialRouteList[index]);
        }
     }

    // Find the best route from the initial group
    std::sort(initialRouteList.begin(), initialRouteList.end(), 
        [](const RouteInfo &a, const RouteInfo &b) -> bool{ 
            return a.routeTime < b.routeTime;  // Sort in ascending
        });

    std::cout << "Best score after iterative:: " << initialRouteList[0].routeTime << std::endl;

    



    
    // ece297exercise 4 --run_tester  M4_Public_Simple_Legality_Toronto
    // ece297exercise 4 --run_tester  M4_Public_Extreme_Toronto

    auto mutationStart = std::chrono::high_resolution_clock::now();
    int genCount = 0;

    while(deliveryList.size() != 1) { 

        // get current time and duration from start
        auto current = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = (current - globalStart);
        // std::cout << "Ran" << std::endl;



        if(duration.count() > GLOBAL_TIMEOUT) {
            std::cout << "TIME:::::: " << duration.count() << std::endl;
            break;
        }
        genCount++;



        // Filled in by the mutation step
        std::vector<RouteInfo> mainPool = initialRouteList;
        std::vector<std::vector<RouteInfo>> subPool = {};

        subPool.resize(initialRouteList.size());
        for (int i = 0; i < initialRouteList.size(); i++) {
            subPool[i] = {};
        }
        // subPool[0] = initialRouteList;

        // Perform crossover to get children from parents
        // #pragma omp parallel for
        // for (int count = 0; count < initialRouteList.size(); count++) {

        //     // int idxA = getRandIdx(initialRouteList.size());
        //     // int idxB = idxA;

        //     // while (idxB == idxA) {
        //     //     idxB = getRandIdx(initialRouteList.size());
        //     // }

        //     // // Handle edge cases
        //     // if (idxA == 0) idxA++;
        //     // else if (idxA == initialRouteList.size() - 1) idxA--;

        //     // if (idxB == 0) idxB++;
        //     // else if (idxB == initialRouteList.size() - 1) idxB--;


        //     // orderCrossover(initialRouteList[idxA], initialRouteList[idxB]);
        //     // optimizeElite(initialRouteList[count]);
        // }

        // std::cout << "done crossover" << std::endl;

        // Mutate children
        #pragma omp parallel for
        for (int count = 0; count < initialRouteList.size(); count++) {
            twoOpt(initialRouteList[count], subPool[count]);
        }

        // std::cout << "done twoOpt" << std::endl;

        
        // Concatenate all the subpools into main pool
        for (auto sub: subPool) {
            mainPool.insert(mainPool.end(), sub.begin(), sub.end());
        }

        // std::cout << "Mutated Pool size:: " << mainPool.size() << std::endl;

        if (mainPool.size() == 0) break;

        // Sort the mutated pool
        std::sort(
            mainPool.begin(), mainPool.end(),
            [](const RouteInfo &a, const RouteInfo &b) -> bool{ 
                    return a.routeTime < b.routeTime;  // Sort in ascending
            }
        );


        // Retain elite

        int numExtend = numSplits < mainPool.size() ? numSplits : mainPool.size();
        int numBadExtend = numExtend / 2;
        // current = std::chrono::high_resolution_clock::now();

        // std::chrono::duration<double> indepVariable = (current - mutationStart);
        // int numBadExtend = numExtend / indepVariable.count() * 2;
        // if (numBadExtend > numExtend / 2) ;
        numExtend -= numBadExtend;

        initialRouteList.clear();
        initialRouteList.insert(initialRouteList.end(), mainPool.begin(), mainPool.begin() + numExtend);

        int countBad = 0;
        while (countBad < numBadExtend) {
            int idx = getRandIdx(mainPool.size() - numExtend) + numExtend;
            // if (idx >= mainPool.size()) idx = mainPool.size() - 1;
            
            initialRouteList.push_back(mainPool[idx]);
            
            countBad++;
        }



    }
    std::cout << "GENERATION: " << genCount << std::endl;


    // The best route is the first one in the initialRouteList
    // bestRoute = shortestRoute.route;
    bestRoute = initialRouteList[0].route;
    std::cout << "Best score after GA:: " << initialRouteList[0].routeTime << std::endl;

    // for (int i = 0; i < bestRoute.size(); i++) {
    //     std::cout << bestRoute[i] << " ";
    // }

    for(int i = 0; i < bestRoute.size() - 1; i++){

        CourierSubPath current;
        current.start_intersection = bestRoute[i];
        current.end_intersection = bestRoute[i + 1];

        int row = locationsMapping[current.start_intersection];
        int col = locationsMapping[current.end_intersection];

        current.subpath = SubPathMatrix[row][col].subpath;
        
        paths.push_back(current);

    }

    // Find the closest depot to the starting intersection (Euclidean)
    IntersectionIdx closestDepot = -1;
    double smallestDistance = std::numeric_limits<double>::infinity();
    IntersectionIdx firstIntersection = bestRoute[0];
    LatLon firstIntersectionLoc = currentMap->intersectionInfoTable[firstIntersection].position;
    for (IntersectionIdx curDepot: depots) {
        
        LatLon depotLoc = currentMap->intersectionInfoTable[curDepot].position;

        double distance = findDistanceBetweenTwoPoints(depotLoc, firstIntersectionLoc);

        if (distance < smallestDistance) {
            smallestDistance = distance;
            closestDepot = curDepot;
        }

    }


    // Find the paths to the depots
    std::vector<StreetSegmentIdx> startingDepotPath = findPathBetweenIntersections({closestDepot, firstIntersection}, turn_penalty);

    IntersectionIdx lastDepot = -1;
    std::vector<StreetSegmentIdx> endingDepotPath = findPathToDepot({bestRoute.back(), depots}, turn_penalty, lastDepot);   

    // Add depots
    CourierSubPath startingDepot = {
        closestDepot, // First intersection in subpath is the depot
        firstIntersection,
        startingDepotPath
    };

    paths.insert(paths.begin(), startingDepot);

    // for (int i = 0; i < endingDepotPath.size(); i++) {
    //     std::cout << endingDepotPath[i] << " ";
    // }

    CourierSubPath endingDepot = {
        bestRoute.back(), 
        lastDepot, // Last intersection in subpath is the depot
        endingDepotPath
    };

    paths.push_back(endingDepot);

    // Clear all globals
    locations.clear();
    locationsMapping.clear();
    SubPathMatrix.clear();
    deliveryList.clear();
    bestRoute.clear();
    pickupIdxs.clear();
    dropoffIdxs.clear();
    currentMap = nullptr;

    // std::cout << "EEEENNNNDDDINNG!!!!" << std::endl;

    return paths;
}

double updateTemperature(double x) {

    return (INITIAL_TEMP / ( 1 + ALPHA * x*x));

}


int getRandIdx(int size) {
    if (size == 0) return 0;

    int idx = rand() % size;
    return idx;
}


bool checkRouteValid(const std::vector<IntersectionIdx> &route) {

    std::unordered_map <IntersectionIdx, int> mapping;

    // Generate index mapping
    for (int idx = 0; idx < route.size(); idx++) {
        mapping.insert({route[idx], idx});
    }

    // If the dropoff is happening before the pickup, return false
    for (DeliveryInf delivery: deliveryList) {
        if (mapping.find(delivery.dropOff) != mapping.end() // If dropoff and pickup exist, do the check
                && mapping.find(delivery.pickUp) != mapping.end()) {
            if (mapping[delivery.dropOff] < mapping[delivery.pickUp]) {
                return false;
            }
        } else if (mapping.find(delivery.dropOff) != mapping.end() // If dropoff exists and pickup does not, false
                    && mapping.find(delivery.pickUp) == mapping.end()) { 

                return false;
        }
    }

    return true;
}

bool checkSubRouteValid(const std::vector<IntersectionIdx>::iterator &subRoute, int length) {

    std::unordered_map <IntersectionIdx, int> mapping;

    // Generate index mapping
    for (int idx = 0; idx < length; idx++) {
        mapping.insert({*(subRoute + idx), idx});
    }

    // If the dropoff is happening before the pickup, return false
    for (DeliveryInf delivery: deliveryList) {
        if (mapping.find(delivery.dropOff) != mapping.end() // If dropoff and pickup exist, do the check
                && mapping.find(delivery.pickUp) != mapping.end()) {
            if (mapping[delivery.dropOff] < mapping[delivery.pickUp]) {
                return false;
            }
        }
    }

    return true;
}

// Function that takes in a route and computes how long it takes
double computeRouteTime(std::vector<IntersectionIdx> &route) {

    double routeTime = 0;

    for (int i = 0; i < route.size() - 1; i++) {

        IntersectionIdx intersectionAIdx = locationsMapping[route[i]];
        IntersectionIdx intersectionBIdx = locationsMapping[route[i + 1]];

        routeTime += SubPathMatrix[intersectionAIdx][intersectionBIdx].time;

        // // Special heuristic
        // if (pickupIdxs.find(intersectionAIdx) != pickupIdxs.end()) {

        //     routeTime += 1000 * i;
        // }
    }

    return routeTime;
}

void threeOptcomputeRouteTime(RouteInfo &originalRoute, RouteInfo &newRoute, std::pair<int, int> originalIndexes, std::pair<int, int> newIndexes) {

    // Make time same as old time
    double routeTime = originalRoute.routeTime;

    // Subtracting the breaks in the route time
    int rowOldRoute1 = locationsMapping[originalRoute.route[originalIndexes.first]];
    int colOldRoutePrevious1 = locationsMapping[originalRoute.route[originalIndexes.first - 1]];


    int rowOldRoute2 = locationsMapping[originalRoute.route[originalIndexes.second]];
    int colOldRoutePrevious2 = locationsMapping[originalRoute.route[originalIndexes.second - 1]];

    routeTime = routeTime - SubPathMatrix[colOldRoutePrevious1][rowOldRoute1].time  - SubPathMatrix[colOldRoutePrevious2][rowOldRoute2].time;
    // std::cout << "removed route time: " << routeTime << std::endl;


    // Adding new route times
    int rowNewRoute1 = locationsMapping[newRoute.route[newIndexes.first]];
    int colNewRoutePrevious1 = locationsMapping[newRoute.route[newIndexes.first - 1]];


    int rowNewRoute2 = locationsMapping[newRoute.route[newIndexes.second]];
    int colNewRoutePrevious2 = locationsMapping[newRoute.route[newIndexes.second - 1]];

    routeTime = routeTime + SubPathMatrix[colNewRoutePrevious1][rowNewRoute1].time  + SubPathMatrix[colNewRoutePrevious2][rowNewRoute2].time;

    newRoute.routeTime = routeTime;
    std::cout << "new route time: " << newRoute.routeTime << std::endl;
}

void twoOptSingle(RouteInfo &parentRoute) {

    std::vector<IntersectionIdx> curRoute = parentRoute.route;
    double costCur = computeRouteTime(curRoute);

    int foundCount = 0;
    int maxIters = curRoute.size() * 2;
    bool found = false;

    while (!found && foundCount < maxIters) {

        int idxA = getRandIdx(curRoute.size());
        int idxB = idxA;

        while (idxB == idxA) {
            idxB = getRandIdx(curRoute.size());
        }

        // Handle edge cases
        if (idxA == 0) idxA++;
        else if (idxA == curRoute.size() - 1) idxA--;

        if (idxB == 0) idxB++;
        else if (idxB == curRoute.size() - 1) idxB--;
        

        // Index A is always lowest
        if (idxB < idxA) {
            std::swap(idxA, idxB);
        }

        // Randomly reverse a segment
        int reverseSegment = getRandIdx(3);
        std::vector<IntersectionIdx> buildRoute = {};

        switch (reverseSegment) {

            case 0: {// Reverse the first segment

                std::vector<IntersectionIdx> firstReversed (curRoute.begin(), curRoute.begin() + idxA);
                std::reverse(firstReversed.begin(), firstReversed.end());

                buildRoute.insert(buildRoute.end(), firstReversed.begin(), firstReversed.end());
                buildRoute.insert(buildRoute.end(), curRoute.begin() + idxA, curRoute.end());

                break;
            }

            case 1: {// Reverse the middle segment
                
                buildRoute.insert(buildRoute.end(), curRoute.begin(), curRoute.begin() + idxA);
                
                std::vector<IntersectionIdx> middleReversed (curRoute.begin() + idxA, curRoute.begin() + idxB);
                std::reverse(middleReversed.begin(), middleReversed.end());

                buildRoute.insert(buildRoute.end(), middleReversed.begin(), middleReversed.end());
                buildRoute.insert(buildRoute.end(), curRoute.begin() + idxB, curRoute.end());   

                break;
            }

            case 2: {// Reverse the last segment

                buildRoute.insert(buildRoute.end(), curRoute.begin(), curRoute.begin() + idxB);

                std::vector<IntersectionIdx> lastReversed (curRoute.begin() + idxB, curRoute.end());
                std::reverse(lastReversed.begin(), lastReversed.end());

                buildRoute.insert(buildRoute.end(), lastReversed.begin(), lastReversed.end());

                break;
            }

            default: 
                break;
        }


        double cost = computeRouteTime(buildRoute);
        if (cost < costCur) {

            if (checkRouteValid(buildRoute)) {
                parentRoute = {cost, buildRoute};
                
                found = true;
            }
        }

        foundCount++;
        
    }
}

// void twoOptSingle(RouteInfo &parentRoute) {

//     std::vector<IntersectionIdx> curRoute = parentRoute.route;
//     double costCur = computeRouteTime(curRoute);

//     int foundCount = 0;
//     int maxIters = curRoute.size() * 2;
//     bool found = false;

//     while (!found && foundCount < maxIters) {

//         int idxA = getRandIdx(curRoute.size());
//         int idxB = idxA;

//         while (idxB == idxA) {
//             idxB = getRandIdx(curRoute.size());
//         }

//         // Handle edge cases
//         if (idxA == 0) idxA++;
//         else if (idxA == curRoute.size() - 1) idxA--;

//         if (idxB == 0) idxB++;
//         else if (idxB == curRoute.size() - 1) idxB--;
        

//         // Index A is always lowest
//         if (idxB < idxA) {
//             std::swap(idxA, idxB);
//         }

//         // Randomly reverse a segment
//         std::vector<std::vector<IntersectionIdx>> buildRoute = {};
//         std::vector<IntersectionIdx> buildRouteFull = {};



//         std::vector<IntersectionIdx> firstSeg (curRoute.begin(), curRoute.begin() + idxA);
//         buildRoute.push_back(firstSeg);

//         std::vector<IntersectionIdx> middleSeg(curRoute.begin() + idxA, curRoute.begin() + idxB);
//         buildRoute.push_back(middleSeg);
        
//         std::vector<IntersectionIdx>lastSeg(curRoute.begin() + idxB, curRoute.end());
//         buildRoute.push_back(lastSeg);

                

//         std::vector<int> indices = {0, 1, 2};
//         auto rng = std::default_random_engine {};

//         // Randomize insertion
//         std::shuffle(indices.begin(), indices.end(), rng);

//         buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[0]].begin(), buildRoute[indices[0]].end());
//         buildRouteFull.push_back(curRoute[idxA]);

//         int newIdxA = buildRouteFull.size() - 1;

//         buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[1]].begin(), buildRoute[indices[1]].end());
//         buildRouteFull.push_back(curRoute[idxB]);

//         int newIdxB = buildRouteFull.size() - 1;


//         buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[2]].begin(), buildRoute[indices[2]].end());


//         RouteInfo newRoute = {0, buildRouteFull};
//         // threeOptcomputeRouteTime(parentRoute, newRoute, {idxA, idxB}, {newIdxA, newIdxB});
//         double cost = computeRouteTime(buildRouteFull);
//         if (cost < parentRoute.routeTime) {
//             if (checkRouteValid(buildRouteFull)) {
//                 parentRoute = {cost, buildRouteFull};
//                 // std::cout << "GOT VALID" << std::endl;
//                 found = true;
//             }

//         }

        

//         foundCount++;
//     }
// }

void twoOpt(RouteInfo &parentRoute, std::vector<RouteInfo> &subpool) {

    std::vector<IntersectionIdx> curRoute = parentRoute.route;
    double costCur = computeRouteTime(curRoute);
    //set time for clock-out
    auto start = std::chrono::high_resolution_clock::now();

    int foundCount = 0;

    while (foundCount < curRoute.size()) {

        // get current time and duration from start
        auto current = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current - start);
        auto globalDuration = std::chrono::duration_cast<std::chrono::seconds>(current - globalStart);


        // // stop iterating if more than defined time
        if(duration.count() > TWO_OPT_TIMEOUT || globalDuration.count() > GLOBAL_TIMEOUT) break;

        int idxA = getRandIdx(curRoute.size());
        int idxB = idxA;

        while (idxB == idxA) {
            idxB = getRandIdx(curRoute.size());
        }

        // Handle edge cases
        if (idxA == 0) idxA++;
        else if (idxA == curRoute.size() - 1) idxA--;

        if (idxB == 0) idxB++;
        else if (idxB == curRoute.size() - 1) idxB--;
        

        // Index A is always lowest
        if (idxB < idxA) {
            std::swap(idxA, idxB);
        }

        // Randomly reverse a segment
        int reverseSegment = getRandIdx(3);
        std::vector<IntersectionIdx> buildRoute = {};

        switch (reverseSegment) {

            case 0: {// Reverse the first segment

                std::vector<IntersectionIdx> firstReversed (curRoute.begin(), curRoute.begin() + idxA);
                std::reverse(firstReversed.begin(), firstReversed.end());

                buildRoute.insert(buildRoute.end(), firstReversed.begin(), firstReversed.end());
                buildRoute.insert(buildRoute.end(), curRoute.begin() + idxA, curRoute.end());

                break;
            }

            case 1: {// Reverse the middle segment
                
                buildRoute.insert(buildRoute.end(), curRoute.begin(), curRoute.begin() + idxA);
                
                std::vector<IntersectionIdx> middleReversed (curRoute.begin() + idxA, curRoute.begin() + idxB);
                std::reverse(middleReversed.begin(), middleReversed.end());

                buildRoute.insert(buildRoute.end(), middleReversed.begin(), middleReversed.end());
                buildRoute.insert(buildRoute.end(), curRoute.begin() + idxB, curRoute.end());   

                break;
            }

            case 2: {// Reverse the last segment

                buildRoute.insert(buildRoute.end(), curRoute.begin(), curRoute.begin() + idxB);

                std::vector<IntersectionIdx> lastReversed (curRoute.begin() + idxB, curRoute.end());
                std::reverse(lastReversed.begin(), lastReversed.end());

                buildRoute.insert(buildRoute.end(), lastReversed.begin(), lastReversed.end());

                break;
            }

            default: 
                break;
        }


        double cost = computeRouteTime(buildRoute);

            if (checkRouteValid(buildRoute)) {
                subpool.push_back({cost, buildRoute});

                if (cost < costCur) {
                    parentRoute = {cost, buildRoute};
                }

                foundCount++;
            }


        
    }
}


/*
void twoOpt(RouteInfo &parentRoute, std::vector<RouteInfo> &subpool) {

    std::vector<IntersectionIdx> curRoute = parentRoute.route;
    //set time for clock-out
    auto start = std::chrono::high_resolution_clock::now();

    int iterCount = 0;
    int numIters = curRoute.size();

    while (iterCount < numIters) {

        // get current time and duration from start
        auto current = std::chrono::high_resolution_clock::now();
        auto globalDuration = std::chrono::duration_cast<std::chrono::seconds>(current - globalStart);


        // // stop iterating if more than defined time
        if(globalDuration.count() > GLOBAL_TIMEOUT) break;

        int idxA = getRandIdx(curRoute.size());
        int idxB = idxA;

        while (idxB == idxA) {
            idxB = getRandIdx(curRoute.size());
        }

        // Handle edge cases
        if (idxA == 0) idxA++;
        else if (idxA == curRoute.size() - 1) idxA--;

        if (idxB == 0) idxB++;
        else if (idxB == curRoute.size() - 1) idxB--;
        

        // Index A is always lowest
        if (idxB < idxA) {
            std::swap(idxA, idxB);
        }

        // Randomly reverse a segment
        std::vector<std::vector<IntersectionIdx>> buildRoute = {};
        std::vector<IntersectionIdx> buildRouteFull = {};



        std::vector<IntersectionIdx> firstSeg (curRoute.begin(), curRoute.begin() + idxA);
        buildRoute.push_back(firstSeg);

        std::vector<IntersectionIdx> middleSeg(curRoute.begin() + idxA, curRoute.begin() + idxB);
        buildRoute.push_back(middleSeg);
        
        std::vector<IntersectionIdx>lastSeg(curRoute.begin() + idxB, curRoute.end());
        buildRoute.push_back(lastSeg);

                

        std::vector<int> indices = {0, 1, 2};
        auto rng = std::default_random_engine {};

        // Randomize insertion
        std::shuffle(indices.begin(), indices.end(), rng);

        buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[0]].begin(), buildRoute[indices[0]].end());
        buildRouteFull.push_back(curRoute[idxA]);

        buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[1]].begin(), buildRoute[indices[1]].end());
        buildRouteFull.push_back(curRoute[idxB]);

        buildRouteFull.insert(buildRouteFull.end(), buildRoute[indices[2]].begin(), buildRoute[indices[2]].end());




        if (checkRouteValid(buildRouteFull)) {
            double cost = computeRouteTime(buildRouteFull);
            subpool.push_back({cost, buildRouteFull});
            
            // if (cost < computeRouteTime(buildRouteFull)) {
            //     parentRoute = {cost, buildRouteFull};
            // }

            // std::cout << "GOT VALID" << std::endl;
            iterCount++;

        }
    }
}

*/

void optimizeElite (RouteInfo &elite){

    bool improved = false;
    int maxNumTries = elite.route.size() * 2;
    int numTries = 0;

    while(!improved && numTries < maxNumTries){

        int idx1, idx2;

        idx1 = getRandIdx(elite.route.size());
        idx2 = idx1;
        
        if (getRandIdx(2) > 0) {
            while(idx2 == idx1){
                idx2 = getRandIdx(elite.route.size());
            }
        } else
            idx2 = idx1 + 1 < elite.route.size() ? idx1 + 1: idx1 - 1;


        RouteInfo temp = elite;
        IntersectionIdx tempIdx = temp.route[idx1];
        temp.route[idx1] = temp.route[idx2];
        temp.route[idx2] = tempIdx;

        auto it = temp.route.begin();

        if(idx1 < idx2) it += idx1;
        else it += idx2;
        

        // if(!checkSubRouteValid(it, abs(idx2 - idx1))){
        //     numTries++;
        //     continue;
        // }

        if (!checkRouteValid(temp.route)) {
            numTries++;
            continue;
        }

        double newRouteTime = computeRouteTime(temp.route);

        if(newRouteTime < elite.routeTime){

            temp.routeTime = newRouteTime;
            elite = temp;
            improved = true;
        }
        numTries++;
    } 

}

