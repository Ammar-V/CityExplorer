#include "pathfinding.h"
#include "loadFunctions.h"
#include <queue>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

#include "m1.h"
#include "m3.h"

#define NO_EDGE -1

// Global variables
std::unordered_map<IntersectionIdx, Node> Nodes = {};
std::vector<StreetSegmentIdx> pathSegments = {};


// Default constructor
QueueElem::QueueElem () {
    nodeId = -1;
    reachingEdge = -1;
    travelTime = -1;
}

// Constructor
QueueElem::QueueElem (int node, int edge, double time, double h) {
    nodeId = node;
    reachingEdge = edge;
    travelTime = time;
    heuristic = h;
}

// Overloading for priority queue and implementing A* 
bool QueueElem::operator() (const QueueElem &lhs, const QueueElem &rhs) {
    return lhs.travelTime + lhs.heuristic > rhs.travelTime + rhs.heuristic;
}

// A* algorithm implementation
bool shortestPath (IntersectionIdx src, IntersectionIdx dest, 
                    double time_penalty, MapData* map) {

    // Clear the previous iteration
    Nodes.clear();

    // Initialize priority queue
    std::priority_queue<QueueElem, std::vector<QueueElem>, QueueElem> intersectionQueue;

    // Pre calculate the desitination position for heuristic calculations
    LatLon destPosition = map->intersectionInfoTable[dest].position;

    // Initialize the queue with the source
    QueueElem srcElem = {src, NO_EDGE, 0, 0};
    intersectionQueue.emplace(srcElem);

    bool destFound = false;
    bool atStart = true;

    

    while (!destFound && intersectionQueue.size() != 0) { // and check if all the edges connected to destination exist

        QueueElem curIntersection = intersectionQueue.top();
        intersectionQueue.pop();

        // Node has not been visited yet
        if (Nodes.find(curIntersection.nodeId) == Nodes.end()) { // TODO: This check might be redundant
            Node newNode = {curIntersection.nodeId,
                            true,
                            curIntersection.reachingEdge,
                            curIntersection.travelTime};

            Nodes.insert({curIntersection.nodeId, newNode});

            // Check if this is destination
            if (newNode.nodeId == dest) {
                destFound = true;
                break;
            }

            // Add the adjacent nodes to the priority queue
            IntersectionInfo &info = map->intersectionInfoTable[curIntersection.nodeId];
            std::vector<IntersectionIdx> adjacentNodes = info.adjacentIntersections;


            for (int i = 0; i < adjacentNodes.size(); i++) {
                IntersectionIdx id = adjacentNodes[i]; 
                
                if (Nodes.find(id) != Nodes.end())
                    continue; // Node already has been visited
                
                // Get the segment that connects to the adjacent intersection
                std::vector<StreetSegmentIdx> segmentIdxs = info.connectingSegments[id];
                for (StreetSegmentIdx segmentIdx: segmentIdxs) {
                    double time = map->streetSegmentVector[segmentIdx].time;

                    // Check for turn penalty
                    if (time_penalty != 0.0 && !atStart) {
                        StreetSegmentIdx prevSegmentIdx = newNode.reachingEdge;

                        if (turn_penality_check(prevSegmentIdx, segmentIdx)) {
                            time += time_penalty;
                        }
                    }

                    // Caluculate the heurisitc function
                    double heuristic = calculateHeuristic(id, destPosition, map);

                    QueueElem newElem = {id, segmentIdx, time + newNode.bestTime, heuristic};
                    intersectionQueue.emplace(newElem);
                }
            }

            // Not at the starting intersection anymore
            atStart = false;
        }
    }

    return destFound;
}

// Heuristic Function which uses Euclidean Distance and Max Speed found the city during pre-loading
double calculateHeuristic (IntersectionIdx cur, LatLon destPosition, MapData* currentMap) {
    LatLon curPosition = currentMap->intersectionInfoTable[cur].position;
    return findDistanceBetweenTwoPoints(curPosition, destPosition) / currentMap->maxSpeed;
}

// Finding the stretsegements for the shortest path
std::vector<StreetSegmentIdx> findPathBetweenIntersections (
    const std::pair<IntersectionIdx, IntersectionIdx> intersection_ids,
    const double turn_penalty
) {

    IntersectionIdx srcIntersection = intersection_ids.first;
    IntersectionIdx destIntersection = intersection_ids.second;

    // Clear the previous path
    pathSegments.clear();

    // Get current map to access data structures
    MapData* map = getCurrentMap();

    // Generate the shortest path (consists of nodes)
    bool pathFound = shortestPath(srcIntersection, destIntersection, turn_penalty, map);

    if (!pathFound) // No path found
        return pathSegments;

    // Construct the path from nodes
    IntersectionIdx curNodeId = destIntersection;
    StreetSegmentIdx curSegment = Nodes[destIntersection].reachingEdge;
    
    while(curSegment != NO_EDGE) { // While there are more street segments to add

        // Add the street segment id
        pathSegments.push_back(curSegment);

        // Update the next intersection id from the previous node
        SegmentInfo &edgeInfo = map->streetSegmentTable[curSegment];
        curNodeId = (edgeInfo.to == curNodeId) ? edgeInfo.from : edgeInfo.to;
        curSegment = Nodes[curNodeId].reachingEdge;

    }

    // Reverse the path
    std::reverse(pathSegments.begin(), pathSegments.end());
    return pathSegments;
}

double computePathTravelTime(const std::vector<StreetSegmentIdx>& path, 
                             const double turn_penalty) {

    double totalTravelTime = 0;  
    int i; 

    // Get current map to access data structures
    MapData* map = getCurrentMap();           

    // loop throught street segments and add time
    for(i = 0; i < path.size() - 1; i++){
        totalTravelTime += map->streetSegmentVector[path[i]].time;
        
        // Get current and next StreetSegmentId for curve check
        StreetSegmentIdx segment1 = path[i];
        StreetSegmentIdx segment2 = path[i+1];

        // if curves, add turn penalty
        if (turn_penality_check(segment1, segment2)){
            totalTravelTime += turn_penalty;
        }
    }

    // checks for the last list element travel time
    totalTravelTime += map->streetSegmentVector[path[i]].time;

    return totalTravelTime;
}





// Multi dijkstra's
bool shortestPathMulti (IntersectionIdx src, std::vector<IntersectionIdx> dest, 
                    double time_penalty, MapData* map, std::unordered_map<IntersectionIdx, Node> &localNodes, 
                    bool isDepot, IntersectionIdx &depot) {

    // Clear the previous iteration
    localNodes.clear();

    // Initialize priority queue
    std::priority_queue<QueueElem, std::vector<QueueElem>, QueueElem> intersectionQueue;

    // Pre calculate the desitination position for heuristic calculations
    // LatLon destPosition = map->intersectionInfoTable[dest].position;

    // Initialize the queue with the source
    QueueElem srcElem = {src, NO_EDGE, 0, 0};
    intersectionQueue.emplace(srcElem);

    // Data structure to keep track of which destination has been found
    std::unordered_map<IntersectionIdx, int> foundDest;
    for (auto destination: dest) {
        foundDest.insert({destination, 1});
    } 

    int numDestFound = 0;
    bool atStart = true; 


    while (numDestFound < dest.size() && intersectionQueue.size() != 0) { // and check if all the edges connected to destination exist

        QueueElem curIntersection = intersectionQueue.top();
        intersectionQueue.pop();

        // Node has not been visited yet
        if (localNodes.find(curIntersection.nodeId) == localNodes.end()) { // TODO: This check might be redundant
            Node newNode = {curIntersection.nodeId,
                            true,
                            curIntersection.reachingEdge,
                            curIntersection.travelTime};

            localNodes.insert({curIntersection.nodeId, newNode});

            // Check if this is a destination
            if (foundDest.find(newNode.nodeId) != foundDest.end()) {
                numDestFound++;

                if (isDepot) {
                    depot = newNode.nodeId;
                }

                if (isDepot || numDestFound >= dest.size()) // All the destinations have been found
                    break;
            }

            // Add the adjacent nodes to the priority queue
            IntersectionInfo &info = map->intersectionInfoTable[curIntersection.nodeId];
            std::vector<IntersectionIdx> adjacentNodes = info.adjacentIntersections;


            for (int i = 0; i < adjacentNodes.size(); i++) {
                IntersectionIdx id = adjacentNodes[i]; 
                
                if (localNodes.find(id) != localNodes.end())
                    continue; // Node already has been visited
                
                // Get the segment that connects to the adjacent intersection
                std::vector<StreetSegmentIdx> segmentIdxs = info.connectingSegments[id];
                for (StreetSegmentIdx segmentIdx: segmentIdxs) {
                    double time = map->streetSegmentVector[segmentIdx].time;

                    // Check for turn penalty
                    if (time_penalty != 0.0 && !atStart) {
                        StreetSegmentIdx prevSegmentIdx = newNode.reachingEdge;

                        if (turn_penality_check(prevSegmentIdx, segmentIdx)) {
                            time += time_penalty;
                        }
                    }

                    // Caluculate the heurisitc function
                    //double heuristic = calculateHeuristic(id, destPosition, map);
                    double heuristic = 0;

                    QueueElem newElem = {id, segmentIdx, time + newNode.bestTime, heuristic};
                    intersectionQueue.emplace(newElem);
                }
            }

            // Not at the starting intersection anymore
            atStart = false;
        }
    }

    return (bool) numDestFound;
}


// Finding the stretsegements for the shortest path
bool findPathBetweenIntersectionsMulti (
    const std::pair<IntersectionIdx, std::vector<IntersectionIdx>> intersection_ids,
    const double turn_penalty,
    std::vector<std::vector<SubPathInfo>> &kMatrix, const int kIdx
) {

    IntersectionIdx srcIntersection = intersection_ids.first;
    std::vector<IntersectionIdx> destIntersections = intersection_ids.second;

    // The local graph that is used to store the map nodes traversed in dijkstras
    std::unordered_map<IntersectionIdx, Node> mapNodes;

    // Get current map to access data structures
    MapData* map = getCurrentMap();

    // Generate the shortest path (consists of nodes)
    IntersectionIdx temp = -1; // Parameter not used since not finding depots
    bool pathFound = shortestPathMulti(srcIntersection, destIntersections, turn_penalty, map, mapNodes, false, temp);

    if (!pathFound) // No path found
        return false;

    // Fill in the current row of the kMatrix
    for (IntersectionIdx destIntersection: destIntersections) {

        // If the src and dest are the same, then the entry in the kMatrix should be 0
        if (destIntersection == srcIntersection) {
            kMatrix[kIdx].push_back({destIntersection, 0, {}});
            continue;
        }

        std::vector<StreetSegmentIdx> subpathSegments = {};

        // Construct the path from nodes
        IntersectionIdx curNodeId = destIntersection;
        StreetSegmentIdx curSegment = mapNodes[destIntersection].reachingEdge;
        
        while(curSegment != NO_EDGE) { // While there are more street segments to add

            // Add the street segment id
            subpathSegments.push_back(curSegment);

            // Update the next intersection id from the previous node
            SegmentInfo &edgeInfo = map->streetSegmentTable[curSegment];
            curNodeId = (edgeInfo.to == curNodeId) ? edgeInfo.from : edgeInfo.to;
            curSegment = mapNodes[curNodeId].reachingEdge;

        }

        // Reverse the path
        std::reverse(subpathSegments.begin(), subpathSegments.end());

        double subpathTime = computePathTravelTime(subpathSegments, turn_penalty);

        // Add to the kMatrix
        kMatrix[kIdx].push_back({destIntersection, subpathTime, subpathSegments});
    }

    return true;
}


std::vector<StreetSegmentIdx> findPathToDepot (
    const std::pair<IntersectionIdx, std::vector<IntersectionIdx>> intersection_ids,
    const double turn_penalty, IntersectionIdx &lastDepot
) {

    std::vector<StreetSegmentIdx> subpathSegments = {};
    std::unordered_map<IntersectionIdx, Node> mapNodes;

    IntersectionIdx srcIntersection = intersection_ids.first;
    std::vector<IntersectionIdx> destIntersections = intersection_ids.second;

    // Get current map to access data structures
    MapData* map = getCurrentMap();

    // Generate the shortest path (consists of nodes)
    IntersectionIdx closestDepot = -1;

    bool pathFound = shortestPathMulti(srcIntersection, destIntersections, turn_penalty, map, mapNodes, true, closestDepot);

    if (!pathFound || closestDepot == -1) // No path found
        return subpathSegments;

    // Set the last depot as the closets depot found
    lastDepot = closestDepot;

    // Construct the path from nodes
    IntersectionIdx curNodeId = closestDepot;
    StreetSegmentIdx curSegment = mapNodes[closestDepot].reachingEdge;
    
    while(curSegment != NO_EDGE) { // While there are more street segments to add

        // Add the street segment id
        subpathSegments.push_back(curSegment);

        // Update the next intersection id from the previous node
        SegmentInfo &edgeInfo = map->streetSegmentTable[curSegment];
        curNodeId = (edgeInfo.to == curNodeId) ? edgeInfo.from : edgeInfo.to;
        curSegment = mapNodes[curNodeId].reachingEdge;

    }

    // Reverse the path
    std::reverse(subpathSegments.begin(), subpathSegments.end());
    return subpathSegments;

}

