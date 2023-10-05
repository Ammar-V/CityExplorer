#include <unordered_map>


#include "loadFunctions.h"




#ifndef LOAD_MAPS_H
#define LOAD_MAPS_H

extern bool closingMap;
extern std::unordered_map<std::string, MapData> Maps;

bool loadAllMaps();







#endif // LOAD_MAPS_H