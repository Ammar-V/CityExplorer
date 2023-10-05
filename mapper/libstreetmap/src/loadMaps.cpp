#include "loadMaps.h"
#include "loadFunctions.h"
#include "m1.h"




bool closingMap = false;
std::unordered_map<std::string, MapData> Maps = {};


bool loadAllMaps() {

    for (auto &mapIter: CityPaths) {
        
        std::string cityName = mapIter.first;
        std::string cityPath = mapIter.second;

        closeMap();
        std::cout << "Closed previous map..." << std::endl;

        bool loaded = loadMap(cityPath);
        if (!loaded) {
            std::cout << "City not loaded: " << cityName << std::endl;
            continue;
        }

        MapData *curMap = getCurrentMap();
        Maps.insert({cityName, *curMap});
        std::cout << "City loaded." << std::endl;

    }

    
    return true;


}
