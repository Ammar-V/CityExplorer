#include <vector>
#include <unordered_map>
#include <string>

#include "ezgl/color.hpp"

#ifndef STREET_ATTRIBUTE_DATA_H
#define STREET_ATTRIBUTE_DATA_H


extern std::vector<ezgl::color> featureColoursLight;

struct StreetAttribute {
    int thickness;
    ezgl::color color;
    ezgl::color nameColor;
};

extern std::unordered_map<std::string, StreetAttribute> StreetAttributesTable;
extern ezgl::color pathColor;
extern int pathWidth;


enum StreetTypes {
    defualt,
    tertiary_link,
    secondary_link,
    primary_link,
    trunk_link,
    motorway_link,
    residential,
    unclassified,
    tertiary,
    secondary,
    primary,
    trunk,
    motorway
};

#endif // STREET_ATTRIBUTE_DATA_H

