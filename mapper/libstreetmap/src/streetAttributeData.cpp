#include "streetAttributeData.h"

std::vector<ezgl::color> featureColoursLight = 
{
    ezgl::color(255,0,92),     //UNKNOWN
    ezgl::color(194,218,184),  //PARK
    ezgl::color(210,170,109),  //BEACH
    ezgl::color(173,216,230),  //LAKE
    ezgl::color(77,77,255),    //RIVER
    ezgl::color(241,238,232),  //ISLAND
    ezgl::color(211,211,211),  //BUILDING
    ezgl::color(75,122,71),    //GREENSPACE
    ezgl::color(66,105,47),    //GOLFCOURSE
    ezgl::color(135,206,235)   //STREAM
};

ezgl::color fontColor = ezgl::color(47, 47, 47);
ezgl::color pathColor = ezgl::color(51, 87, 255);
int pathWidth = 15; // 15 meters for visibility


/*

    Table to hold all the street types that are being mapped.
    Each type has:
        (thickness)
        (color)
        (fontColor)
*/
std::unordered_map<std::string, StreetAttribute> StreetAttributesTable = 

{
    {
        "motorway",
        {
            12,
            ezgl::color(233, 144, 160),
            fontColor
        }
    },

    {
        "trunk",
        {
            10,
            ezgl::color(251, 192, 172),
            fontColor
        }
    },

    {
        "primary",
        {
            8,
            ezgl::color(253, 215, 161),
            fontColor
        }
    },

    {
        "secondary",
        {
            7,
            ezgl::color(206, 210, 147),
            fontColor
        }
    },

    {
        "tertiary",
        {
            5,
            ezgl::color(180, 180, 180),

            fontColor
        }
    },

    {
        "unclassified",
        {
            5,
            ezgl::color(180, 180, 180),

            fontColor
        }
    },

    {
        "residential",
        {
            5,
            ezgl::color(180, 180, 180),

            fontColor
        }
    },

    {
        "motorway_link",
        {
            4,
            ezgl::color(233, 144, 160),
            fontColor
        }
    },

    {
        "trunk_link",
        {
            3,
            ezgl::color(251, 192, 172),
            fontColor
        }
    },

    {
        "primary_link",
        {
            3,
            ezgl::color(253, 215, 161),
            fontColor
        }
    },

    {
        "secondary_link",
        {
            3,
            ezgl::color(206, 210, 147),
            fontColor
        }
    },

    {
        "tertiary_link",
        {
            2,
            ezgl::color(180, 180, 180),

            fontColor
        }
    },

    {
        "default",
        {
            3,
            ezgl::color(180, 180, 180),

            fontColor
        }
    },
};