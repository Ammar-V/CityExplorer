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

#include "loadFunctions.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

#include "loadFunctions.h"
#include "streetAttributeData.h"
#include "loadMaps.h"
#include "pathfinding.h"


#include <iostream>
#include <math.h>
#include <chrono>
#include <ctime>
#include <thread>
#include <sstream>
#include <string>
#include <fstream>
#include <thread>
#include <future>
#include <algorithm>
#include <iostream>

// #include <opencv2/opencv.hpp>

// Set up the ezgl graphics window and hand control to it, as shown in the 
// ezgl example program. 
// This function will be called by both the unit tests (ece297exercise) 
// and your main() function in main/src/main.cpp.
// The unit tests always call loadMap() before calling this function
// and call closeMap() after this function returns.

#define BEAR_ANGLE 2.356
#define SHARP_ANGLE 0.7854

// Helper function declarations
double metersToPx(ezgl::renderer *g, int thickness);
int getWindowWidth(ezgl::renderer *g);
void drawSegment (ezgl::renderer *g, SegmentInfo &ssInfo, ezgl::point2d &p1, ezgl::point2d &p2, bool isPath);
float getSlopeTwoPoints(ezgl::point2d &p1, ezgl::point2d &p2);
void loadHeatMap();
void zoomToIntersection();
void resetUI();
float lon_from_x(double x);
float lat_from_y(double y);
void drawStreetName (ezgl::renderer *g, StreetNameSegment nameSegment);
void draw_features(ezgl::renderer *g, std::vector<featureInfo> &features);
void draw_buildings(ezgl::renderer *g);
void findIntersectionsBetweenPartialNames(/*ezgl::renderer *g,*/ std::string street1, std::string street2);
void draw_heatmap(ezgl::renderer *g);
std::unordered_set<StreetIdx> getIntersections(std::string street1, std::string street2);
void initializeIntersectionsList();

void getDirections();
bool turn (StreetSegmentIdx &cur, StreetSegmentIdx &next);
float getTurnAngle (ezgl::point2d &start, ezgl::point2d &turn, ezgl::point2d &end);
std::vector<ezgl::point2d> getPointsOfTurn (StreetSegmentIdx &cur, StreetSegmentIdx &next);
std::string getLeftOrRight (ezgl::point2d &start, ezgl::point2d &turn, ezgl::point2d &end);

// Callback function declarations
void draw_main_canvas (ezgl::renderer *g);
void draw_intersections(ezgl::renderer *g);
void draw_street_segments(ezgl::renderer *g, StreetZones &streetSegs, int roadSize, bool isPath);
void draw_POI(ezgl::renderer *g);
void draw_POI_x(ezgl::renderer *g);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void highlight_intersection_id(ezgl::application* app, GdkEventButton* event, double x, double y);
void highlight_intersection_id_find(ezgl::application* app, IntersectionIdx intersectionId);
void bookmark_point_of_interest(ezgl::application* app, GdkEventButton* event, double x, double y);
bool check_in_bookmark(POIIdx poiIndex);


void initial_setup(ezgl::application * application, bool /*new_window*/);
void draw_weather(ezgl::renderer *g);


// GUI Callback functions
void act_on_find_button_click(GtkButton * find);
void act_on_show_find_click(GtkButton * showFind);
void act_on_map_select(GtkComboBoxText *box);
void act_on_changed_search_entry(GtkEntry * entry);
void act_on_enter_press_search_entry(GtkEntry * entry);
void act_on_find_next_button(GtkButton * findNext);
void act_on_explorer_toggle(GtkSwitch * toggle);
void show_add_bookmark_button();
void act_on_toggle_restaurants(GtkToggleButton * filter);
void act_on_toggle_hospitals(GtkToggleButton * filter);
void act_on_toggle_bars(GtkToggleButton * filter);
void act_on_toggle_things_to_do(GtkToggleButton * filter);
void act_on_toggle_gas_stations(GtkToggleButton * filter);
void act_on_heat_map_button(GtkButton * heatmap);
void act_on_weather_button(GtkButton * weather);
void act_on_bookmark_button(GtkButton * bookmark);
void act_on_clear_bookmark_button(GtkButton * clearBookmark);
void act_on_route_button(GtkButton * route);
void act_on_directions_button(GtkButton * directions);
void act_on_close_directions_button(GtkButton * directions);
void act_on_enter_press_from_entry(GtkEntry * entry);
void act_on_from_entry_delete();
void act_on_to_entry_delete();

void threadApplication();
void printLoading(std::string selectedCity);



// Global variable to hold the map
MapData *curMap;
LocationInfo locs;
ezgl::canvas *mapCanvas;
ezgl::application *App;
float kmWidth;
std::unordered_set<IntersectionIdx> intersections;
auto it = intersections.begin();
IntersectionIdx currentIntersection = -1;
IntersectionIdx from = -1;
IntersectionIdx to = -1;
bool restaurants, hospitals, bars, things_to_do, gas_stations;
bool showHeatMap = 0, showWeather = 0, showBookmark = 0;

std::string cityName = "Toronto"; // Initial city
std::vector<ezgl::point2d> bookmarks;

std::vector<std::vector<ezgl::color>> HeatmapImg = {};

ezgl::rectangle previousScreen; // Used to limit zoom after a certain point

// std::pair<IntersectionIdx, IntersectionIdx> currentPathIntersections = {-1, -1};
std::vector<StreetSegmentIdx> currentPath = {};

void drawMap() {

   ezgl::application::settings settings;
   settings.main_ui_resource = directory_path + "/libstreetmap/resources/main.ui";

   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";

   // Get the current map object
   curMap = getCurrentMap();

   ezgl::application application(settings);
   App = &application;
   
   locs = curMap->locationInfo;
   double min_lat = curMap->locationInfo.min_lat;
   double min_lon = curMap->locationInfo.min_lon;
   double max_lat = curMap->locationInfo.max_lat;
   double max_lon = curMap->locationInfo.max_lon;

   if (cityName == "Toronto") {
      loadHeatMap();
   }

   ezgl::rectangle initial_world({curMap->x_from_lon(min_lon), curMap->y_from_lat(min_lat)},
                                 {curMap->x_from_lon(max_lon), curMap->y_from_lat(max_lat)});
   mapCanvas = application.add_canvas("MainCanvas", 
                           draw_main_canvas,
                           initial_world);

   application.run(initial_setup, act_on_mouse_click,
                   nullptr, nullptr);
   
}

//Render in map features
void draw_main_canvas (ezgl::renderer *g) {

   // Using time point and system_clock
   std::chrono::time_point<std::chrono::system_clock> start, end;

   // Start time
   start = std::chrono::system_clock::now();

   //Set background color to beige
   g->set_color(ezgl::color(241, 238, 232));
   
   g->fill_rectangle({curMap->x_from_lon(locs.min_lon), curMap->y_from_lat(locs.min_lat)},
                     {curMap->x_from_lon(locs.max_lon), curMap->y_from_lat(locs.max_lat)});
   
   // Get window width in km
   kmWidth = getWindowWidth(g) / 1000.0;

   // If zoomed in too much then reset to previous screen
   if (kmWidth < 0.005) g->set_visible_world(previousScreen);

   //render features based on area and window size
   draw_features(g, curMap->xLargeFeatures);
   if(kmWidth < 30){
      draw_features(g, curMap->largeFeatures);
   }
   if(kmWidth < 10){
      draw_features(g, curMap->mediumFeatures);
      draw_features(g, curMap->openFeatures);
   }
   if(kmWidth < 5){
      draw_features(g, curMap->smallFeatures);
   }
   if (kmWidth < 1){
      draw_buildings(g);
   }

   // render streets based on window size and street segment size
   if (kmWidth < 10) {
      draw_street_segments(g, curMap->smallSSegments, 1, false);
   }
   if (kmWidth < 30) {
      draw_street_segments(g, curMap->mediumSSegments, 2, false);
   }
   //always render large segments and highways
   draw_street_segments(g, curMap->largeSSegments, 3, false);
   draw_street_segments(g, curMap->highwaySSegments, 4, false);


   // Draw the current path if routing is on and a valid route is selected
   if (gtk_widget_get_visible(GTK_WIDGET(App->get_object("fromEntry"))) 
         && from != -1 
         && to != -1) {
      StreetZones dummyVector = {};

      pathWidth = 5;
      draw_street_segments(g, dummyVector, 4, true);

      pathWidth = metersToPx(g, 12); // 15 meters
      draw_street_segments(g, dummyVector, 4, true);
   }


   //render highlighted intersections
   draw_intersections(g);

   //render POI's
   if (kmWidth < 1) {
      draw_POI(g);
   }
  
   //render heat map if window screen is small and heat map is turned on
   if (kmWidth < 5 && showHeatMap) {
      if (cityName == "Toronto") draw_heatmap(g);
   }

   // Show weather   
   if (showWeather)
      draw_weather(g);

   // End time
   end = std::chrono::system_clock::now();
   std::chrono::duration<double> elapsed_seconds = end - start;

   double fps = 1 / elapsed_seconds.count();
   App->update_message(std::to_string(fps));

   //Print highlighted intersection name to status bar
   if (currentIntersection != -1) {
      std::string ss  =  "Intersection Selected: " + curMap->intersectionInfoTable[currentIntersection].name;
      App->update_message(ss);
   }
   
   //Set previous screen to restrict zoom
   previousScreen = g->get_visible_world();

}


//Connect all signals for widgets
void initial_setup(ezgl::application * application, bool /*new_window*/){

   gtk_widget_set_visible(
      GTK_WIDGET(App->get_object("findEntry")), false);

   gtk_widget_set_visible(GTK_WIDGET(App->get_object("FindIntersectionsButton")), false);

   //Connect Directions Button
   GObject *directionsButton = application->get_object("directionsButton");
   g_signal_connect(
   directionsButton, 
   "clicked", 
   G_CALLBACK(act_on_directions_button),
   NULL 
   );

   //Connect Close Directions Button
   GObject *closeDirectionsButton = application->get_object("closeDirections");
   g_signal_connect(
   closeDirectionsButton, 
   "clicked", 
   G_CALLBACK(act_on_close_directions_button),
   NULL 
   );
   
   //Connect from entry enter press
   GObject *fromEntryActivate = application->get_object("fromEntry");
   g_signal_connect(
   fromEntryActivate, 
   "activate", 
   G_CALLBACK(act_on_enter_press_from_entry), 
   NULL 
   );

   //Connect to entry enter press
   GObject *toEntryActivate = application->get_object("toEntry");
   g_signal_connect(
   toEntryActivate, 
   "activate", 
   G_CALLBACK(act_on_route_button), 
   NULL 
   );

   //Connect from entry delete
   GObject *fromEntryDelete = application->get_object("fromEntry");
   g_signal_connect(
   fromEntryDelete, 
   "icon-press", 
   G_CALLBACK(act_on_from_entry_delete), 
   NULL 
   );

   //Connect to entry delete
   GObject *toEntryDelete = application->get_object("toEntry");
   g_signal_connect(
   toEntryDelete, 
   "icon-press", 
   G_CALLBACK(act_on_to_entry_delete), 
   NULL 
   );

   //Connect Route Button
   GObject *routeButton = application->get_object("routeButton");
   g_signal_connect(
   routeButton, 
   "clicked", 
   G_CALLBACK(act_on_route_button),
   NULL 
   );

   //Connect Explorer toggle
   GObject *explorerToggle = application->get_object("explorerToggle");
   g_signal_connect(
   explorerToggle, 
   "state-set", 
   G_CALLBACK(act_on_explorer_toggle), 
   NULL 
   );

   //Connect heat map button
   GObject *heatButton = application->get_object("social");
   g_signal_connect(
   heatButton, 
   "clicked", 
   G_CALLBACK(act_on_heat_map_button),
   NULL 
   );

   //Connect weather button
   GObject *weatherButton = application->get_object("weather");
   g_signal_connect(
   weatherButton, 
   "clicked", 
   G_CALLBACK(act_on_weather_button),
   NULL 
   );

   //Connect bookmark button
   GObject *bookmarkButton = application->get_object("bookmark");
   g_signal_connect(
   bookmarkButton, 
   "clicked", 
   G_CALLBACK(act_on_bookmark_button),
   NULL 
   );

   //Connect clear bookmarks button
   GObject *clearBookmarkButton = application->get_object("clearBookmarks");
   g_signal_connect(
   clearBookmarkButton, 
   "clicked", 
   G_CALLBACK(act_on_clear_bookmark_button),
   NULL 
   );

   //Connect search entry enter press
   GObject *searchEntryActivate = application->get_object("searchEntry");
   g_signal_connect(
   searchEntryActivate, 
   "activate", 
   G_CALLBACK(act_on_enter_press_search_entry), 
   NULL 
   );

   //Connect find entry enter press
   GObject *findEntryActivate = application->get_object("findEntry");
   g_signal_connect(
   findEntryActivate, 
   "activate", 
   G_CALLBACK(act_on_find_button_click), 
   NULL 
   );

   //Connect the find intersections button
   GObject *findButton = application->get_object("FindIntersectionsButton");
   g_signal_connect(
   findButton, 
   "clicked", 
   G_CALLBACK(act_on_find_button_click), 
   NULL 
   );

   //Connect findNext button
   GObject *findNext = application->get_object("findNext");
   g_signal_connect(
   findNext, 
   "clicked", 
   G_CALLBACK(act_on_find_next_button), 
   NULL 
   );

   //Connect showFind button
   GObject *showFind = application->get_object("showFind");
   g_signal_connect(
   showFind, 
   "clicked", 
   G_CALLBACK(act_on_show_find_click), 
   NULL 
   );

   //Connect restaurant filter
   GObject *restaurantFilter = application->get_object("restaurants");
   g_signal_connect(
   restaurantFilter, 
   "toggled", 
   G_CALLBACK(act_on_toggle_restaurants), 
   NULL 
   );

   //Connect hospitals filter
   GObject *hospitalFilter = application->get_object("hospitals");
   g_signal_connect(
   hospitalFilter, 
   "toggled", 
   G_CALLBACK(act_on_toggle_hospitals), 
   NULL 
   );

   //Connect bar filter
   GObject *barFilter = application->get_object("bars");
   g_signal_connect(
   barFilter, 
   "toggled", 
   G_CALLBACK(act_on_toggle_bars), 
   NULL 
   );

   //Connect things to do filter
   GObject *thingsToDoFilter = application->get_object("thingsToDo");
   g_signal_connect(
   thingsToDoFilter, 
   "toggled", 
   G_CALLBACK(act_on_toggle_things_to_do), 
   NULL 
   );

   //Connect gas stations filter
   GObject *gasStationFilter = application->get_object("gasStations");
   g_signal_connect(
   gasStationFilter, 
   "toggled", 
   G_CALLBACK(act_on_toggle_gas_stations), 
   NULL 
   );

   //connect search bar to search entry
   GtkSearchBar * bar = GTK_SEARCH_BAR(application->get_object("SearchBar"));
   GtkEntry * entry = GTK_ENTRY(application->get_object("SearchEntry"));
   gtk_search_bar_connect_entry(bar, entry);

   // Connect the City Selector combo box
   GObject *citySelector = application->get_object("CitySelectorComboBox");
   g_signal_connect(
      citySelector,
      "changed",
      G_CALLBACK(act_on_map_select),
      nullptr
   );

   initializeIntersectionsList(); 
}

void initializeIntersectionsList() {
   //Initialize the ListStore for navigation search completion

   GtkListStore *intersectionList = GTK_LIST_STORE(App->get_object("intersectionNames"));

   //Clear current list store
   gtk_list_store_clear(GTK_LIST_STORE(intersectionList));


   //Fill with new intersections 
   for(int i = 0; i < curMap->intersectionInfoTable.size(); i++){
      gtk_list_store_insert_with_values(intersectionList, NULL, -1, 0, curMap->intersectionInfoTable[i].name.c_str(), -1);
   }
}


void act_on_directions_button(GtkButton * /*directions*/){

   gtk_widget_set_visible(GTK_WIDGET(App->get_object("directions")), true);
   gtk_widget_set_visible(GTK_WIDGET(App->get_object("search")), false);

}

void act_on_close_directions_button(GtkButton * /*directions*/){


   gtk_container_foreach(GTK_CONTAINER(App->get_object("directionsGrid")), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);
   gtk_widget_set_visible(GTK_WIDGET(App->get_object("directions")), false);
   gtk_widget_set_visible(GTK_WIDGET(App->get_object("search")), true);
   gtk_widget_set_visible(GTK_WIDGET(App->get_object("directionsWindow")), false);

   //Clear to and from locations
   gtk_entry_set_text(GTK_ENTRY(App->get_object("fromEntry")), "");
   gtk_entry_set_text(GTK_ENTRY(App->get_object("toEntry")), "");

   from = -1;
   to = -1;

   App->refresh_drawing();

}

void act_on_from_entry_delete() {

   from = -1;
   gtk_container_foreach(GTK_CONTAINER(App->get_object("directionsGrid")), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);
   App->refresh_drawing();
}

void act_on_to_entry_delete() {

   to = -1;
   gtk_container_foreach(GTK_CONTAINER(App->get_object("directionsGrid")), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);
   App->refresh_drawing();
}

void act_on_enter_press_from_entry(GtkEntry * /*entry*/){

   
   gtk_widget_grab_focus(GTK_WIDGET(App->get_object("toEntry")));
   
}

void act_on_route_button(GtkButton * /*route*/){

   //get text from entry bars
   GtkEntry* text_entry1 = GTK_ENTRY(App->get_object("fromEntry"));
   GtkEntry* text_entry2 = GTK_ENTRY(App->get_object("toEntry"));
   const gchar* name1 = gtk_entry_get_text(text_entry1);
   const gchar* name2 = gtk_entry_get_text(text_entry2);

   std::stringstream string1(name1);
   std::stringstream string2(name2);
   
   std::string street1, street2, street3, street4;
   std::getline(string1, street1, '&');
   std::getline(string1, street2);
   std::getline(string2, street3, '&');
   std::getline(string2, street4);

   std::unordered_set<IntersectionIdx> possibleIntersectionsFrom, possibleIntersectionsTo;

   //call function to highlight intersections between the possible streets
   if(street1 != "" && street2 != "" && from == -1){
      possibleIntersectionsFrom = getIntersections(street1, street2);
      if(possibleIntersectionsFrom.size() != 0) from = *possibleIntersectionsFrom.begin();
      
   }
   if(street3 != "" && street4 != "" && to == -1){
      possibleIntersectionsTo = getIntersections(street3, street4);
      if(possibleIntersectionsTo.size() != 0) to = *possibleIntersectionsTo.begin();
      
   }
   if(from == -1 || to == -1){

      std::string error = "Error: intersection does not exist";
      App->update_message(error);
      return;

   }

   gtk_entry_set_text(GTK_ENTRY(App->get_object("fromEntry")), curMap->intersectionInfoTable[from].name.c_str());
   gtk_entry_set_text(GTK_ENTRY(App->get_object("toEntry")), curMap->intersectionInfoTable[to].name.c_str());

   currentPath = findPathBetweenIntersections({from, to}, 0);
   getDirections();


   gtk_widget_set_visible(GTK_WIDGET(App->get_object("directionsWindow")), true);
   
   App->refresh_drawing();

}

//Make explorer mode features appear and disappear on toggle of explorer mode
void act_on_explorer_toggle(GtkSwitch * /*toggle*/){

   GtkWidget* filters = GTK_WIDGET(App->get_object("searchFilters"));
   GtkWidget* social = GTK_WIDGET(App->get_object("social"));
   GtkWidget* weather = GTK_WIDGET(App->get_object("weather"));
   GtkWidget* bookmark = GTK_WIDGET(App->get_object("bookmark"));

   
   if (!gtk_switch_get_active(GTK_SWITCH(App->get_object("explorerToggle")))){
      restaurants = 0;
      hospitals = 0;
      bars = 0;
      things_to_do = 0;
      gas_stations = 0;
      showHeatMap = 0;
      showWeather = 0;
      showBookmark = 0;
      
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("restaurants")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("hospitals")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("bars")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("thingsToDo")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("gasStations")), false);

      gtk_widget_set_visible(GTK_WIDGET(App->get_object("addBookmark")), false);
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("clearBookmarks")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("addBookmark")), false);

   }

   gtk_widget_set_visible(filters, !gtk_widget_get_visible(filters));
   gtk_widget_set_visible(social, !gtk_widget_get_visible(social));
   gtk_widget_set_visible(weather, !gtk_widget_get_visible(weather));
   gtk_widget_set_visible(bookmark, !gtk_widget_get_visible(bookmark));

   App->refresh_drawing();

}

//Make the second search entry box and find button appear on click of down arrow
void act_on_show_find_click(GtkButton * /*find*/){

   GtkWidget* entry = GTK_WIDGET(App->get_object("findEntry"));
   GtkWidget* button = GTK_WIDGET(App->get_object("FindIntersectionsButton"));

   GtkImage* showFind = GTK_IMAGE(App->get_object("showFindImage"));
   
   std::string icon_name;

   if(gtk_widget_get_visible(entry)){
      icon_name = "go-down";
   }
   else{
      icon_name = "go-up";
   }

   gtk_image_set_from_icon_name(showFind, icon_name.c_str(), GTK_ICON_SIZE_BUTTON);

   gtk_widget_set_visible(entry, !gtk_widget_get_visible(entry));
   gtk_widget_set_visible(button, !gtk_widget_get_visible(button));

}

//Clear search bar on enter
void act_on_enter_press_search_entry(GtkEntry * entry){

   const gchar* searched = gtk_entry_get_text(entry);

   std::stringstream searchedStream(searched);
   std::string searchEntry;
   std::getline(searchedStream, searchEntry);

   std::string ss  =  "Searched for: " + searchEntry;
   App->update_message(ss);

   gtk_entry_set_text(entry, "");

}

bool mapLoaded = false;

// Multi-threading
void printLoading(std::string selectedCity)
{
   mapLoaded = false;

   std::string cityPath = CityPaths[selectedCity];

   closeMap();
   std::cout << "Closed previous map..." << std::endl;

   bool loaded = loadMap(cityPath);
   if (!loaded) {
      std::cout << "City not loaded." << std::endl;
      return;
   }

   mapLoaded = true;

}

//Load in different maps based on combo box selection
void act_on_map_select(GtkComboBoxText *box) {
   std::string selectedCity = std::string(gtk_combo_box_text_get_active_text(box));

   if (CityPaths.find(selectedCity) == CityPaths.end()) {
      std::cout << "City Map Path not found." << std::endl;
      return;
   }

   mapLoaded = false;
   // delete curMap;


   cityName = selectedCity;


   // Show loading message
   std::thread th(&printLoading, std::move(selectedCity));// Starting Thread & move the future object in lambda function by reference

   while (!mapLoaded) {

      App->update_message("Loading");
      App->flush_drawing();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));


      App->update_message("Loading.");
      App->flush_drawing();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      App->update_message("Loading..");
      App->flush_drawing();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));


      App->update_message("Loading...");
      App->flush_drawing();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

   }


   // End the thread
   th.join(); //Waiting for thread to be joined.
   

   std::cout << "City loaded." << std::endl;
   curMap = getCurrentMap();
   currentPath.clear();
   Nodes.clear();
   from = -1;
   to = -1;
      

   // Set the coordinate system
   locs = curMap->locationInfo;
   double min_lat = curMap->locationInfo.min_lat;
   double min_lon = curMap->locationInfo.min_lon;
   double max_lat = curMap->locationInfo.max_lat;
   double max_lon = curMap->locationInfo.max_lon;

   ezgl::rectangle new_world({curMap->x_from_lon(min_lon), curMap->y_from_lat(min_lat)},
                                 {curMap->x_from_lon(max_lon), curMap->y_from_lat(max_lat)});
   App->change_canvas_world_coordinates("MainCanvas", new_world);

   ezgl::renderer *g = App->get_renderer();
   // Set the font for the new city
   if (cityName == "Beijing")
      g->format_font("Noto Sans CJK SC", ezgl::font_slant(0), ezgl::font_weight(0));
   else if (cityName == "Tokyo")
      g->format_font("Noto Serif CJK JP", ezgl::font_slant(0), ezgl::font_weight(0));
   else if (cityName == "Hong Kong")
      g->format_font("Noto Serif CJK TC", ezgl::font_slant(0), ezgl::font_weight(0));
   else if (cityName == "Cairo")
      g->format_font("Traditional Arabic", ezgl::font_slant(0), ezgl::font_weight(0));
   else
      g->format_font("Noto Sans", ezgl::font_slant(0), ezgl::font_weight(0));

   // Reset UI
   resetUI();

   App->refresh_drawing();

}

//Reset the UI for loading in a different map
void resetUI () {

   currentIntersection = -1;

   restaurants = 0;
   hospitals = 0;
   bars = 0;
   things_to_do = 0;
   gas_stations = 0;
   showHeatMap = 0;
   showWeather = 0;
   showBookmark = 0;
   bookmarks.clear();
   
   gtk_switch_set_active(GTK_SWITCH(App->get_object("explorerToggle")), false);


   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("restaurants")), false);
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("hospitals")), false);
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("bars")), false);
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("thingsToDo")), false);
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("gasStations")), false);

   gtk_widget_set_visible(GTK_WIDGET(App->get_object("addBookmark")), false);
   gtk_widget_set_visible(GTK_WIDGET(App->get_object("clearBookmarks")), false);
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("addBookmark")), false);

   GtkWidget* filters = GTK_WIDGET(App->get_object("searchFilters"));
   GtkWidget* social = GTK_WIDGET(App->get_object("social"));
   GtkWidget* weather = GTK_WIDGET(App->get_object("weather"));
   GtkWidget* bookmark = GTK_WIDGET(App->get_object("bookmark"));

   gtk_widget_set_visible(filters, false);
   gtk_widget_set_visible(social, false);
   gtk_widget_set_visible(weather, false);
   gtk_widget_set_visible(bookmark, false);

   // Reset Search
   GtkWidget* entry = GTK_WIDGET(App->get_object("findEntry"));
   GtkWidget* button = GTK_WIDGET(App->get_object("FindIntersectionsButton"));
   GtkWidget* findNext = GTK_WIDGET(App->get_object("findNext"));
   GtkImage* showFind = GTK_IMAGE(App->get_object("showFindImage"));
   
   std::string icon_name = "go-down";

   gtk_image_set_from_icon_name(showFind, icon_name.c_str(), GTK_ICON_SIZE_BUTTON);
   

   gtk_widget_set_visible(entry, false);
   gtk_widget_set_visible(button, false);
   gtk_widget_set_visible(findNext, false);

   // Reset the route search boxes
   act_on_close_directions_button(NULL);
   initializeIntersectionsList();


}

//Clear existing bookmarks on click of the clear button
void act_on_clear_bookmark_button(GtkButton * /*clearBookmark*/){

   bookmarks.clear();

   App->refresh_drawing();
}

//Highlight intersections between two partials on click of find button
void act_on_find_button_click(GtkButton * /*find*/){


   //get text from Find Intersection Text Entry bar
   GtkEntry* text_entry1 = GTK_ENTRY(App->get_object("searchEntry"));
   GtkEntry* text_entry2 = GTK_ENTRY(App->get_object("findEntry"));
   const gchar* name1 = gtk_entry_get_text(text_entry1);
   const gchar* name2 = gtk_entry_get_text(text_entry2);

   std::stringstream string1(name1);
   std::stringstream string2(name2);
   
   std::string street1, street2;
   std::getline(string1, street1);
   std::getline(string2, street2);

   //call function to highlight intersections between the possible streets
   if(street1 != "" && street2 != ""){
      findIntersectionsBetweenPartialNames(/*App->get_renderer(),*/ street1, street2);
   }

   gtk_entry_set_text(text_entry1, "");
   gtk_entry_set_text(text_entry2, "");
}

//Loop through all possible highlighted intersections on click
void act_on_find_next_button(GtkButton * /*findNext*/){

   it++;

   //if no more intersections to highlight, return
   if(it == intersections.end()){
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("findNext")), false);
      currentIntersection = -1;
      App->refresh_drawing();

      return;
   }

   //highlight the next intersection in the vector
   currentIntersection = *it;
   zoomToIntersection();

   App->refresh_drawing();
}

//Highlight intersections or drop bookmarks on mouse click
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y){


   if(gtk_widget_get_visible(GTK_WIDGET(App->get_object("directions")))){ //right click
      LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));

      //Get the intersection
      int intersectionIdx = findClosestIntersection(pos);

      if(intersectionIdx == from){ //if location is already set as from, unset
         from = -1;
         gtk_container_foreach(GTK_CONTAINER(App->get_object("directionsGrid")), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);
         gtk_widget_set_visible(GTK_WIDGET(App->get_object("directionsWindow")), true);
         gtk_entry_set_text(GTK_ENTRY(App->get_object("fromEntry")), "");
         App->refresh_drawing();

      }
      else if (to == intersectionIdx) { // if location is already set as to, unset
         to = -1;
         gtk_container_foreach(GTK_CONTAINER(App->get_object("directionsGrid")), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);
         gtk_widget_set_visible(GTK_WIDGET(App->get_object("directionsWindow")), true);
         gtk_entry_set_text(GTK_ENTRY(App->get_object("toEntry")), "");
         App->refresh_drawing();
      }
      else if (from == -1) { //set from location
         from = intersectionIdx;


         gtk_entry_set_text(GTK_ENTRY(App->get_object("fromEntry")), curMap->intersectionInfoTable[from].name.c_str());

         if (from != -1 && to != -1)
            act_on_route_button(NULL);

         App->refresh_drawing();
      } 
      else if (to == -1 || from != -1) { //if to location isn't set, set to
         to = intersectionIdx;
  

         LatLon intersectionPoint = curMap->intersectionInfoTable[to].position;
         gtk_entry_set_text(GTK_ENTRY(App->get_object("toEntry")), curMap->intersectionInfoTable[to].name.c_str());
         ezgl::surface* locationIcon = App->get_renderer()->load_png((directory_path + "/libstreetmap/resources/icons/to.png").c_str());
         ezgl::point2d point;
         point.x = curMap->x_from_lon(intersectionPoint.longitude());
         point.y = curMap->y_from_lat(intersectionPoint.latitude());
         App->get_renderer()->draw_surface(locationIcon, point, 0.1);
         App->get_renderer()->free_surface(locationIcon);

         if (from != -1 && to != -1)
            act_on_route_button(NULL);

         App->refresh_drawing();

      }

   }

   else{
      if(showBookmark && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(App->get_object("addBookmark")))){
         bookmarks.push_back({x,y});
         //addBookmark = false;
         App->refresh_drawing();
      }
      else{
         highlight_intersection_id(app, event, x, y);

      }
   }
}

//Render in heat map on click of heat map button
void act_on_heat_map_button(GtkButton * /*heatmap*/){

   showHeatMap = !showHeatMap;

   if(showHeatMap){
      showWeather = 0;
      showBookmark = 0;
   }

   App->refresh_drawing();

}

//Show weather icon on click of weather button
void act_on_weather_button(GtkButton * /*weather*/){

   showWeather = !showWeather;

   if(showWeather){
      showHeatMap = 0;
      showBookmark = 0;
   }

   App->refresh_drawing();

}

//Show add and clear bookmark buttons on click of bookmark button
void act_on_bookmark_button(GtkButton * /*bookmark*/){

   showBookmark = !showBookmark;

   if(showBookmark){
      showWeather = 0;
      showHeatMap = 0;
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("addBookmark")), true);
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("clearBookmarks")), true);
   }
   else{
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("addBookmark")), false);
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("clearBookmarks")), false);
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(App->get_object("addBookmark")), false);
   }

   App->refresh_drawing();

}

//Display only restaurant icons when restaurant filter is on
void act_on_toggle_restaurants(GtkToggleButton * filter){
   
   restaurants = gtk_toggle_button_get_active(filter);
   
   App->refresh_drawing();

}

//Display only hospital icons when hospital filter is on
void act_on_toggle_hospitals(GtkToggleButton * filter){

   hospitals = gtk_toggle_button_get_active(filter);
   App->refresh_drawing();

}

//Display only bar icons when bar filter is on
void act_on_toggle_bars(GtkToggleButton * filter){

   bars = gtk_toggle_button_get_active(filter);
   App->refresh_drawing();

}

//Display only things to do when things to do filter is on
void act_on_toggle_things_to_do(GtkToggleButton * filter){

   things_to_do = gtk_toggle_button_get_active(filter);
   App->refresh_drawing();

}

//Display only gas stations when gas station filter is on
void act_on_toggle_gas_stations(GtkToggleButton * filter){

   gas_stations = gtk_toggle_button_get_active(filter);

   App->refresh_drawing();

}


//Pre load heat map data
void loadHeatMap() {

   std::string line;
   std::ifstream myfile (directory_path + "/libstreetmap/src/heatmap/heatmap.txt");
   int i = 0, j;
   if (myfile.is_open()) {

      while(std::getline(myfile, line)) {
         std::vector<int> nums;
         std::stringstream sstream(line);
         std::string rowString;

         int count = 0;
         j = 0;
         HeatmapImg.push_back({});

         while(std::getline(sstream, rowString, ' ')) {
            count++;
            nums.push_back(std::stoi(rowString));
            
            if (count == 4) {
               int size = nums.size();

               int red = nums[size - 4];
               int green = nums[size - 3];
               int blue = nums[size - 2];
               int alpha = nums[size - 1];

               ezgl::color color = ezgl::color(red, green, blue, alpha);
               HeatmapImg[i].push_back(color);

               count = 0;

               j++; // Increment the column index
            }

         }
         i++;
      }

      myfile.close();
   }

}

// Render in heatmap based on current zone
void draw_heatmap(ezgl::renderer *g) {

   if (kmWidth > 30)
      return;

   // MAP PARAMS
   int width = 2541;
   int height = 1957;
   float delta = 25.0;

   ezgl::point2d topCorner = g->get_visible_world().top_left();
   ezgl::point2d bottomCorner = g->get_visible_world().bottom_right();

   long int x1 = topCorner.x;
   long int x2 = bottomCorner.x;

   long int y1 = topCorner.y;
   long int y2 = bottomCorner.y;

   // Calculate initial coords
   int col_min = (x1 - curMap->x_from_lon(locs.min_lon)) / delta;
   int col_max = (x2 - curMap->x_from_lon(locs.max_lon)) / delta;

   int row_min = (y1 - curMap->y_from_lat(locs.max_lat)) / delta;
   int row_max = (y2 - curMap->y_from_lat(locs.min_lat)) / delta;

   // Bound checking
   if (col_min < 0) col_min = 0;
   if (col_min >= width) return;

   if (col_max > 0) col_max = width;
   if (col_max < 0) col_max = width + col_max;
   if (col_max < 0) return;

   if (row_min > 0) row_min = 0;
   if (row_min < 0) row_min *= -1;
   if (row_min >= height) return;

   if (row_max > height) return;
   if (row_max > 0) row_max = height - row_max;
   if (row_max <= 0) row_max = height;

   // Decide base corner
   long int base_x = col_min == 0 ? curMap->x_from_lon(locs.min_lon) : x1; // Add from base_x to go right in the frame
   long int base_y = row_min == 0 ? curMap->y_from_lat(locs.max_lat) : y1; // Subtract from base_y to go down in the frame
   // base_y -= abs(row_max - row_min) * 50;

   for (int i = row_min; i <= row_max; i++) {
      for (int j = col_min; j <= col_max; j++) {
         g->set_color(HeatmapImg[i][j]); // Manual offsets to fix on screen offset
         
         if (kmWidth < 1.1)
            g->fill_rectangle({base_x + (j - col_min) * delta, base_y - ((i - row_min ) * delta)},
                           {base_x + (j - col_min + 1) * delta, base_y - ((i - row_min + 1) * delta)});
         else
            g->fill_arc({base_x + (j - col_min) * delta + delta/2, base_y - ((i - row_min ) * delta - delta/2)}, metersToPx(g, delta / 2), 0, 360);

      }
   }
}


// Function that draws all street segments
void draw_street_segments(ezgl::renderer *g, StreetZones &streetSegsZoned, int roadSize, bool isPath) {
   
   std::vector<SegmentInfo> &ssTable = curMap->streetSegmentTable;

   int row_min, row_max, col_min, col_max;

   if (!isPath) {

      ezgl::point2d topCorner = g->get_visible_world().top_left();
      ezgl::point2d bottomCorner = g->get_visible_world().bottom_right();

      int x1 = topCorner.x;
      int x2 = bottomCorner.x;

      int y1 = topCorner.y;
      int y2 = bottomCorner.y;

      int width = streetSegsZoned.size();
      int height = streetSegsZoned[0].size();

      // Calculate initial coords
      col_min = (x1 - curMap->x_from_lon(locs.min_lon)) / curMap->zoneWidth;
      col_max = (x2 - curMap->x_from_lon(locs.max_lon)) / curMap->zoneWidth;

      row_min = (y1 - curMap->y_from_lat(locs.max_lat)) / curMap->zoneWidth;
      row_max = (y2 - curMap->y_from_lat(locs.min_lat)) / curMap->zoneWidth;
      
      // Bound checking horizontal
      if (col_min < 0) col_min = 0;
      if (col_min >= width) return;

      if (col_max > 0) col_max = width;
      if (col_max < 0) col_max = width + col_max;
      if (col_max < 0) return;

      // Bound checking vertical
      if (row_min < 0) row_min = 0;
      if (row_min >= height) return;

      if (row_max > 0) row_max = height;
      if (row_max < 0) row_max = height + row_max;
      if (row_max < 0) return;


      // If zoomed out
      if (abs(y2 - y1) > (curMap->y_from_lat(locs.min_lat) - curMap->y_from_lat(locs.max_lat))) {
         row_min = 0;
         row_max = height;
      }  
      

      // Change the bounds to account for edge cases
      col_max = std::min(width, col_max);
      row_max = std::min(height, row_max);

      col_min = std::max(0, col_min);
      row_min = std::max(0, row_min);

      int zoomOut = roadSize == 4 ? 6 : 3;

      // Zoom out slightly from the bounds
      if (col_max + zoomOut <= width) col_max += zoomOut;
      if (row_max + zoomOut <= height) row_max += zoomOut;

      if (col_min - zoomOut >= 0) col_min -= zoomOut;
      if (row_min - zoomOut >= 0) row_min -= zoomOut;
   } else {

      // The function is drawing street segments on a path
      row_min = 0;
      row_max = 1;
      col_min = 0;
      col_max = 2;
   }


   // ssTable.resize(getNumStreetSegments());
   for (int i = row_min; i < row_max; i++){
      for(int j = col_min + 1; j < col_max; j++){

         std::vector<StreetSegmentIdx>* streetVectorPtr;

         if (!isPath)
            streetVectorPtr = &streetSegsZoned[j][i];
         else
            streetVectorPtr = &currentPath;

         // Select the vector of segments
         std::vector<StreetSegmentIdx> &streetSegs = *streetVectorPtr;

         // For safety
         streetVectorPtr = nullptr;


         if (streetSegs.size() == 0) continue;

         for (StreetSegmentIdx iSS: streetSegs) {
            SegmentInfo &ssInfo = ssTable[iSS];
            // Check if the street is in current window
            LatLon locFrom = curMap->intersectionInfoTable[ssInfo.from].position;
            LatLon locTo = curMap->intersectionInfoTable[ssInfo.to].position;

            // StreetAttribute &attribute = ssInfo.attributes; 

            if (curMap->intersectionInfoTable[ssInfo.from].highlight 
                  || curMap->intersectionInfoTable[ssInfo.to].highlight) {

            }

            // Graphics settings
            g->set_line_cap(ezgl::line_cap(2));
            g->set_line_width(metersToPx(g, ssInfo.attributes.thickness));
            g->set_color(ssInfo.attributes.color);
            if (isPath) {
               g->set_line_width(pathWidth);
               g->set_color(pathColor);
            }


            for (int iCP = 0; iCP < ssInfo.numCurvePoints - 1; iCP++) {

               LatLon loc1 = getStreetSegmentCurvePoint(iSS, iCP);
               LatLon loc2 = getStreetSegmentCurvePoint(iSS, iCP + 1);

               // Get the actual points to draw between
               ezgl::point2d p1 = {curMap->x_from_lon(loc1.longitude()), curMap->y_from_lat(loc1.latitude())};
               ezgl::point2d p2 = {curMap->x_from_lon(loc2.longitude()), curMap->y_from_lat(loc2.latitude())};

               drawSegment(g, ssInfo, p1, p2, isPath);

               // Draw a line connecting first curve point to intersection
               if (iCP == 0) {
                  
                  ezgl::point2d p0 = {curMap->x_from_lon(locFrom.longitude()), curMap->y_from_lat(locFrom.latitude())};

                  drawSegment(g, ssInfo, p0, p1, isPath);

               } 
               // Draw a line connecting last cuve point to intersection
               if((iCP + 1) == ssInfo.numCurvePoints - 1) {
                  
                  ezgl::point2d pn = {curMap->x_from_lon(locTo.longitude()), curMap->y_from_lat(locTo.latitude())};
                  
                  drawSegment(g, ssInfo, p2, pn, isPath);

               }
            }


            ezgl::point2d p0 = {curMap->x_from_lon(locFrom.longitude()), curMap->y_from_lat(locFrom.latitude())};
            ezgl::point2d pn = {curMap->x_from_lon(locTo.longitude()), curMap->y_from_lat(locTo.latitude())};

            // Check if no curve points exists
            if (ssInfo.numCurvePoints == 0) {

               drawSegment(g, ssInfo, p0, pn, isPath);

            } else if (ssInfo.numCurvePoints == 1) {

               LatLon loc1 = getStreetSegmentCurvePoint(iSS, 0);

               // Get the actual points to draw between
               ezgl::point2d p1 = {curMap->x_from_lon(loc1.longitude()), curMap->y_from_lat(loc1.latitude())};

               drawSegment(g, ssInfo, p0, p1, isPath);
               drawSegment(g, ssInfo, p1, pn, isPath);

            }
         }
      }
   }
}

//Draw highlights on the current selected intersections
void draw_intersections(ezgl::renderer *g) {

   //Draw from location if it is set
   if(from != -1){
      LatLon intersectionPoint = curMap->intersectionInfoTable[from].position;
      // ezgl::surface* locationIcon = App->get_renderer()->load_png((directory_path + "/libstreetmap/resources/icons/to.png").c_str());
      ezgl::point2d point;
      point.x = curMap->x_from_lon(intersectionPoint.longitude());
      point.y = curMap->y_from_lat(intersectionPoint.latitude());
      // App->get_renderer()->draw_surface(locationIcon, point, 0.1);
      // App->get_renderer()->free_surface(locationIcon);

      g->set_color(ezgl::color(250, 0, 0));
      g->fill_arc(point, kmWidth*10, 0, 360);
   }
   //Draw to location if it is set
   if(to != -1){
      LatLon intersectionPoint = curMap->intersectionInfoTable[to].position;
         ezgl::surface* locationIcon = App->get_renderer()->load_png((directory_path + "/libstreetmap/resources/icons/to.png").c_str());
         ezgl::point2d point;
         point.x = curMap->x_from_lon(intersectionPoint.longitude());
         point.y = curMap->y_from_lat(intersectionPoint.latitude());
         App->get_renderer()->draw_surface(locationIcon, point, 0.1);
         App->get_renderer()->free_surface(locationIcon);
   }

   //Draw bookmarks if bookmark mode is on
   if(showBookmark){
      //Graphics Settings
      g->set_color(ezgl::color(50, 235, 180));
      g->set_line_cap(ezgl::line_cap());
      g->set_line_width(kmWidth/100);

      for(int i = 0; i < bookmarks.size(); i++){

         g->fill_arc(bookmarks[i], kmWidth*10, 0, 360);
      }

   }
   //Return if no current highlighted intersection
   if (currentIntersection == -1) return;

   //Highlight the selected intersection
   // Graphics settings
   g->set_color(ezgl::color(255, 105, 180));
   g->set_line_cap(ezgl::line_cap());
   g->set_line_width(metersToPx(g, 5));


   LatLon &loc = curMap->intersectionInfoTable[currentIntersection].position;
   float x = curMap->x_from_lon(loc.longitude());
   float y = curMap->y_from_lat(loc.latitude());

   g->fill_arc({x, y}, 10, 0, 360);

}

//Draw in feature colors
void draw_features(ezgl::renderer *g, std::vector<featureInfo> &features){
   

   for(size_t i = 0; i < features.size(); i++) {
      featureInfo &cur = features[i];


      g->set_color(featureColoursLight[cur.type]);

      if(cur.area > 0){ //render closed features

         if (cur.points.size() > 1){

            g->fill_poly(cur.points);
            
         }   
      }
      else{ //render open features

         g->set_line_cap(ezgl::line_cap(2));
         for(size_t j = 1; j < cur.numPoints; j++){
               g->set_line_width(metersToPx(g, 4));
               g->draw_line(cur.points[j-1], cur.points[j]);
            }

      }
   }
}


//Return a vector of all possible intersections
 std::unordered_set<StreetIdx> getIntersections(std::string street1, std::string street2){

    std::vector<StreetIdx> partials1, partials2;
    std::unordered_set<IntersectionIdx> possibleIntersections;

   partials1 = findStreetIdsFromPartialStreetName(street1);
   partials2 = findStreetIdsFromPartialStreetName(street2);

   //Find all possible intersections of those two strings
   for(int i = 0; i < partials1.size(); i++){
      for(int j = 0; j < partials2.size(); j++){
         std::vector<IntersectionIdx> intersectionStreets = findIntersectionsOfTwoStreets(partials1[i], partials2[j]);
         std::copy(intersectionStreets.begin(),intersectionStreets.end(),std::inserter(possibleIntersections,possibleIntersections.end()));
      }
   }

   return possibleIntersections;
    
 }


//Store the intersections needed to highlight from Find Search
void findIntersectionsBetweenPartialNames(/*ezgl::renderer *g,*/ std::string street1, std::string street2){

   intersections = getIntersections(street1, street2);

   //If no intersections
   //Return and print no results to status bar
   if(intersections.size() == 0){
      std::string ss  =  "No results";
      App->update_message(ss);
      return;
   }

   //Highlight first intersecction (with zoom to intersection)
   it = intersections.begin();
   currentIntersection = *it;

   //Zoom to highlighted intersection
   zoomToIntersection();
   

   //If multiple intersections to highlight
   //Forward button appears and remaining intersections highlighted on click
   if(intersections.size() > 1){
      
      gtk_widget_set_visible(GTK_WIDGET(App->get_object("findNext")), true);

   }
   else{
      intersections.clear();
   }

   App->refresh_drawing();
}

//Helper function to draw the street names
void drawStreetName(ezgl::renderer *g, StreetNameSegment nameSegment) {
   g->set_color(nameSegment.color);
   g->set_font_size(nameSegment.thickness / 2);

   ezgl::point2d &p1 = nameSegment.p1;
   ezgl::point2d &p2 = nameSegment.p2;


   float angle = getSlopeTwoPoints(p1, p2); // First: slope; Second: angle;
   g->set_text_rotation(angle / kDegreeToRadian);

   ezgl::point2d center = {(p2.x + p1.x) / 2, (p2.y + p1.y) / 2};

   g->draw_text(center, nameSegment.streetName);

}

//Function to convert real world thickenss to pixel thickness
double metersToPx(ezgl::renderer *g, int thickness) {

   // ezgl::camera currentCamera = mapCanvas->get_camera();

   ezgl::point2d topCorner = g->get_visible_world().top_left();

   ezgl::point2d psudoPoint = {topCorner.x + thickness, topCorner.y + thickness};
   ezgl::rectangle psudoRectangle (topCorner, psudoPoint);

   double width = g->world_to_screen(psudoRectangle).top_right().x;

   return width;
}

// Get the window screen width in meters.
int getWindowWidth(ezgl::renderer *g) {
   return g->get_visible_world().top_right().x - g->get_visible_world().top_left().x;
}

//Helper function to draw street segment
void drawSegment (ezgl::renderer *g, SegmentInfo &ssInfo, ezgl::point2d &p1, ezgl::point2d &p2, bool isPath) {

   // Draw the street segment
   g->draw_line(p1, p2);
   float length = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));

   if (ssInfo.streetName == "<unknown>") ssInfo.streetName = "Unknown Street";


   // Draw street names
   if ((kmWidth < 1 && length > 50) && (ssInfo.roadType != "motorway" || ssInfo.roadType != "motorway_link")) {

      double fontSize = (metersToPx(g, ssInfo.attributes.thickness) / 2);

      ezgl::color fontColor = ssInfo.attributes.nameColor;

      drawStreetName(g, {ssInfo.streetName, p1, p2, fontSize, fontColor});

      g->set_color(ssInfo.attributes.color);
      if (isPath) {
         g->set_line_width(pathWidth);
         g->set_color(pathColor);
      }
   }

   // Names for highways
   else if ((kmWidth < 10 && length > 150) & (ssInfo.roadType == "motorway" || ssInfo.roadType == "motorway_link")) {
     

      double fontSize = (metersToPx(g, ssInfo.attributes.thickness) / 2.5);

      ezgl::color fontColor = ssInfo.attributes.nameColor;

      drawStreetName(g, {ssInfo.streetName, p1, p2, fontSize, fontColor});

      g->set_color(ssInfo.attributes.color);

      if (isPath) {
         g->set_line_width(pathWidth);
         g->set_color(pathColor);
      }

   }

   // Draw one way arrows
   else if ((kmWidth < 0.5 && ssInfo.oneWay)
         && (length < 50  && length > 15)) {
      g->set_line_width(2);
      ezgl::point2d a1, a2, a3, a4;
      a1 = {(p2.x + p1.x) / 2, (p2.y + p1.y) / 2}; // Center of the street segment

      // The code below calculates the location of a2, a3, and a4
      if (p1.x == p2.x) { // If line is vertical
         if (p1.y < p2.y) {
            a2 = {a1.x, a1.y + 0.75};
            a3 = {a1.x - 0.25, a1.y + 0.6};
            a4 = {a1.x + 0.25, a1.y + 0.6};
         }
         else {
            a2 = {a1.x, a1.y - 0.75};
            a3 = {a1.x - 0.25, a1.y - 0.6};
            a4 = {a1.x + 0.25, a1.y - 0.6};
         }
      } else { // If line is not vertical
         float slope = (p2.y - a1.y) / (p2.x - a1.x);
         float angle = atan(slope);
         if (p1.x > p2.x) angle += M_PI; // When angle is in quadrant 2 and 3

         // float theta = 0.3947911119; // Calculated manually, required to find a3 and a4
         float theta = 0.2094395;
         float b_to_h = 2.25; // length base to head
         float t_to_h = 1.50; // length tip to head


         a2 = {a1.x + b_to_h * cos(angle), a1.y + b_to_h * sin(angle)};
         a3 = {a1.x + t_to_h * cos(angle + theta), a1.y + t_to_h * sin(angle + theta)};
         a4 = {a1.x + t_to_h * cos(angle - theta), a1.y + t_to_h * sin(angle - theta)};
      }

      g->set_color(ezgl::color());

      // Draw the arrow
      g->draw_line(a1, a2);
      g->fill_poly({a2, a3, a4});

      g->set_line_width(metersToPx(g, ssInfo.attributes.thickness));
      g->set_color(ssInfo.attributes.color);

      if (isPath) {
         g->set_line_width(pathWidth);
         g->set_color(pathColor);
      }

   }
}

float getTurnAngle (ezgl::point2d &start, ezgl::point2d &turn, ezgl::point2d &end){


   float fSlope = (turn.y - start.y) / (turn.x - start.x);
   float sSlope = (end.y - turn.y) / (end.x - turn.x);
   

   //if the two streets are in a straight line
   if(fSlope == sSlope){
      return 0;
   }
   else{

      //Cos law for turn angle
      float c = -1, b = -1, a = -1;

      //get length of c
      float cDeltaX = abs(end.x - start.x);
      float cDeltaY = abs(end.y - start.y);
      
      if(cDeltaY == 0){
         c = cDeltaX;
      }
      else if(cDeltaX == 0){
         c = cDeltaY;
      }
      else{
         c = sqrt(cDeltaX * cDeltaX + cDeltaY * cDeltaY);
      }

      //get length of b
      float bDeltaX = abs(end.x - turn.x);
      float bDeltaY = abs(end.y - turn.y);

      if(cDeltaY == 0){
         c = cDeltaX;
      }
      else if(cDeltaX == 0){
         c = cDeltaY;
      }
      else{
         b = sqrt(bDeltaX * bDeltaX + bDeltaY * bDeltaY);
      }

      //get length of a
      float aDeltaX = abs(turn.x - start.x);
      float aDeltaY = abs(turn.y - start.y);

      if(cDeltaY == 0){
         c = cDeltaX;
      }
      else if(cDeltaX == 0){
         c = cDeltaY;
      }
      else{
         a = sqrt(aDeltaX * aDeltaX + aDeltaY * aDeltaY);
      }
      
         
      float angle = acos((a*a + b*b - c*c) / (2 * a * b));

      return angle;

   }
}


std::vector<ezgl::point2d> getPointsOfTurn (StreetSegmentIdx &cur, StreetSegmentIdx &next){

   SegmentInfo first = curMap->streetSegmentTable[cur];
   SegmentInfo second = curMap->streetSegmentTable[next];
   
   LatLon firstTo = curMap->intersectionInfoTable[first.to].position;
   LatLon firstFrom = curMap->intersectionInfoTable[first.from].position;
   LatLon secondTo = curMap->intersectionInfoTable[second.to].position;
   LatLon secondFrom = curMap->intersectionInfoTable[second.from].position;
   
   ezgl::point2d start, turn, end;

   if((first.to == second.from) | (first.to == second.to)){
         //get turn point
         turn = {curMap->x_from_lon(firstTo.longitude()), curMap->y_from_lat(firstTo.latitude())};

         //get start point
         if(first.numCurvePoints != 0){
            LatLon curStart = getStreetSegmentCurvePoint(cur, first.numCurvePoints - 1);
            start = {curMap->x_from_lon(curStart.longitude()), curMap->y_from_lat(curStart.latitude())};
         }
         else{
            start = {curMap->x_from_lon(firstFrom.longitude()), curMap->y_from_lat(firstFrom.latitude())};
         }
         //get end point
         if(second.from == first.to){
            
            if(second.numCurvePoints != 0){
               LatLon curEnd = getStreetSegmentCurvePoint(next, 0);
               end = {curMap->x_from_lon(curEnd.longitude()), curMap->y_from_lat(curEnd.latitude())};
            }
            else{
               end = {curMap->x_from_lon(secondTo.longitude()), curMap->y_from_lat(secondTo.latitude())};
            }
         }
         else{
            if(second.numCurvePoints != 0){
               LatLon curEnd = getStreetSegmentCurvePoint(next, second.numCurvePoints - 1);
               end = {curMap->x_from_lon(curEnd.longitude()), curMap->y_from_lat(curEnd.latitude())};
            }
            else{
               end = {curMap->x_from_lon(secondFrom.longitude()), curMap->y_from_lat(secondFrom.latitude())};
            }
         }
      }
      else if((first.from == second.from) | (first.from == second.to)){
         //get turn point
         turn = {curMap->x_from_lon(firstFrom.longitude()), curMap->y_from_lat(firstFrom.latitude())};

         //get start point
         if(first.numCurvePoints != 0){
            LatLon curStart = getStreetSegmentCurvePoint(cur, 0);
            start = {curMap->x_from_lon(curStart.longitude()), curMap->y_from_lat(curStart.latitude())};
         }
         else{
            start = {curMap->x_from_lon(firstTo.longitude()), curMap->y_from_lat(firstTo.latitude())};
         }
         //get end point
         if(second.from == first.from){
            
            if(second.numCurvePoints != 0){
               LatLon curEnd = getStreetSegmentCurvePoint(next, 0);
               end = {curMap->x_from_lon(curEnd.longitude()), curMap->y_from_lat(curEnd.latitude())};
            }
            else{
               end = {curMap->x_from_lon(secondTo.longitude()), curMap->y_from_lat(secondTo.latitude())};
            }

         }
         else{

            if(second.numCurvePoints != 0){
               LatLon curEnd = getStreetSegmentCurvePoint(next, second.numCurvePoints - 1);
               end = {curMap->x_from_lon(curEnd.longitude()), curMap->y_from_lat(curEnd.latitude())};
            }
            else{
               end = {curMap->x_from_lon(secondFrom.longitude()), curMap->y_from_lat(secondFrom.latitude())};
            }
         }
      }

   return {start, turn, end};

}

bool turn_penality_check(StreetSegmentIdx &cur, StreetSegmentIdx &next){
   curMap = getCurrentMap();

   SegmentInfo first = curMap->streetSegmentTable[cur];
   SegmentInfo second = curMap->streetSegmentTable[next];

   if(first.streetID == second.streetID){
      return false;
   }

   return true;
}

//Helper function to get slope between two points
float getSlopeTwoPoints(ezgl::point2d &p1, ezgl::point2d &p2) {

   bool swap = false;
   ezgl::point2d temp;

   
   // make sure the second point is further right
   if (p2.x < p1.x) {
      // Swap p1 and p2
      temp = p2;

      p2 = p1;
      p1 = temp;

      swap = true;
   }

   float angle;
   // If line is vertical
   if (p2.x != p1.x) {
      float slope = (p2.y - p1.y) / (p2.x - p1.x);
      angle = atan(slope);
   } else {
      angle = M_PI / 2; // Set angle to pi/2
   }

   if (swap) {
      // Swap p1 and p2
      temp = p2;

      p2 = p1;
      p1 = temp;
   }

   return angle;
}

std::string getLeftOrRight (ezgl::point2d &start, ezgl::point2d &turn, ezgl::point2d &end){

   float x0 = turn.x - start.x;
   float y0 = turn.y - start.y;
   float x1 = end.x - turn.x;
   float y1 = end.y - turn.y;

   float crossProduct = (x0 * y1) - (x1 * y0);

   if(crossProduct > 0){
      return "left";
   }
   else{
      return "right";
   }
   

}

//Create directions from street segments
void getDirections(){

   std::string directionsText = "";

   GtkGrid * grid = GTK_GRID(App->get_object("directionsGrid"));

   gtk_container_foreach(GTK_CONTAINER(grid), (GtkCallback) (GtkWidget*) gtk_widget_destroy, NULL);

   auto cur = currentPath.begin();
   auto next = cur;

   int gridRow = 0;
   std::string direction = "";
   std::string imagePath = "";
   GtkLabel * directionLabel;
   GtkImage * directionImage;

   while(next + 1 != currentPath.end()){

      int length = curMap->streetSegmentVector[*cur].distance;
      int streetIdx = curMap->streetSegmentTable[*cur].streetID;

      //get the street segment on the next street
      while(next + 1 != currentPath.end() && curMap->streetSegmentTable[*next].streetID == streetIdx){
         length += (int) curMap->streetSegmentVector[*next].distance;
         next++;
      }

      bool arrived = 0;
      if(next + 1 == currentPath.end()){
         arrived = 1;
      }

      directionsText += "\nContinue on " + curMap->streetSegmentTable[*cur].streetName + " for " + std::to_string(length) + " meters.\n";

      imagePath = directory_path + "/libstreetmap/resources/ui-icons/straight.svg";
      directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());

      direction = "Continue on " + curMap->streetSegmentTable[*cur].streetName + " for " + std::to_string(length) + " meters.";

      directionLabel = (GtkLabel*) gtk_label_new (direction.c_str());
      gtk_label_set_line_wrap(directionLabel, true);
      gtk_label_set_xalign(directionLabel, 0);
      gtk_grid_insert_row (grid, gridRow);
      gtk_grid_attach(grid, GTK_WIDGET(directionImage), 0, gridRow, 1, 1);
      gtk_grid_attach(grid, GTK_WIDGET(directionLabel), 1, gridRow, 1, 1);
      gridRow++;


      //if arrived, break out of this loop
      if(arrived){
         break;
      }

      //make cur point to the last segment of the cur street before turning
      while(cur + 1 != next){
         cur++;
      }

      //get the start, turn, and end points
      ezgl::point2d start, turn, end;

      std::vector<ezgl::point2d> points = getPointsOfTurn(*cur, *next);
      start = points[0];
      turn = points[1];
      end = points[2];

      
      //get turn angle
      float angle = getTurnAngle(start, turn, end);
      std:: string command = "";

      //Check straight or not
      if(angle == 0){
         command = "Continue straight";
         imagePath = directory_path + "/libstreetmap/resources/ui-icons/straight.svg";
         directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());
      }
      //Check bear or turn
      else{

         std::string turnDirection = getLeftOrRight(start, turn, end);
   
         if(angle > BEAR_ANGLE){
            command = "Bear";
            imagePath = directory_path + "/libstreetmap/resources/ui-icons/bear-" + turnDirection + ".svg";
            directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());
         }
         else if(angle < SHARP_ANGLE){
            command = "Sharp turn";
            imagePath = directory_path + "/libstreetmap/resources/ui-icons/sharp-" + turnDirection + ".svg";
            directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());
         }
         else{
            command = "Turn";
            imagePath = directory_path + "/libstreetmap/resources/ui-icons/" + turnDirection + ".svg";
            directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());
         } 

         //Check left or right
         command += " " + turnDirection;
      }


      //Add direction 
      directionsText += "\n" + command + " on " + curMap->streetSegmentTable[*next].streetName + ".\n";

      direction = command + " on " + curMap->streetSegmentTable[*next].streetName + ".";
      directionLabel = (GtkLabel*) gtk_label_new(direction.c_str());
      gtk_label_set_line_wrap(directionLabel, true);
      gtk_label_set_xalign(directionLabel, 0);
      gtk_grid_insert_row (grid, gridRow);
      gtk_grid_attach(grid, GTK_WIDGET(directionImage), 0, gridRow, 1, 1);
      gtk_grid_attach (grid, GTK_WIDGET(directionLabel), 1, gridRow, 1, 1);
      gridRow++;

      //Increment current pointer
      cur = next;
      
      //If there is only one segment on the last street
      if(next + 1 == currentPath.end()){
         length = (int) curMap->streetSegmentVector[*cur].distance;
         directionsText += "\nContinue on " + curMap->streetSegmentTable[*cur].streetName + " for " + std::to_string(length) + " meters.\n";

         direction = "Continue on " + curMap->streetSegmentTable[*cur].streetName + " for " + std::to_string(length) + " meters.";
         directionLabel = (GtkLabel*) gtk_label_new(direction.c_str());
         gtk_label_set_line_wrap(directionLabel, true);
         gtk_label_set_xalign(directionLabel, 0);
         gtk_grid_insert_row (grid, gridRow);
         gtk_grid_attach(grid, GTK_WIDGET(directionImage), 0, gridRow, 1, 1);
         gtk_grid_attach (grid, GTK_WIDGET(directionLabel), 1, gridRow, 1, 1);
         gridRow++;

         }
   }

   directionsText += "\nYou have arrived.\n";

   direction = "You have arrived.";
   directionLabel = (GtkLabel*) gtk_label_new(direction.c_str());

   imagePath = directory_path + "/libstreetmap/resources/ui-icons/location-indicator-red.svg";
   directionImage = (GtkImage*) gtk_image_new_from_file(imagePath.c_str());
   gtk_label_set_line_wrap(directionLabel, true);
   gtk_label_set_xalign(directionLabel, 0);
   gtk_grid_insert_row (grid, gridRow);
   gtk_grid_attach(grid, GTK_WIDGET(directionImage), 0, gridRow, 1, 1);
   gtk_grid_attach (grid, GTK_WIDGET(directionLabel), 1, gridRow, 1, 1);
   gridRow++;

   // Show all the widgets
   gtk_widget_show_all((GtkWidget*)grid);

   gtk_text_buffer_set_text(GTK_TEXT_BUFFER(App->get_object("printDirection")), directionsText.c_str(), directionsText.length());

}

//
void draw_POI_x(ezgl::renderer *g){
   ezgl::point2d point;
   g -> set_color(ezgl::color(0,0,250));
   std::set<std::string> set1; 
   
   for (int i = 0; i < getNumPointsOfInterest(); i++){
        LatLon latLon = getPOIPosition(i);
        point.x = curMap -> x_from_lon(latLon.longitude());
        point.y = curMap -> y_from_lat(latLon.latitude());
         g -> draw_text(point, getPOIName(i));

   }

}

//Helper function to render in buildings
void draw_buildings(ezgl::renderer *g){

   ezgl::point2d topCorner = g->get_visible_world().top_left();
   ezgl::point2d bottomCorner = g->get_visible_world().bottom_right();

   signed long int x_min = abs(curMap->x_from_lon(locs.min_lon) - topCorner.x )/ curMap->zoneWidth;
   signed long int y_max = abs(curMap->y_from_lat(locs.min_lat) - topCorner.y )/ curMap->zoneWidth;

   signed long int x_max = abs(curMap->x_from_lon(locs.min_lon) - bottomCorner.x) / curMap->zoneWidth;
   signed long int y_min = abs(curMap->y_from_lat(locs.min_lat) - bottomCorner.y) / curMap->zoneWidth;


   //Only render the buildings that are within the window
   for (int i = x_min; i <= x_max; i++){
      for(int j = y_min; j <= y_max; j++){

         std::vector<featureInfo> &cur = curMap->buildingZone[i][j];

         for(int idx = 0; idx < cur.size(); idx++){

            g->set_color(featureColoursLight[6]);

            if(cur[idx].area > 0){ 

               if(cur[idx].points.size() > 1){

                  g->fill_poly(cur[idx].points);
            
               }   
            }
         }
      }
   }

}

//Helper function to render in POI's
void draw_POI(ezgl::renderer *g){
   ezgl::point2d point;
   g -> set_color(ezgl::color(0,0,0));
   g->set_text_rotation(0);
   ezgl::point2d topCorner = g->get_visible_world().top_left();
   ezgl::point2d bottomCorner = g->get_visible_world().bottom_right();

   signed long int x_min = abs(curMap->x_from_lon(locs.min_lon) - topCorner.x )/ curMap->zoneWidth;
   signed long int y_max = abs(curMap->y_from_lat(locs.min_lat) - topCorner.y )/ curMap->zoneWidth;

   signed long int x_max = abs(curMap->x_from_lon(locs.min_lon) - bottomCorner.x) / curMap->zoneWidth;
   signed long int y_min = abs(curMap->y_from_lat(locs.min_lat) - bottomCorner.y) / curMap->zoneWidth;

   

   for (int i = x_min; i <= x_max; i++){
      for(int j = y_min; j <= y_max; j++){
         std::vector<POIIdx> xy = curMap -> poiZone[i][j];
         for (int k = 0; k < xy.size(); k++){
            if (kmWidth < 1){
               LatLon latLon = getPOIPosition(xy[k]);
               point.x = curMap -> x_from_lon(latLon.longitude());
               point.y = curMap -> y_from_lat(latLon.latitude());
               std::string poiTypeData = getPOIType(xy[k]);
               if (poiTypeData == "fast_food") continue;

               if(curMap->surfacePaths.find(poiTypeData) != curMap->surfacePaths.end()){
                  if(gtk_switch_get_active(GTK_SWITCH(App->get_object("explorerToggle"))) 
                        && (restaurants || bars || things_to_do || gas_stations || hospitals)){
                     if(restaurants && (poiTypeData == "restaurant" || poiTypeData == "fast_food")){
                        g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                        g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     }
                     else if(bars && poiTypeData == "bar"){
                        g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                        g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     }
                     else if(things_to_do && (poiTypeData == "museum" || poiTypeData == "beach"  || poiTypeData == "park"  || poiTypeData == "theater"  || poiTypeData == "art_gallery" || poiTypeData == "shopping")){
                        g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                        g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     } 
                     else if(hospitals && (poiTypeData == "hospital" || poiTypeData == "doctors")){
                        g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                        g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     }
                     else if(gas_stations && poiTypeData == "fuel"){
                        g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                        g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     }
                  }
                  else{
                     g -> draw_surface(curMap->surfacePaths[getPOIType(xy[k])], point, 0.03);
                     point.y = point.y + 3;
                     g -> draw_text(point, getPOIName(xy[k]), 50, 50);
                  }
               }
            }
         }
      }
   }
}

//Convert x coordinate to longitude
float lon_from_x(double x){
    return x / (kEarthRadiusInMeters * kDegreeToRadian * cos(((curMap->locationInfo.max_lat + curMap->locationInfo.min_lat) / 2.0 )* kDegreeToRadian));
}

//Convert y coordinate to latitude
float lat_from_y(double y){
    return y / (kEarthRadiusInMeters * kDegreeToRadian);
}

//Helper function to highlight intersections on mouse click
void draw_weather(ezgl::renderer *g){
   g -> set_color(ezgl::color(0,0,0));
   g->set_text_rotation(0);
   ezgl::point2d center = g->get_visible_world().center();
   ezgl::point2d topCorner = g->get_visible_world().top_left();
   ezgl::point2d point;

   point.x = center.x;
   point.y = (center.y + 2 * topCorner.y) / 3;

   ezgl::point2d point_text;
   point_text.x = point.x;
   point_text.y = (center.y + topCorner.y) / 2;

   std::string temperature = std::to_string(curMap->temperature - 273) + "C°";

   g -> draw_surface(curMap->weather_icon_surface, point, 0.3);
   g -> set_font_size(15);
   g -> draw_text(point_text, temperature);
}


void highlight_intersection_id(ezgl::application* /*app*/, GdkEventButton* /*event*/, double x, double y){

   LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
   
   //Get the intersection that needs to be highlighted
   int intersectionId = findClosestIntersection(pos);
   IntersectionInfo infoTable= curMap->intersectionInfoTable[intersectionId];
   
   infoTable.highlight = !infoTable.highlight;
   
   //If already highlighted, unhighlight
   if (infoTable.highlight) {
      if (currentIntersection == intersectionId) { // TOGGLE
         currentIntersection = -1;
         App->refresh_drawing();
         return;
      }
      //To highlight, save to global variable for the current highlighted intersection
      else currentIntersection = intersectionId;

   } else {
      currentIntersection = -1;

      App->refresh_drawing();
      return;
   }

   App->refresh_drawing();
   
}


//Set the current visible window to highlighted intersection
void zoomToIntersection() {
   if (currentIntersection == -1) return;

   ezgl::renderer *g = App->get_renderer();

   LatLon &loc = curMap->intersectionInfoTable[currentIntersection].position;
   float x = curMap->x_from_lon(loc.longitude());
   float y = curMap->y_from_lat(loc.latitude());

   g->set_visible_world({{x - 100, y - 100}, {x + 100, y + 100}});

}

