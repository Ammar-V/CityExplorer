from crawler import *
from geopy.geocoders import Nominatim

import json
import math
import numpy as np
import cv2
from colour import Color


filename = "/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/data/toronto_pois.txt"
out_filename = "/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/data/toronto_pois_address.txt"

geolocator = Nominatim(user_agent="app")
# location = geolocator.reverse("43.65816701825535, -79.39963940266877")
# address = location.raw['address']
# print(address)

# # building = address['building']
# building = 'tim hortons'
# house_number = address['house_number']
# road = address['road']
# # city = address['city']
# city = 'Toronto'
# state = address['state']
# postcode = address['postcode']

# poi_location = f'({building}) {house_number} {road}, {city}, {state}, {postcode}'
# print(poi_location)
# # poi_location = '43.65254733000179, -79.39865771330564'
# location_info = get_populartimes_by_formatted_address(poi_location)
# print(location_info)

# day, pop_times = get_popularity_for_day(location_info['populartimes'])
# print(pop_times)

def latlon_to_address():
    with open(filename, "r") as inFile:
        with open(out_filename, "w") as outFile:
            count = 0
            for line in inFile:
                elements = line.split(',')

                # Get the raw info from file
                name = elements[1]
                lat = elements[2]
                lon = elements[3]
                print(f'{count}: {name}')

                # Get the address
                try:
                    location = geolocator.reverse(f'{lat}, {lon}')
                    address = location.raw['address']
                except:
                    location = ''

                # Format the address
                house_number = ''
                road = ''
                state = ''
                postcode = ''
                if 'house_number' in address.keys():
                    house_number = address['house_number'] + ' '
                if 'road' in address.keys():
                    road = address['road'] + ', '
                if 'state' in address.keys():
                    state = address['state'] + ', '
                if 'postcode' in address.keys():
                    postcode = address['postcode']
                city = 'Toronto, '

                formatted = f'({name}) {house_number}{road}{city}{state}{postcode}\n'

                outFile.write(formatted)

                count += 1


        outFile.close()

    inFile.close()

data_filename = "/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/data/toronto_heatmap.json"

def address_to_heatmap():
    with open(filename, "r") as inFile:
        with open(out_filename, "r") as addFile:
            count = 0
            # with open(data_filename, "a") as outFile:
            #     outFile.write('[')
            # outFile.close()
            for ogLine in inFile:
                if count < 11122:
                    count += 1
                    continue
                address = addFile.readline()[:-2]

                # Get the raw info from og file
                elements = ogLine.split(',')
                name = elements[1]
                lat = elements[2]
                lon = elements[3][:-2]

                location_info = get_populartimes_by_formatted_address(address)
                try:
                    info = location_info['populartimes']
                except:
                    info = ""

                out_data = {
                    "location": f"{lat}, {lon}",
                    "name": name,
                    "address": address,
                    "popular_times": info,
                    "data": location_info
                }


                with open(data_filename, "a") as outFile:
                    outFile.write(json.dumps(out_data))
                    outFile.write(',\n')
                outFile.close()

                print(f'{count}:', '')
                print(out_data)

                # if count > 2000:
                #     break
                count += 1


        with open(data_filename, "a") as outFile:
            outFile.write('{}]')
        outFile.close()
        addFile.close()
    inFile.close()

small_filename = "/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/data/toronto_heatmap_small.json"

def reduce_data_size ():

    with open(data_filename, 'r') as inFile:
        start = True
        count = 0
        for line in inFile:
            # if count < 19915:
            #     start = False
            #     count += 1
            #     print(count)
            #     continue
            print(count)
            data = json.loads(line[1 if start else 0:len(line) - 2])

            if data['popular_times'] != '':
                clean_data = {
                    'id': count,
                    'location': data['location'],
                    'name': data['name'],
                    'address': data['address'],
                    'popular_times': data['popular_times']
                }

                with open(small_filename, 'a') as smallFile:
                    if start:
                        smallFile.write('{')
                    smallFile.write(json.dumps(clean_data))
                    smallFile.write(',\n')
                smallFile.close()
            start = False
            count += 1
    inFile.close()



# reduce_data_size()


min_lon = -79.79
max_lon = -79
min_lat = 43.48
max_lat = 43.92

earth_radius_m = 6372797.560856
degree_to_radian = 0.017453292519943295769236907684886

def x_from_lon(lon):
    return earth_radius_m * lon * degree_to_radian \
        * math.cos((max_lat + min_lat) / 2.0 * degree_to_radian)

def y_from_lat(lat):
    return earth_radius_m * lat * degree_to_radian


class Zone:
    def __init__(self):
        self.count = 0
        self.times = np.array([], dtype=np.int)
    def __str__(self):
        return f'Count: {self.count}, Popularity Times: {self.times}'

def create_heatmap():
   
    # Load data into an object
    data = []
    with open(small_filename, "r") as inFile:
        start = True
        count = 0
        for line in inFile:
            data.append(json.loads(line[1 if start else 0:len(line) - 2]))
            start = False
            count += 1

    inFile.close()

    # Data loaded
    min_x = x_from_lon(min_lon)
    min_y = y_from_lat(min_lat)

    max_x = x_from_lon(max_lon)
    max_y = y_from_lat(max_lat)

    dx = 25  # 50 meter width
    dy = 25 # 50 meter height

    width = (int)(abs(max_x - min_x) / dx) + 1
    height = (int)(abs(max_y - min_y) / dy) + 1

    print(height, width)

    # Decide dimensions of heatmap

    heatmap = np.ndarray((width, height), dtype=Zone)
    errors = 0
    # Just do Monday at 12
    for i in range(0, len(data)):
        # Popularity of the place
        popularity = data[i]['popular_times'][0]['data'][12]
       
        # Get lat and lon
        location = data[i]['location'].split(',')

        try:
            lat = (float)(location[0])
            lon = (float)(location[1])
        except:
            errors += 1
            continue

        col = (int)(abs(min_x - x_from_lon(lon)) // dx) 
        row = (int)(abs(min_y - y_from_lat(lat)) // dy) 

        # Add to the zone
        zone = Zone() if heatmap[col, row] is None else heatmap[col, row]
        zone.count += 1
        zone.times = np.append(zone.times, popularity)
        heatmap[col, row] = zone

   
    red = Color("blue")
    # colors = list(red.range_to(Color("red"),10))
    colors = [
    (255, 0, 0),
    (255, 69, 0),
    (255, 101, 0),
    (255, 128, 0),
    (255, 151, 0),
    (255, 173, 0),
    (255, 193, 33),
    (255, 213, 60),
    (255, 232, 85),
    (255, 250, 111)]

    # Get an image of a heatmap
    heatmap_img = np.ndarray((width, height, 4), 'uint8')
    for i in range(0, width):
        for j in range(0, height):
            if heatmap[i, j] is not None:
                val = np.mean(heatmap[i, j].times)
                c = colors[min(9, (int)(val // 10))]
                red, green, blue = c
                heatmap_img[i, j] = (blue, green, red, 255)
                # print(i, j, heatmap_img[i, j, :])
            else:
                # heatmap_img[i, j] = (10, 10, 10, 10)
                heatmap_img[i, j] = (0, 0, 0, 0)

    
    
    # Swap axis and flip imagels
    heatmap_img = np.transpose(heatmap_img, (1, 0, 2))
    heatmap_img = np.flip(heatmap_img, 0)
    # heatmap_img = np.flip(heatmap_img, 1)

    image = heatmap_img
    # Smooth image
    # image[:, :, 2] = np.clip(image[:, :, 2] + 50.0, 0, 255)
    image = cv2.GaussianBlur(heatmap_img, (3, 3), 0)
    image[:, :, 2:] = np.clip(image[:, :, 2:] * 1.5, 0, 255) # Brighter

    # image = cv2.GaussianBlur(heatmap_img, (3, 3), 0)
    # image[:, :, :] = np.clip(image[:, :, :] * 2, 0, 255)
    # image = cv2.GaussianBlur(heatmap_img, (3, 3), 0)
    # image[:, :, :] = np.clip(image[:, :, :] * 1.5, 0, 255) # Brighter
    # image[:, :, 2] = np.clip(image[:, :, 2] + 20.0, 0, 255) # More red
    # image[:, :, 0:2] = np.clip(image[:, :, 0:2] + 20.0, 0, 255) # more blue
    # image = cv2.GaussianBlur(heatmap_img, (5, 5), 0)
    # image[:, :, :] = np.clip(image[:, :, :] * 2.3, 0, 255) # Brighter

    image = image.astype('int64')

    for i in range(0, image.shape[0]):
        for j in range(1, image.shape[1]):
            if (image[i, j, 2]) > (image[i, j, 0] + image[i, j, 1]):
                image[i, j, 2] += 30
                # image[i, j, 3] += 30
            # image[i, j, 3] += 10

    image = np.clip(image[:, :, :], 0, 255)
    







    # image = cv2.GaussianBlur(heatmap_img, (5, 5), 0)

    # image = cv2.rectangle(image, (0, 0), (width - 1, height - 1), (0, 0, 255, 255), 10)
    
    # image = heatmap_img



    cv2.imwrite("/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/test.png", image)

    # Write to a .txt file
    with open("/nfs/ug/homes-4/w/wangcin5/ece297/work/mapper/libstreetmap/src/heatmap/heatmap.txt", "w") as imgOut:
        for i in range(0, height):
            for j in range(0, width):
                # print(image[i, j])
                imgOut.write(f"{image[i, j, 2]} {image[i, j, 1]} {image[i, j, 0]} {image[i, j, 3]} ")
            imgOut.write("\n")
    imgOut.close()




    print(errors)





create_heatmap()