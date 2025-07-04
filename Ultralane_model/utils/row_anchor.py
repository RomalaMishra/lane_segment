import json
import os
from collections import Counter

label_folder = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/Curvelanesown/label'

json_files = [os.path.join(label_folder, file) for file in os.listdir(label_folder) if file.endswith('.json')]

all_y_coordinates = []
y_list = []
for json_file in json_files:
    with open(json_file, 'r') as file:
        data = json.load(file)
        y_coordinates = data.get('y_coordinates', [])
        all_y_coordinates.extend(y_coordinates)

y_coordinate_counts = Counter(all_y_coordinates)

most_common_y_coordinates = y_coordinate_counts.most_common(30)

for y_coord, _ in most_common_y_coordinates:
    y_coord = int(y_coord * 0.4)
    y_list.append(y_coord)

# print(y_list)
row_anchor = list(set(y_list))
cls_num_per_lane = len(row_anchor)

    
