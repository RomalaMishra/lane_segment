import os

train_dir = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/Curvelanesown/train'

output_file = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/Curvelanesown/train.txt'

with open(output_file, 'w') as f:
    for root, dirs, files in os.walk(train_dir):
        for file in files:
            file_path = os.path.join(root, file)
            relative_path = os.path.relpath(file_path, train_dir)
            
            f.write(relative_path + '\n')

print(f"train.txt file has been created at: {output_file}")
