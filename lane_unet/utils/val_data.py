import pandas as pd
from sklearn.model_selection import train_test_split

# Define the path to the CSV file
csv_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset/dataset.csv'

# Read the CSV file into a DataFrame
dataset_df = pd.read_csv(csv_path)

# Extract features (x) and labels (y) from the DataFrame
x = dataset_df['image']
y = dataset_df['label']

# Split the dataset into training and validation sets
x_train, x_val, y_train, y_val = train_test_split(x, y, test_size=0.2, random_state=42)

# Create DataFrames for the training and validation sets
train_df = pd.DataFrame({'image': x_train, 'label': y_train})
val_df = pd.DataFrame({'image': x_val, 'label': y_val})

# Define the paths for the training and validation CSV files
train_csv_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset/train.csv'
val_csv_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset/val.csv'

# Save the training and validation DataFrames to CSV files
train_df.to_csv(train_csv_path, index=False)
val_df.to_csv(val_csv_path, index=False)

