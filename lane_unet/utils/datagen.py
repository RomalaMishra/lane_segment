import os
import csv

class DatasetCSVCreator:
    def __init__(self, dataset_folder):
        # Initialize the DatasetCSVCreator object with the dataset folder
        self.dataset_folder = dataset_folder
        self.image_folder = os.path.join(self.dataset_folder, 'image')
        self.label_folder = os.path.join(self.dataset_folder, 'label')
        self.csv_file_path = os.path.join(self.dataset_folder, 'dataset.csv')

    def create_csv(self):
        # Get list of batch folders in the image folder
        batch_folders = sorted(os.listdir(self.image_folder))

        # Create or overwrite the CSV file
        with open(self.csv_file_path, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['image', 'label'])  # Header row

            # Iterate through batch folders
            for batch_folder in batch_folders:
                image_batch_path = os.path.join(self.image_folder, batch_folder)
                label_batch_path = os.path.join(self.label_folder, batch_folder)

                # Get list of image files in the current batch
                image_files = sorted(os.listdir(image_batch_path))

                # Iterate through image files
                for image_file in image_files:
                    # Construct the paths for the image and corresponding label
                    image_path = os.path.join(image_batch_path, image_file)
                    label_file = 'image' + image_file[6:]  # Assuming label files start with 'mask'
                    label_path = os.path.join(label_batch_path, label_file)

                    # Write the image and label paths to the CSV file
                    csv_writer.writerow([image_path, label_path])

        print(f"CSV file created at: {self.csv_file_path}")

if __name__ == "__main__":
    dataset_folder_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset'
    csv_creator = DatasetCSVCreator(dataset_folder_path)
    csv_creator.create_csv()
