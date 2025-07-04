import os
import cv2
import numpy as np
import json

class LaneLabeling:
    def __init__(self, base_dir):
        self.base_dir = base_dir
        self.train_dir = os.path.join(self.base_dir, 'train')
        self.label_dir = os.path.join(self.base_dir, 'label')
        os.makedirs(self.label_dir, exist_ok=True)

    def process_images(self):
        for folder in os.listdir(self.train_dir):
            folder_path = os.path.join(self.train_dir, folder)

            if os.path.isdir(folder_path):
                for image_name in os.listdir(folder_path):
                    if image_name.endswith(('.jpg', '.jpeg', '.png')):
                        image_path = os.path.join(folder_path, image_name)
                        original_image = cv2.imread(image_path)

                        resized_image = cv2.resize(original_image, (1280, 720))

                        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

                        edges = cv2.Canny(gray, 50, 150)

                        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                        innermost = [cnt for cnt in contours if cv2.contourArea(cv2.convexHull(cnt)) > 100]
                        innermost_contours = [cv2.approxPolyDP(cnt, epsilon=5, closed=True) for cnt in innermost]

                        all_points = np.vstack(innermost_contours).reshape(-1, 2)
                        all_points[:, 0] = (all_points[:, 0] * original_image.shape[1] / resized_image.shape[1]).astype(int)
                        all_points[:, 1] = (all_points[:, 1] * original_image.shape[0] / resized_image.shape[0]).astype(int)

                        data = {
                            "image_name": image_path,
                            "x_coordinates": all_points[:, 0].tolist(),
                            "y_coordinates": all_points[:, 1].tolist()
                        }

                        json_data = json.dumps(data, indent=2)

                        json_filename = os.path.join(self.label_dir, f"{folder}_{image_name.replace('.', '_')}.json")
                        with open(json_filename, "w") as json_file:
                            json_file.write(json_data)

                        print(f"Processed: {json_filename}")

if __name__ == "__main__":
    base_directory = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/Curvelanesown'
    lane_labeling = LaneLabeling(base_directory)
    lane_labeling.process_images()
