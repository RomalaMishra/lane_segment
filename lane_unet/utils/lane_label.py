import os
import cv2
import numpy as np

class LaneLabeling:
    def __init__(self, base_dir):
        # Initialize the LaneLabeling object with the base directory
        self.base_dir = base_dir
        self.train_dir = os.path.join(self.base_dir, 'train')
        self.label_dir = os.path.join(self.base_dir, 'label2')
        os.makedirs(self.label_dir, exist_ok=True)

    def euclidean_distance(self, point1, point2):
        # Calculate Euclidean distance between two points
        return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

    def process_images(self):
        batch_number = 41
        image_counter = 0

        # Iterate over folders in the training directory
        for folder in os.listdir(self.train_dir):
            folder_path = os.path.join(self.train_dir, folder)

            if os.path.isdir(folder_path):
                # Iterate over images in the current folder
                for image_name in os.listdir(folder_path):
                    if image_name.endswith(('.jpg', '.jpeg', '.png')):
                        image_path = os.path.join(folder_path, image_name)
                        original_image = cv2.imread(image_path)

                        # Resize the image
                        resized_image = cv2.resize(original_image, (1280, 720))

                        # Convert the image to grayscale and apply median blur
                        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
                        gray = cv2.medianBlur(gray, 7)

                        # Apply Canny edge detection
                        edges = cv2.Canny(gray, 50, 150)

                        # Find contours and filter out innermost contours
                        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        innermost = [cnt for cnt in contours if cv2.contourArea(cv2.convexHull(cnt)) > 1000]
                        innermost_contours = [cv2.approxPolyDP(cnt, epsilon=4, closed=True) for cnt in innermost]

                        # Create an empty image
                        labeled_image = np.zeros((720, 1280, 3), dtype=np.uint8)

                        # Draw contours on the empty image
                        cv2.drawContours(labeled_image, innermost_contours, -1, (255, 255, 255), 10)
                        labeled_image = cv2.medianBlur(labeled_image, 1)

                        # Threshold the grayscale and labeled images
                        ret, thresh = cv2.threshold(gray, 70, 255, 0)
                        thresh = cv2.merge((thresh, thresh, thresh))
                        ret1, thresh1 = cv2.threshold(labeled_image, 70, 255, 0)
                        final_image = cv2.bitwise_or(thresh, thresh1)

                        # Update image counter and batch number
                        if image_counter < 10:
                            image_counter += 1
                        else:
                            image_counter = 1
                            batch_number += 1

                        # Create the batch directory
                        batch_dir = os.path.join(self.label_dir, f'batch{batch_number}')
                        os.makedirs(batch_dir, exist_ok=True)

                        # Save the processed image
                        save_path = os.path.join(batch_dir, f'image{image_counter}.jpg')
                        cv2.imwrite(save_path, final_image)

if __name__ == "__main__":
    base_directory = '/home/subun/ros_igvc/igvc_ros-feature-lane_extraction_done/igvc_perception/dl_model/dataset/Curvelanesown'
    lane_labeling = LaneLabeling(base_directory)
    lane_labeling.process_images()
