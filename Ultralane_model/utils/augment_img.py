import os
from keras.preprocessing.image import ImageDataGenerator, img_to_array, load_img

class ImageAugmentor:
    def __init__(self, source_dir, destination_dir, num_augmented_images_per_original=5):
        self.source_dir = source_dir
        self.destination_dir = destination_dir
        self.num_augmented_images_per_original = num_augmented_images_per_original

        os.makedirs(self.destination_dir, exist_ok=True)

        self.datagen = ImageDataGenerator(
            rotation_range=40,
            width_shift_range=0.2,
            height_shift_range=0.2,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True,
            fill_mode='nearest'
        )

    def augment_images(self):
        image_filenames = os.listdir(self.source_dir)

        for filename in image_filenames:
            original_image_path = os.path.join(self.source_dir, filename)

            img = load_img(original_image_path)
            x = img_to_array(img)
            x = x.reshape((1,) + x.shape)

            i = 0
            for batch in self.datagen.flow(x, batch_size=1, save_to_dir=self.destination_dir, save_prefix=filename[:-4], save_format='jpeg'):
                i += 1
                if i >= self.num_augmented_images_per_original:
                    break

        print("Image augmentation complete.")

source_directory = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/lane_only_masked'
destination_directory = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset/Curvelanesown/train'

augmentor = ImageAugmentor(source_directory, destination_directory)
augmentor.augment_images()
