import numpy as np
from igvc_perception.lane_unet.utils.model import unet
import pandas as pd
from tensorflow.keras.callbacks import EarlyStopping
from skimage import io

input_size = (128, 128, 3)
batch_size = 32

model = unet(pretrained_weights=None, input_size=input_size)

# Load CSV files
train_csv_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset/train.csv'
val_csv_path = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/lane_unet/Dataset/val.csv'

# Read CSV files
train_df = pd.read_csv(train_csv_path)
val_df = pd.read_csv(val_csv_path)

# Extract file paths from CSV
train_images = train_df['image'].tolist()
train_masks = train_df['label'].tolist()

val_images = val_df['image'].tolist()
val_masks = val_df['label'].tolist()

def generator(images, masks, batch_size):
    while True:
        for i in range(0, len(images), batch_size):
            batch_images = []
            batch_masks = []

            for j in range(i, min(i + batch_size, len(images))):
                image = io.imread(images[j]) / 255.0
                mask = io.imread(masks[j], as_gray=True) / 255.0

                batch_images.append(image)
                batch_masks.append(mask)

            yield np.array(batch_images), np.array(batch_masks)

train_generator = generator(train_images, train_masks, batch_size)
val_generator = generator(val_images, val_masks, batch_size)

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

early_stopping = EarlyStopping(monitor='val_accuracy', patience=5, restore_best_weights=True)

model.fit(
    train_generator,
    steps_per_epoch=len(train_images) // batch_size,
    epochs=100,
    validation_data=val_generator,
    validation_steps=len(val_images) // batch_size,
    callbacks=[early_stopping],
    verbose=1
)

model.save_weights('lane_model.h5')
