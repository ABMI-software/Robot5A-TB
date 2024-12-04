#!/bin/bash

# Directories
CAMERA_1_DIR="/home/eliott-frohly/Robot5A_BT/src/robot_visual/config/camera_1_images"
CAMERA_2_DIR="/home/eliott-frohly/Robot5A_BT/src/robot_visual/config/camera_2_images"

# Rename camera_2_xxx.jpg in CAMERA_1_DIR to camera_1_xxx.jpg
for file in "$CAMERA_1_DIR"/camera_2_*.jpg; do
    if [ -e "$file" ]; then
        base=$(basename "$file")
        new_name="${base/camera_2_/camera_1_}"
        mv "$file" "$CAMERA_1_DIR/$new_name"
        echo "Renamed $file to $CAMERA_1_DIR/$new_name"
    fi
done

# Rename camera_1_xxx.jpg in CAMERA_2_DIR to camera_2_xxx.jpg
for file in "$CAMERA_2_DIR"/camera_1_*.jpg; do
    if [ -e "$file" ]; then
        base=$(basename "$file")
        new_name="${base/camera_1_/camera_2_}"
        mv "$file" "$CAMERA_2_DIR/$new_name"
        echo "Renamed $file to $CAMERA_2_DIR/$new_name"
    fi
done

echo "Renaming complete!"
