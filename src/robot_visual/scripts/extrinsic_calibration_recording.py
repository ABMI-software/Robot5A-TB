import cv2
import os

# Path for saving images (common folder for both cameras)
base_path = "/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/extrinsic_images"

# Create the directory if it does not exist
os.makedirs(base_path, exist_ok=True)

def get_next_index(folder_path, prefix):
    """
    Get the next available index for images in the specified folder.
    Args:
        folder_path (str): The folder to check for existing images.
        prefix (str): The prefix of the image files to search for.
    Returns:
        int: The next available index for image files.
    """
    files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
    if not files:
        return 0
    indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
    return max(indices) + 1

def main():
    # Detect next available indices for each camera prefix in the common folder
    camera_1_count = get_next_index(base_path, "camera_1")
 

    print(f"Starting from index {camera_1_count} for Camera 1 images.")
    

    # Open cameras
    cap_1 = cv2.VideoCapture(0)  # First camera

    if not cap_1.isOpened():
        print("Error: Camera 1 not found.")
        return

    # Set resolution
    cap_1.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap_1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    

    print("Press 'a' to save a picture from Camera 1.")
    print("Press 'q' to quit.")

    while True:
        # Capture frames from both cameras
        ret_1, frame_1 = cap_1.read()


        if ret_1:
            cv2.imshow("Camera 1", frame_1)
        else:
            print("Warning: Empty frame from Camera 1.")



        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('a'):  # Save image from Camera 1
            filename = os.path.join(base_path, f"camera_1_{camera_1_count:03d}.jpg")
            cv2.imwrite(filename, frame_1)
            print(f"Saved: {filename}")
            camera_1_count += 1

        elif key == ord('q'):  # Quit
            print("Quitting...")
            break

    # Release cameras and close windows
    cap_1.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
