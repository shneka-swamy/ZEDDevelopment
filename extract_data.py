# This code is inspired from streo labs -- zed-yolo

import pyzed.sl as sl
import cv2
import argparse
from pathlib import Path

def argparser():
    parser = argparse.ArgumentParser("Extract images and point cloud in required format")
    parser.add_argument('--output_dir', help="Path to the output folder")
    return parser.parse_args()

def initialize_camera():
    # Initialize the camera
    print("Initializing Camera...")
    zed = sl.Camera()
    input_type = sl.InputType()

    init_params = sl.InitParameters(input_t = input_type, svo_real_time_mode=True)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    # Minimum depth information is at 120 mm in zed camera -- hence cannot be used
    init_params.depth_maximum_distance = 700

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Camera Open: {repr:(status)}. Exit Program.")
        exit()
    runtime_params = sl.RuntimeParameters()
    print("Initializing Camera... DONE")

    return zed, runtime_params

def record_values(zed, runtime_params, output):
    # Creating data containers
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    i = 0
    while i < 10:
        # Can grab new keyframe
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Convert to numpy array
            image_np = image.get_data()
            depth_np  = depth.get_data()

            cv2.imwrite(f'{output}Image/image_{i}.png', image_np)
            cv2.imwrite(f'{output}Depth/depth_{i}.png', depth_np)
            if point_cloud.write(f'{output}PointCloud/point_cloud_{i}.ply') != sl.ERROR_CODE.SUCCESS:
                print("Error writing point cloud data")
                exit()
        i += 1
    print("Stored 10 images and depth files -- Closing zed")
    zed.close()
    print("Camera closed successfully")

def main():
    args = argparser()
    zed, runtime_params = initialize_camera()

    # Creating folders to store the images
    output_image = f"{args.output_dir}Image/"
    output_depth = f"{args.output_dir}Depth/"
    output_point_cloud = f"{args.output_dir}PointCloud/"

    Path(output_image).mkdir(parents=True, exist_ok=True)
    Path(output_depth).mkdir(parents=True, exist_ok=True)
    Path(output_point_cloud).mkdir(parents=True, exist_ok=True)

    record_values(zed, runtime_params, args.output_dir)


if __name__ == '__main__':
    main()