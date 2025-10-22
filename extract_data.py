# This code is inspired from streo labs -- zed-yolo

import pyzed.sl as sl
import cv2
import numpy as np
import argparse
import open3d as o3d
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

    pos_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(pos_params)
    
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Camera Open: {repr:(status)}. Exit Program.")
        exit()
    runtime_params = sl.RuntimeParameters()
    print("Initializing Camera... DONE")

    return zed, runtime_params

def normalize_depth_data(depth_np):
    depth_np == np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
    depth_min = 0.5
    depth_max = 10
    depth_clipped = np.clip(depth_np, depth_min, depth_max)
    depth_normalized = (255 * (1 - (depth_clipped - depth_min) / (depth_max - depth_min))).astype(np.uint8)
    return depth_normalized    

def record_values(zed, runtime_params, output):
    print("Inside record values")
    # Creating data containers
    image = sl.Mat()
    depth = sl.Mat()
    cam_pose = sl.Pose()
    point_cloud = sl.Mat()
    fused_pcd = o3d.geometry.PointCloud()

    i = 0
    while i < 10:
        print(f"I is: {i}")
        # Can grab new keyframe
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Convert to numpy array
            image_np = image.get_data()
            depth_np  = depth.get_data()
            depth_normalized = normalize_depth_data(depth_np)

            point_cloud_np = point_cloud.get_data()
            point_cloud_val = point_cloud_np[..., :3]
            mask = np.isfinite(point_cloud_val).all(-1)
            pts = point_cloud_val[mask]

            cv2.imwrite(f'{output}Image/image_{i}.png', image_np)
            cv2.imwrite(f'{output}Depth/depth_{i}.png', depth_normalized)
            if point_cloud.write(f'{output}PointCloud/point_cloud_{i}.ply') != sl.ERROR_CODE.SUCCESS:
                print("Error writing point cloud data")
                exit()
        track_status = zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
        if track_status == sl.POSITIONAL_TRACKING_STATE.OK:
            T_wc = np.array(cam_pose.get_matrix)
            pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
            pts_world = (T_wc @ pts_h.T).T[:, :3]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts_world)
            fused_pcd += pcd.voxel_down_sample(0.02)


        i += 1
    print("Stored 10 images and depth files -- Closing zed")
    
    o3d.io.write_point_cloud(f'{output}PointCloud/fused_point_cloud.ply', fused_pcd)
    print("Saved fused point cloud as fused_point_cloud.ply")

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