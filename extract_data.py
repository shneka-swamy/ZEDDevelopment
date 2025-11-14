# This code is inspired from streo labs -- zed-yolo
# Extracts RGB image, depth image, tracking pose data.

import pyzed.sl as sl
import cv2
import numpy as np
import argparse
import open3d as o3d
from pathlib import Path
import os
from scipy.spatial.transform import Rotation as R

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
    pos_params = sl.PositionalTrackingParameters()
    err = zed.enable_positional_tracking(pos_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"enable_positional_tracking failed: {err}")

    print("Initializing Camera... DONE")

    return zed, runtime_params

def normalize_depth_data(depth_np):
    depth_np = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
    valid = np.isfinite(depth_np) & (depth_np > 0)
    vmin, vmax = np.percentile(depth_np[valid], (2, 98))
    depth_norm = np.zeros_like(depth_np, dtype=np.uint8)
    depth_norm[valid] = (255 * (1 - (np.clip(depth_np[valid], vmin, vmax) - vmin) / (vmax - vmin))).astype(np.uint8)

    return depth_norm   

def record_values(zed, runtime_params, output):
    print("Inside record values")

    # Make sure output dirs exist
    os.makedirs(os.path.join(output, "Image"), exist_ok=True)
    os.makedirs(os.path.join(output, "Depth"), exist_ok=True)
    os.makedirs(os.path.join(output, "PointCloud"), exist_ok=True)

    # Data containers
    image = sl.Mat()
    depth = sl.Mat()
    cam_pose = sl.Pose()
    point_cloud = sl.Mat()
    fused_pcd = o3d.geometry.PointCloud()

    i = 0
    while i < 10:
        print(f"I is: {i}")
        if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
            continue

        # Retrieve frame data
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Convert to numpy
        image_np = image.get_data()
        depth_np  = depth.get_data()
        depth_normalized = normalize_depth_data(depth_np)

        pc_np = point_cloud.get_data()          
        pts_cam = pc_np[..., :3]
        mask = np.isfinite(pts_cam).all(-1)
        pts = pts_cam[mask].astype(np.float32)   # Nx3

        # Save per-frame artifacts 
        cv2.imwrite(os.path.join(output, "Image", f"image_{i}.png"), image_np)
        cv2.imwrite(os.path.join(output, "Depth", f"depth_{i}.png"), depth_normalized)
        if point_cloud.write(os.path.join(output, "PointCloud", f"point_cloud_{i}.ply")) != sl.ERROR_CODE.SUCCESS:
            print("Error writing point cloud data")
            break

        state = zed.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
        if state == sl.POSITIONAL_TRACKING_STATE.OK and pts.size > 0:
            # Build 4x4 T from translation + quaternion
            t_sl = cam_pose.get_translation(sl.Translation()) 
            tx, ty, tz = t_sl.get()                           

            qx, qy, qz, qw = cam_pose.get_orientation().get()
            R_wc = R.from_quat([qx, qy, qz, qw]).as_matrix().astype(np.float32)

            T_wc = np.eye(4, dtype=np.float32)
            T_wc[:3, :3] = R_wc
            T_wc[:3,  3] = np.array([tx, ty, tz], dtype=np.float32)

            # Transform points to world
            pts_h = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=np.float32)])
            pts_world = (T_wc @ pts_h.T).T[:, :3]

            # Accumulate (with light downsample)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts_world)
            fused_pcd += pcd.voxel_down_sample(0.02)

            num_points = np.asarray(fused_pcd.points).shape[0]
            print(f"Total points in fused cloud: {num_points}")
        else:
            print("Tracking not OK this frame:", state)

        i += 1

    # Save fused cloud
    fused_path = os.path.join(output, "PointCloud", "fused_point_cloud.ply")
    o3d.io.write_point_cloud(fused_path, fused_pcd)
    print(f"Saved fused point cloud as {fused_path}")

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