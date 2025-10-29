import argparse
import open3d as o3d
import glob

def argparser():
    parser = argparse.ArgumentParser("Visualize the point cloud data by ZED")
    parser.add_argument('--input_folder', help="Path to the input folder", default='./zed_files/PointCloud/')
    return parser.parse_args()

def display_point_cloud(file):
    pcd = o3d.io.read_point_cloud(file)
    print(pcd)
    o3d.visualization.draw_geometries(
        [pcd],
        window_name="Open3D Point Cloud",
        width=1280, height=720,
        left=50, top=50
    )

def main():
    args = argparser()
    files = glob.glob(f"{args.input_folder}/fused_point_cloud.ply")
    for file in files:
        display_point_cloud(file)

if __name__ == '__main__':
    main()