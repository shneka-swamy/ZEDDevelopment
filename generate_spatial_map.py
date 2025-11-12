import pyzed.sl as sl
import argparse
from pathlib import Path

def argparser():
    parser = argparse.ArgumentParser("Get the spatial map of the environment")
    parser.add_argument('--output_dir', help="Path to the output folder")
    parser.add_argument('--map_range', help="Specify the resolution [Near, Medium, Far] of the map required", default='Far')
    parser.add_argument('--map_type', help="Choose the type of map [mesh, point cloud] to be used", default='Mesh')
    parser.add_argument('--map_resolution', help="Specify map resolution [Low, medium, High]", default='Low')
    return parser.parse_args()

# Start the camera
def initialize_camera():
    print("Initializing Camera...")
    zed = sl.Camera()
    
    init_params = sl.InitParameters()

    # Since positional tracking is used -- it is recommended to us hd 720
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Camera not Open: {repr:(status)}. Exit Program")
        exit()
    return zed

# Run the tracking module
def initialize_tracking(zed):
    tracking_param = sl.PositionalTrackingParameters()
    err = zed.enable_positional_tracking(tracking_param)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"enable tracking failed: {err}")


# Run the spatial mapping for the camera
def initialize_spatial_mapping(zed, map_resolution, map_range, map_type):
    mapping_params = sl.SpatialMappingParameters()

    # Map resolution
    if map_resolution == 'Low':
        mapping_params.resolution_meter = mapping_params.get_resolution_preset(sl.MAPPING_RESOLUTION.LOW)
    elif map_resolution == 'Medium':
        mapping_params.resolution_meter = mapping_params.get_resolution_preset(sl.MAPPING_RESOLUTION.MEDIUM)
    else:
        assert map_resolution == 'High', "Resolution can be Low, Medium, High"
        mapping_params.resolution_meter = mapping_params.get_resolution_preset(sl.MAPPING_RESOLUTION.HIGH)

    # Map range 
    if map_range == 'Near':
        mapping_params.range_meter = mapping_params.get_range_preset(sl.MAPPING_RANGE.NEAR)
    elif map_range == 'Medium':
        mapping_params.range_meter = mapping_params.get_range_preset(sl.MAPPING_RANGE.MEDIUM)
    else:
        assert map_range == 'Far', "Resolution can be Near, Medium, Far others are not implemented"
        try:
            mapping_params.range_meter = mapping_params.get_range_preset(sl.MAPPING_RANGE.FAR)
        except AttributeError:
            mapping_params.range_meter = 10.0

    # Map type
    if map_type == 'Mesh':
        mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH
    else:
        assert map_type == 'Point_cloud', "Other representation is not provided"
        mapping_params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD

    err = zed.enable_spatial_mapping(mapping_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"enable spatial mapping failed: {err}")

def develop_mesh(zed, output_dir):
    mesh = sl.Mesh()
    timer = 0
    while timer < 30:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            timer += 1
    zed.extract_whole_spatial_map(mesh)
    mesh.apply_texture()
    mesh.save(f"{output_dir}Mesh_full.obj")

# To be implemented
def develop_point_cloud(zed):
    pass

def main():
    args = argparser()
    output_dir = f"{args.output_dir}Mesh/"


    zed = initialize_camera()
    initialize_tracking(zed)
    initialize_spatial_mapping(zed, args.map_resolution, args.map_range, args.map_type)

    if args.map_type == 'Mesh':
        develop_mesh(zed, output_dir)
    else:
        print("To be implemented")
        exit()

    zed.disable_spatial_mapping()
    zed.disable_tracking()
    zed.close()

if __name__ == '__main__':
    main()