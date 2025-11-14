# **MetaQuest 3 – Real-World Data Extraction & Mapping**

This project provides tools to extract, visualize, and generate spatial maps from real-world data captured for projection in **MetaQuest 3**.

---

## ** extract_data.py**

### **Run Command**
```bash
python3 extract_data.py --output_dir {output_directory}
```

### **Description**
This script extracts the following for each captured frame:

- **RGB image**
- **Depth map**
- **Point cloud**
- **Camera pose estimation**

All results are saved inside the specified output directory.

---

## ** visualize_data.py**

### **Run Command**
```bash
python3 visualize_data.py --input_folder {path_to_point_cloud}
```

### **Description**
This tool visualizes the generated point clouds (`.ply` or `.obj`) using **Open3D**.

---

## ** generate_spatial_map.py**

### **Run Command**
```bash
python3 generate_spatial_map.py \
    --output_dir {output_directory} \
    --map_range {depth_range_for_map} \
    --map_type {Mesh or Pointcloud} \
    --map_resolution {desired_resolution}
```

### **Description**
This script generates a **continuous spatial map** of the environment using the extracted frames.

You can export:

- **Mesh**
- **Point cloud**

The resulting map is stored for future use, such as scene reconstruction or VR/AR applications.

---
