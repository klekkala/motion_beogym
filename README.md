# motion_beogym

This is a Python program that converts point cloud files into 3D elevation images for visualization in Open3D.

## Description

At the beginning of the program, the point cloud file was filtered and the yz coordinates were exchanged. When running the specific point cloud file, please pay attention to the corresponding coordinate index of xyz. Some subsequent operations have been annotated and explained in the code. 

## Getting Started

### Dependencies and Installing

```bash
pip install open3d numpy matplotlib scipy
```

Directly run the Python file of 3d_elevation_map, followed by your specific point cloud file path.

```bash
python 3d_elevation_map.py surfaceMap.pcd
```