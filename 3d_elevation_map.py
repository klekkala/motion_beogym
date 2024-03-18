import argparse
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.spatial import Delaunay

parser = argparse.ArgumentParser(description="Load .pcd")

parser.add_argument("pcd_file_path", help="input .pcd path")

args = parser.parse_args()

pcd = o3d.io.read_point_cloud(args.pcd_file_path)

print("Number of points", len(pcd.points))
print("Point cloud type:", type(pcd))

point_cloud_np = np.asarray(pcd.points)

# Filter and retain points on the z-coordinate that are less than or equal to 1.5
# Exchange y and z coordinates in point cloud files
points = point_cloud_np[point_cloud_np[:, 1] <= 1.5]
points[:, [1, 2]] = points[:, [2, 1]]

print("NumPy array shape:", points.shape)


# Traverse the filtered point cloud and extract the highest point from the dictionary
xy_to_z = {}
scale = 0.8  # Define a scaling factor to determine how points are mapped to the grid
for point in points:
    xy_key = (np.floor(point[0] * scale), np.floor(point[1] * scale))
    z_value = point[2]
    if xy_key not in xy_to_z or z_value > xy_to_z[xy_key][0]:
        xy_to_z[xy_key] = (z_value, point)

highest_points = np.array([val[1] for val in xy_to_z.values()])


# Create a new point cloud object
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(highest_points)
filtered_pcd.paint_uniform_color([0.2, 0.2, 0.2])

min_bound = filtered_pcd.get_min_bound()
max_bound = filtered_pcd.get_max_bound()

z_max = np.max(highest_points[:, 2])
z_min = np.min(highest_points[:, 2])


# Add a grid at the lowest point of z
grid_scale = 0.1
x_size = max_bound[0] - min_bound[0]
y_size = max_bound[1] - min_bound[1]
x_spaces = np.linspace(min_bound[0], max_bound[0], num=int(x_size * grid_scale))
y_spaces = np.linspace(min_bound[1], max_bound[1], num=int(y_size * grid_scale))

lines = []
for i, x in enumerate(x_spaces):
    for j, y in enumerate(y_spaces):
        if i < len(x_spaces) - 1:
            lines.append([[x, y, z_min], [x_spaces[i + 1], y, z_min]])
        if j < len(y_spaces) - 1:
            lines.append([[x, y, z_min], [x, y_spaces[j + 1], z_min]])

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.array(lines).reshape(-1, 3)),
    lines=o3d.utility.Vector2iVector(np.arange(0, len(lines) * 2).reshape(-1, 2))
)

# Create a cube from 0 to the z-value for points with z-values greater than 0
voxel_size = 0.8 
colormap = plt.get_cmap('jet')
cubes = []
for point in highest_points:
    if point[2] > 0:
        height = point[2] - z_min
        color = colormap((height) / (z_max - z_min))[:3]

        cube = o3d.geometry.TriangleMesh.create_box(width=voxel_size,
                                                    height=voxel_size,
                                                    depth=height)
        cube.translate([point[0] - voxel_size / 2, point[1] - voxel_size / 2, z_min])
        cube.paint_uniform_color(color)
        cubes.append(cube)
mesh = cubes[0]
for c in cubes[1:]:
    mesh += c


# Calculate the vertices of convex hulls on the xy plane
xy_points = points[:, :2] 
hull = ConvexHull(xy_points)
hull_indices = hull.vertices

hull_vertices = points[hull_indices, :]
hull_points = xy_points[hull.vertices]
hull_points_3d = np.hstack((hull_points, np.full((hull_points.shape[0], 1), z_min)))

# Delaunay triangulation of convex hull vertices
delaunay = Delaunay(hull_points)

# Create Open3D triangular meshes
hull_mesh = o3d.geometry.TriangleMesh()

# Add vertex
hull_mesh.vertices = o3d.utility.Vector3dVector(hull_points_3d)

# Add triangular faces
hull_mesh.triangles = o3d.utility.Vector3iVector(delaunay.simplices)

# Calculate the normal vector so that the mesh can render correctly
hull_mesh.compute_vertex_normals()

# Coloring with deep green filling
hull_mesh.paint_uniform_color([0.0, 0.5, 0.2])


o3d.visualization.draw_geometries([line_set, mesh, hull_mesh])
