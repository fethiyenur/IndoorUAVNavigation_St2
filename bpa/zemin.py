import open3d as o3d
import numpy as np

mesh = o3d.io.read_triangle_mesh("output.obj")
vertices = np.asarray(mesh.vertices)
print(f"Min Z: {vertices[:,2].min():.3f}")
print(f"Max Z: {vertices[:,2].max():.3f}")
print(f"Orta Z: {vertices[:,2].mean():.3f}")