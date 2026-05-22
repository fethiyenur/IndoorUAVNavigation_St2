import open3d as o3d
import numpy as np

# 1. Mesh'i yükle
mesh = o3d.io.read_triangle_mesh("output.obj")
print(f"Orijinal ucgen sayisi: {len(mesh.triangles)}")

# 2. Bağlı bileşenleri bul
triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
triangle_clusters = np.asarray(triangle_clusters)
cluster_n_triangles = np.asarray(cluster_n_triangles)

print(f"Toplam bilesen sayisi: {len(cluster_n_triangles)}")
for i, n in enumerate(cluster_n_triangles):
    print(f"  Bilesen {i}: {n} ucgen")

# 3. Sadece en büyük bileşeni tut
en_buyuk = cluster_n_triangles.argmax()
silincek_mask = triangle_clusters != en_buyuk
mesh.remove_triangles_by_mask(silincek_mask)
mesh.remove_unreferenced_vertices()

print(f"Temizlik sonrasi ucgen sayisi: {len(mesh.triangles)}")

# 4. Temiz mesh'i kaydet
o3d.io.write_triangle_mesh("output_temiz.obj", mesh)
print("output_temiz.obj kaydedildi.")

# 5. Görsel
o3d.visualization.draw_geometries(
    [mesh],
    window_name="Temiz Mesh",
    width=1280, height=720
)