import open3d as o3d

# 读取 PLY 文件
ply_file_path = "./object_point_cloud.ply"
point_cloud = o3d.io.read_point_cloud(ply_file_path)

# 检查点云是否包含足够的点来生成网格
if len(point_cloud.points) < 3:
    print("Error: Point cloud does not contain enough points to form a surface.")
else:
    # 估算法线
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # 使用泊松曲面重建法生成三角形网格
    print("Reconstructing mesh using Poisson surface reconstruction...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=9)
    
    # 计算法线
    print("Computing normals for the mesh...")
    mesh.compute_vertex_normals()  # 这是解决问题的关键步骤

    # 保存为 STL 文件
    stl_file_path = "output.stl"
    o3d.io.write_triangle_mesh(stl_file_path, mesh)
    print(f"Saved STL file to {stl_file_path}")
    
    # 可视化结果，确保网格质量
    o3d.visualization.draw_geometries([mesh])


