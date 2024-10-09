import open3d as o3d
import numpy as np
import trimesh

# 加载点云（.ply）文件
def load_point_cloud(ply_file_path):
    print(f"Loading point cloud from {ply_file_path}")
    point_cloud = o3d.io.read_point_cloud(ply_file_path)
    return point_cloud

# 加载STL文件并提取顶点数据
def load_stl_as_point_cloud(stl_file_path):
    print(f"Loading STL mesh from {stl_file_path}")
    mesh = trimesh.load_mesh(stl_file_path)
    stl_points = np.array(mesh.vertices)  # 提取顶点作为点云
    stl_point_cloud = o3d.geometry.PointCloud()
    stl_point_cloud.points = o3d.utility.Vector3dVector(stl_points)
    return stl_point_cloud

# 手动初始变换，用于初步对齐两个点云
def manual_initial_alignment(source_point_cloud, target_point_cloud):
    print("Applying manual initial alignment...")

    # 初始化变换矩阵，进行旋转和平移调整
    trans_init = np.eye(4)
    
    # 对旋转进行调整，例如沿X轴旋转45度
    # trans_init[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 4, 0, 0))      # X轴旋转45度
    # trans_init[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 4, 0, np.pi))  # X轴旋转45度，Z轴旋转180度
    # trans_init[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 3, 0, np.pi))  # X轴旋转60度，Z轴旋转180度
    trans_init[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.pi))

    # 对平移进行调整，沿Z轴移动0.1单位
    trans_init[:3, 3] = [0.0, 0.0, 0.1]  # 调整平移

    # 应用初始变换到源点云
    source_point_cloud.transform(trans_init)
    print(" trans_init : ")
    print(trans_init)
    return source_point_cloud, trans_init

# 使用ICP算法对齐点云和STL文件
def align_point_clouds(initial_transformation_matrix, source_point_cloud, target_point_cloud, voxel_size=0.005, max_correspondence_distance=0.1):
    # 对点云进行下采样，增加ICP的效率
    print("Downsampling point cloud...")
    source_downsampled = source_point_cloud.voxel_down_sample(voxel_size)

    # 初始化变换矩阵为单位矩阵
    trans_init = initial_transformation_matrix

    print("Running ICP to align the point clouds...")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_downsampled,                 # 源点云
        target_point_cloud,                 # 目标点云 (STL顶点)
        max_correspondence_distance,        # ICP算法的最大对应点距离
        trans_init,                         # 初始变换矩阵
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    # debug
    print("Fitness:", reg_p2p.fitness)
    print("Inlier RMSE:", reg_p2p.inlier_rmse)

    # 返回ICP计算出的变换矩阵
    return reg_p2p.transformation

# 智能缩放点云：根据两者的边界框大小自动调整比例
def smart_scale_point_clouds(source_point_cloud, target_point_cloud):
    print("Calculating bounding boxes for automatic scaling...")

    # 获取源点云和目标点云的边界框
    source_bbox = source_point_cloud.get_axis_aligned_bounding_box()
    target_bbox = target_point_cloud.get_axis_aligned_bounding_box()

    # 计算边界框的范围（bounding box size）
    source_extent = source_bbox.get_extent()  # 源点云的尺寸
    target_extent = target_bbox.get_extent()  # 目标点云的尺寸

    # 计算缩放因子：比较两个点云的边界框，取最大尺度比
    scale_factor = target_extent.max() / source_extent.max()

    print(f"Calculated scale factor: {scale_factor}")

    # 对源点云进行缩放
    points = np.asarray(source_point_cloud.points)
    points *= scale_factor
    source_point_cloud.points = o3d.utility.Vector3dVector(points)

    return source_point_cloud

# 主函数：计算齐次变换矩阵并可视化
def main(ply_file_path, stl_file_path):
    # 加载点云和STL文件
    point_cloud = load_point_cloud(ply_file_path)
    stl_point_cloud = load_stl_as_point_cloud(stl_file_path)

    # 智能缩放点云
    point_cloud = smart_scale_point_clouds(point_cloud, stl_point_cloud)

    # 手动初始对齐
    # point_cloud, initial_transformation_matrix = manual_initial_alignment(point_cloud, stl_point_cloud)

    # 初始化变换矩阵，进行旋转和平移调整
    initial_transformation_matrix = np.eye(4)
    initial_transformation_matrix[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.pi))
    initial_transformation_matrix[:3, 3] = [0.0, 0.0, 0.1]  # 调整平移

    # 对齐并获取ICP变换矩阵
    icp_transformation_matrix = align_point_clouds(initial_transformation_matrix, point_cloud, stl_point_cloud, max_correspondence_distance=0.1)

    print("ICP Transformation Matrix:")
    print(icp_transformation_matrix)

    # 提取所需要的旋转矩阵
    rotation_matrix = icp_transformation_matrix[:3, :3]
    print("Rotation Matrix from final transformation:")
    print(rotation_matrix)

    # 应用最终变换矩阵到源点云
    point_cloud.transform(icp_transformation_matrix)

    # 可视化对齐结果
    print("Visualizing the aligned point clouds...")
    o3d.visualization.draw_geometries([point_cloud, stl_point_cloud])

if __name__ == "__main__":
    # 提供Ply文件路径和STL文件路径
    ply_file_path = "./object_point_cloud.ply"
    stl_file_path = "./mustard_bottle.stl"

    # 运行主函数
    main(ply_file_path, stl_file_path)
