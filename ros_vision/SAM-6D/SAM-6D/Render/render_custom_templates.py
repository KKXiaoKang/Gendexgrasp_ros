import blenderproc as bproc

import os
import argparse
import cv2
import numpy as np
import trimesh

"""
    --cad_path :  指定 CAD 模型路径
    --output_dir : 输出目录
    --normalize : 是否归一化模型
    --colorize : 是否为模型上色
    --base_color : 模型基础颜色

    这个脚本的主要功能是从指定的 .ply 文件加载三维模型，使用预定义的相机姿态进行渲染，并将渲染结果（包括 RGB 图像、掩码和 NOCS 数据）保存到指定的输出目录中
     - RGB 图像  | rgb_'+str(idx)+'.png
     - 掩码图像   | mask_'+str(idx)+'.png
     - NOCS 数据  | 'xyz_'+str(idx)+'.npy'
"""
parser = argparse.ArgumentParser()
parser.add_argument('--cad_path', help="The path of CAD model")
parser.add_argument('--output_dir', help="The path to save CAD templates")
parser.add_argument('--normalize', default=True, help="Whether to normalize CAD model or not")
parser.add_argument('--colorize', default=False, help="Whether to colorize CAD model or not")
parser.add_argument('--base_color', default=0.05, help="The base color used in CAD model")
args = parser.parse_args()

# set the cnos camera path
# 加载相机姿态
render_dir = os.path.dirname(os.path.abspath(__file__))
cnos_cam_fpath = os.path.join(render_dir, '../Instance_Segmentation_Model/utils/poses/predefined_poses/cam_poses_level0.npy')

# 初始化 BlenderProc
bproc.init()

# 获取模型的标准化信息
def get_norm_info(mesh_path):
    """
        加载 .ply 文件并将其转换为网格
        从网格中采样 1024 个点
        计算点云的最小值和最大值
        计算半径，然后计算标准化缩放因子
    """
    mesh = trimesh.load(mesh_path, force='mesh')

    model_points = trimesh.sample.sample_surface(mesh, 1024)[0]
    model_points = model_points.astype(np.float32)

    min_value = np.min(model_points, axis=0)
    max_value = np.max(model_points, axis=0)

    radius = max(np.linalg.norm(max_value), np.linalg.norm(min_value))

    return 1/(2*radius)


# load cnos camera pose
# 加载相机姿态
cam_poses = np.load(cnos_cam_fpath)

# calculating the scale of CAD model
# 设置缩放因子
if args.normalize:
    scale = get_norm_info(args.cad_path)
else:
    scale = 1

# 渲染循环
for idx, cam_pose in enumerate(cam_poses):
    print(f"开始渲染循环 : 第 {idx} 次")
    bproc.clean_up()

    # load object
    obj = bproc.loader.load_obj(args.cad_path)[0]
    obj.set_scale([scale, scale, scale])
    obj.set_cp("category_id", 1)

    # assigning material colors to untextured objects
    if args.colorize:
        color = [args.base_color, args.base_color, args.base_color, 0.]
        material = bproc.material.create('obj')
        material.set_principled_shader_value('Base Color', color)
        obj.set_material(0, material)

    # convert cnos camera poses to blender camera poses
    cam_pose[:3, 1:3] = -cam_pose[:3, 1:3]
    cam_pose[:3, -1] = cam_pose[:3, -1] * 0.001 * 2
    bproc.camera.add_camera_pose(cam_pose)
    
    # set light
    light_scale = 2.5
    light_energy = 1000
    light1 = bproc.types.Light()
    light1.set_type("POINT")
    light1.set_location([light_scale*cam_pose[:3, -1][0], light_scale*cam_pose[:3, -1][1], light_scale*cam_pose[:3, -1][2]])
    light1.set_energy(light_energy)

    bproc.renderer.set_max_amount_of_samples(50)
    # render the whole pipeline
    data = bproc.renderer.render()
    # render nocs
    data.update(bproc.renderer.render_nocs())
    
    # check save folder
    save_fpath = os.path.join(args.output_dir, "templates")
    print(f"渲染保存路径 Saving images to: {save_fpath}")
    if not os.path.exists(save_fpath):
        os.makedirs(save_fpath)

    # save rgb image
    color_bgr_0 = data["colors"][0]
    color_bgr_0[..., :3] = color_bgr_0[..., :3][..., ::-1]
    cv2.imwrite(os.path.join(save_fpath,'rgb_'+str(idx)+'.png'), color_bgr_0)

    # save mask
    mask_0 = data["nocs"][0][..., -1]
    cv2.imwrite(os.path.join(save_fpath,'mask_'+str(idx)+'.png'), mask_0*255)
    
    # save nocs
    xyz_0 = 2*(data["nocs"][0][..., :3] - 0.5)
    np.save(os.path.join(save_fpath,'xyz_'+str(idx)+'.npy'), xyz_0.astype(np.float16))