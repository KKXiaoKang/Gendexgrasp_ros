import os, sys
import numpy as np
import shutil
from tqdm import tqdm
import time
import torch
from PIL import Image
import logging
import os, sys
import os.path as osp
from hydra import initialize, compose
# set level logging
logging.basicConfig(level=logging.INFO)
import logging
import trimesh
import numpy as np
from hydra.utils import instantiate
import argparse
import glob
from omegaconf import DictConfig, OmegaConf
from torchvision.utils import save_image
import torchvision.transforms as T
import cv2
import imageio
import distinctipy
from skimage.feature import canny
from skimage.morphology import binary_dilation
from segment_anything.utils.amg import rle_to_mask

from utils.poses.pose_utils import get_obj_poses_from_template_level, load_index_level_in_level2
from utils.bbox_utils import CropResizePad
from model.utils import Detections, convert_npz_to_json
from model.loss import Similarity
from utils.inout import load_json, save_json_bop23

inv_rgb_transform = T.Compose(
        [
            T.Normalize(
                mean=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
                std=[1 / 0.229, 1 / 0.224, 1 / 0.225],
            ),
        ]
    )

"""
    visualize 函数用于将检测结果与原图像进行可视化，步骤包括：
    @param rgb: 输入图像 | rgb = Image.open(rgb_path).convert("RGB") 通过PIL将RGB图像读取并且转换为RGB格式
    @param detections: 预测结果 | detections = load_json(det_path) 读取检测结果的json文件
    @param save_path: 保存路径 | f"{output_dir}/sam6d_results/vis_ism.png
     - 将图像转换为灰度图像并再转回 RGB。
     - 使用 distinctipy 为每个检测结果分配颜色。
     - 将分割掩膜应用到原图像上，并高亮边缘。
     - 将处理后的图像和预测结果拼接在一起，并保存
"""
def visualize(rgb, detections, save_path="tmp.png"):
    img = rgb.copy()
    gray = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2GRAY)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    colors = distinctipy.get_colors(len(detections))
    alpha = 0.33

    best_score = 0.
    for mask_idx, det in enumerate(detections):
        if best_score < det['score']:
            best_score = det['score']
            best_det = detections[mask_idx]

    mask = rle_to_mask(best_det["segmentation"])
    edge = canny(mask)
    edge = binary_dilation(edge, np.ones((2, 2)))
    obj_id = best_det["category_id"]
    temp_id = obj_id - 1

    r = int(255*colors[temp_id][0])
    g = int(255*colors[temp_id][1])
    b = int(255*colors[temp_id][2])
    img[mask, 0] = alpha*r + (1 - alpha)*img[mask, 0]
    img[mask, 1] = alpha*g + (1 - alpha)*img[mask, 1]
    img[mask, 2] = alpha*b + (1 - alpha)*img[mask, 2]   
    img[edge, :] = 255
    
    img = Image.fromarray(np.uint8(img))
    img.save(save_path)
    prediction = Image.open(save_path)
    
    # concat side by side in PIL
    img = np.array(img)
    concat = Image.new('RGB', (img.shape[1] + prediction.size[0], img.shape[0]))
    concat.paste(rgb, (0, 0))
    concat.paste(prediction, (img.shape[1], 0))
    return concat

def visualize_origin(rgb, detections, save_path="output.png"):
    img = np.array(rgb.copy())
    colors = distinctipy.get_colors(detections['masks'].shape[0])
    alpha = 0.5

    # 遍历每个 mask 进行可视化
    for mask_idx in range(detections['masks'].shape[0]):
        mask = detections['masks'][mask_idx].cpu().numpy()
        mask = mask.astype(bool)
        
        # 生成 mask 的边缘
        edge = canny(mask)
        edge = binary_dilation(edge, np.ones((2, 2)))

        # 获取对应的边界框
        box = detections['boxes'][mask_idx].cpu().numpy().astype(int)
        
        # 获取当前 mask 的颜色
        color = colors[mask_idx]
        r, g, b = [int(255 * c) for c in color]
        
        # 在图像中叠加 mask
        img[mask, 0] = alpha * r + (1 - alpha) * img[mask, 0]
        img[mask, 1] = alpha * g + (1 - alpha) * img[mask, 1]
        img[mask, 2] = alpha * b + (1 - alpha) * img[mask, 2]
        
        # 将边缘设置为白色
        img[edge, :] = 255
        
        # 在图像上绘制边界框
        cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (r, g, b), 2)
    
    # 保存结果并返回拼接的图像
    img = Image.fromarray(np.uint8(img))
    img.save(save_path)
    prediction = Image.open(save_path)
    
    # 将原始图像和预测结果拼接
    concat = Image.new('RGB', (rgb.width + prediction.width, rgb.height))
    concat.paste(rgb, (0, 0))
    concat.paste(prediction, (rgb.width, 0))
    return concat

"""
    batch_input_data 函数用于处理深度图像和相机参数，输出一个包含这些数据的字典：
    @param depth_path: 输入深度图像路径
    @param cam_path: 输入相机参数路径
    @param device: 设备类型(torch.cuda设备)

    - 读取深度图像，并将其转换为张量。
    - 读取相机内参矩阵和深度尺度，并将其转换为张量
"""
def batch_input_data(depth_path, cam_path, device):
    batch = {}
    cam_info = load_json(cam_path)
    depth = np.array(imageio.imread(depth_path)).astype(np.int32)
    cam_K = np.array(cam_info['cam_K']).reshape((3, 3))
    depth_scale = np.array(cam_info['depth_scale'])

    batch["depth"] = torch.from_numpy(depth).unsqueeze(0).to(device)
    batch["cam_intrinsic"] = torch.from_numpy(cam_K).unsqueeze(0).to(device)
    batch['depth_scale'] = torch.from_numpy(depth_scale).unsqueeze(0).to(device)
    return batch

"""
    run_inference | 主推理函数
     - (1) 使用 hydra 从配置文件中加载和初始化模型。根据 segmentor_model 参数选择不同的配置 | 使用sam
"""
def run_inference(segmentor_model, output_dir, cad_path, rgb_path, depth_path, cam_path, stability_score_thresh):
    # 初始化 Hydra 配置管理工具，并加载 run_inference.yaml 配置文件
    with initialize(version_base=None, config_path="configs"):
        cfg = compose(config_name='run_inference.yaml')

    # 根据指定的分割模型选择对应的配置文件
    if segmentor_model == "sam":
        # 初始化 Hydra 并加载 ISM_sam.yaml 配置文件
        with initialize(version_base=None, config_path="configs/model"):
            cfg.model = compose(config_name='ISM_sam.yaml')
        # 设置模型的稳定性评分阈值
        cfg.model.segmentor_model.stability_score_thresh = stability_score_thresh
    elif segmentor_model == "fastsam":
        # 初始化 Hydra 并加载 ISM_fastsam.yaml 配置文件
        with initialize(version_base=None, config_path="configs/model"):
            cfg.model = compose(config_name='ISM_fastsam.yaml')
    else:
        # 如果分割模型不支持，抛出异常
        raise ValueError("The segmentor_model {} is not supported now!".format(segmentor_model))

    # 日志记录 模型 初始化的开始
    logging.info("Initializing model")
    # 实例化模型
    model = instantiate(cfg.model)
    
    # 选择设备（GPU 或 CPU）
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # 将描述符模型移动到指定设备
    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device
    # if there is predictor in the model, move it to device
    # 如果模型中存在预测器，将其也移动到设备
    if hasattr(model.segmentor_model, "predictor"):
        model.segmentor_model.predictor.model = (
            model.segmentor_model.predictor.model.to(device)
        )
    else:
        # 如果没有预测器，设置模型
        model.segmentor_model.model.setup_model(device=device, verbose=True)
    # 日志记录模型移动到设备的完成
    logging.info(f"Moving models to {device} done!")
        
    # 日志记录 模板 初始化的开始
    logging.info("Initializing template")
    # 获取模板目录，并统计模板数量
    template_dir = os.path.join(output_dir, 'templates')
    num_templates = len(glob.glob(f"{template_dir}/*.npy")) # 检索相机视角的模板数量
    boxes, masks, templates = [], [], []

    # 处理每个模板
    for idx in range(num_templates):
        image = Image.open(os.path.join(template_dir, 'rgb_'+str(idx)+'.png')) # 读取rgb图片
        mask = Image.open(os.path.join(template_dir, 'mask_'+str(idx)+'.png')) # 读取mask灰度模板图片
        # 获取掩膜的边界框
        boxes.append(mask.getbbox())

        # 将图像和掩膜转换为张量，并进行归一化处理
        image = torch.from_numpy(np.array(image.convert("RGB")) / 255).float()
        mask = torch.from_numpy(np.array(mask.convert("L")) / 255).float()
        # 应用掩膜到图像
        image = image * mask[:, :, None]
        # 将处理后的图像和掩膜添加到列表中
        templates.append(image)
        masks.append(mask.unsqueeze(-1))
    
    # 将模板和掩膜列表转换为张量
    templates = torch.stack(templates).permute(0, 3, 1, 2)
    masks = torch.stack(masks).permute(0, 3, 1, 2)
    boxes = torch.tensor(np.array(boxes))
    
    # 创建图像处理配置
    processing_config = OmegaConf.create(
        {
            "image_size": 224,
        }
    )
    # 初始化图像处理工具
    proposal_processor = CropResizePad(processing_config.image_size)
    # 处理模板图像和掩膜
    templates = proposal_processor(images=templates, boxes=boxes).to(device)
    masks_cropped = proposal_processor(images=masks, boxes=boxes).to(device)

    # 初始化模型的参考数据
    model.ref_data = {}
    # 计算模板的描述符
    model.ref_data["descriptors"] = model.descriptor_model.compute_features(
                    templates, token_name="x_norm_clstoken"
                ).unsqueeze(0).data
    # 计算模板的外观描述符
    model.ref_data["appe_descriptors"] = model.descriptor_model.compute_masked_patch_feature(
                    templates, masks_cropped[:, 0, :, :]
                ).unsqueeze(0).data
    
    # 运行推理
    # 打开 RGB 图像
    rgb = Image.open(rgb_path).convert("RGB")
    # 使用模型生成掩膜
    detections = model.segmentor_model.generate_masks(np.array(rgb))
    # # print(" detections : ", detections)
    # 可视化检测结果
    vis_img = visualize_origin(rgb, detections, f"{output_dir}/sam6d_results/origin_vis_ism.png")
    # 保存可视化结果
    vis_img.save(f"{output_dir}/sam6d_results/origin_vis_ism.png")

    # 后续检查
    detections = Detections(detections)
    
    # 计算查询图像的描述符
    query_decriptors, query_appe_descriptors = model.descriptor_model.forward(np.array(rgb), detections)

    # matching descriptors
    # 匹配描述符
    (
        idx_selected_proposals,
        pred_idx_objects,
        semantic_score,
        best_template,
    ) = model.compute_semantic_score(query_decriptors)
    # print(" idx_selected_proposals : ", idx_selected_proposals)
    # print(" pred_idx_objects : ", pred_idx_objects)
    # print(" semantic_score : ", semantic_score)
    # print(" best_template : ", best_template)
    
    # update detections
    # 更新检测结果
    detections.filter(idx_selected_proposals)
    query_appe_descriptors = query_appe_descriptors[idx_selected_proposals, :]

    # compute the appearance score
    # 计算外观分数
    appe_scores, ref_aux_descriptor= model.compute_appearance_score(best_template, pred_idx_objects, query_appe_descriptors)

    # compute the geometric score
    # 计算几何分数
    batch = batch_input_data(depth_path, cam_path, device)
    template_poses = get_obj_poses_from_template_level(level=2, pose_distribution="all")
    template_poses[:, :3, 3] *= 0.4
    poses = torch.tensor(template_poses).to(torch.float32).to(device)
    model.ref_data["poses"] =  poses[load_index_level_in_level2(0, "all"), :, :]

    # 加载 CAD 模型并采样点云
    mesh = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(2048).astype(np.float32) / 1000.0
    model.ref_data["pointcloud"] = torch.tensor(model_points).unsqueeze(0).data.to(device)
    
    # 投影模板到图像上
    image_uv = model.project_template_to_image(best_template, pred_idx_objects, batch, detections.masks)

    # 计算几何分数和可见性比率
    geometric_score, visible_ratio = model.compute_geometric_score(
        image_uv, detections, query_appe_descriptors, ref_aux_descriptor, visible_thred=model.visible_thred
        )

    # final score
    # 计算最终分数
    final_score = (semantic_score + appe_scores + geometric_score*visible_ratio) / (1 + 1 + visible_ratio)

    # 将最终分数添加到检测结果中
    detections.add_attribute("scores", final_score)
    detections.add_attribute("object_ids", torch.zeros_like(final_score))   
    
    # 将检测结果转换为 numpy 格式
    detections.to_numpy()
    # 定义结果保存路径
    save_path = f"{output_dir}/sam6d_results/detection_ism"
    print(f" 定义分割结果保存路径 : {save_path}")
    # 保存检测结果到文件
    detections.save_to_file(0, 0, 0, save_path, "Custom", return_results=False)
    # 将 npz 文件转换为 json 格式
    detections = convert_npz_to_json(idx=0, list_npz_paths=[save_path+".npz"])
    save_json_bop23(save_path+".json", detections)
    # 可视化检测结果
    vis_img = visualize(rgb, detections, f"{output_dir}/sam6d_results/vis_ism.png")
    # 保存可视化结果
    vis_img.save(f"{output_dir}/sam6d_results/vis_ism.png")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--segmentor_model", default='sam', help="The segmentor model in ISM")
    parser.add_argument("--output_dir", nargs="?", help="Path to root directory of the output")
    parser.add_argument("--cad_path", nargs="?", help="Path to CAD(mm)")
    parser.add_argument("--rgb_path", nargs="?", help="Path to RGB image")
    parser.add_argument("--depth_path", nargs="?", help="Path to Depth image(mm)")
    parser.add_argument("--cam_path", nargs="?", help="Path to camera information")
    parser.add_argument("--stability_score_thresh", default=0.97, type=float, help="stability_score_thresh of SAM")
    args = parser.parse_args()
    os.makedirs(f"{args.output_dir}/sam6d_results", exist_ok=True)
    run_inference(
        args.segmentor_model, args.output_dir, args.cad_path, args.rgb_path, args.depth_path, args.cam_path, 
        stability_score_thresh=args.stability_score_thresh,
    )