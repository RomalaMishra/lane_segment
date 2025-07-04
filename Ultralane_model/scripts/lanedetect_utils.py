#!/usr/bin/env python3

import os
import sys
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms

sys.path.append("/home/sree/ros_ws/src/igvc_ros/igvc_perception/dl_model")
from model.model import parsingNet
from data.constant import tusimple_row_anchor

sys.path.append("/home/sree/ros_ws/src/igvc_ros/igvc_perception/dl_model/utils")
from common import merge_config
from dist_utils import dist_print

import scipy.special
import numpy as np
from PIL import Image
import cv2

def initialize_lane_detection():

    # Merge configuration options
    args, cfg = merge_config()
    dist_print('start testing...')

    # Check backbone configuration
    assert cfg.backbone in ['18', '34', '50', '101', '152', '50next', '101next', '50wide', '101wide']

    # Set the number of classes per lane
    cls_num_per_lane = 56

    # Create the lane detection model
    net = parsingNet(pretrained=False, backbone=cfg.backbone, cls_dim=(cfg.griding_num + 1, cls_num_per_lane, 4),
                use_aux=False).cuda()  # we dont need auxiliary segmentation in testing

    state_dict = torch.load(cfg.test_model, map_location='cpu')['model']

    compatible_state_dict = {}

    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v

    net.load_state_dict(compatible_state_dict, strict=False)
    net.eval()

    img_w, img_h = 1280, 720
    row_anchor = tusimple_row_anchor

    return net, img_w, img_h, row_anchor , cfg

def preprocess_image(img, img_transforms):

    img = Image.fromarray(img)
    x = img_transforms(img)
    x = x.unsqueeze(0).cuda() + 1

    return x

def postprocess_image(out, imgs_copy, cfg, img_w, img_h, cls_num_per_lane, row_anchor):

    col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
    col_sample_w = col_sample[1] - col_sample[0]

    out_j = out[0].data.cpu().numpy()
    out_j = out_j[:, ::-1, :]
    prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
    idx = np.arange(cfg.griding_num) + 1
    idx = idx.reshape(-1, 1, 1)
    loc = np.sum(prob * idx, axis=0)
    out_j = np.argmax(out_j, axis=0)
    loc[out_j == cfg.griding_num] = 0
    out_j = loc

    imgs_black = np.zeros((img_h, img_w, 3), dtype=np.uint8)

    for i in range(out_j.shape[1]):

        if np.sum(out_j[:, i] != 0) > 2:

            for k in range(out_j.shape[0]):

                if out_j[k, i] > 0:
                    
                    ppp = (int(out_j[k, i] * col_sample_w * img_w / 800) - 1,
                            int(img_h * (row_anchor[cls_num_per_lane - 1 - k] / 288)) - 1)
                    
                    cv2.circle(imgs_black, ppp, 5, (0, 255, 0), -1)
                    cv2.circle(imgs_copy,ppp,5,(0,255,0),-1)

    return imgs_black, imgs_copy


