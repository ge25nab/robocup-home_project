from __future__ import division

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import numpy as np

# from utils.parse_config import *
# from utils.utils import build_targets
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
from torchvision import transforms,models



# class VGG(nn.Module):

#     def __init__(self, features, num_classes=3):
#         super(VGG, self).__init__()
#         self.features = features
#         self.classifier = nn.Sequential(
#             nn.Linear(24576, 1024),
#             nn.ReLU(True),
#             nn.Dropout(),
#             nn.Linear(1024, 256),
#             nn.ReLU(True),
#             nn.Dropout(),
#             nn.Linear(256, num_classes),
#         )
#         self._initialize_weights()

#     def forward(self, x):
#         # print(x.shape)
#         x = self.features(x)
#         x = x.view(x.size(0), -1)
#         x = self.classifier(x)
#         return x

#     def _initialize_weights(self):
#         for m in self.modules():
#             if isinstance(m, nn.Conv2d):
#                 n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
#                 m.weight.data.normal_(0, math.sqrt(2. / n))
#                 if m.bias is not None:
#                     m.bias.data.zero_()
#             elif isinstance(m, nn.BatchNorm2d):
#                 m.weight.data.fill_(1)
#                 m.bias.data.zero_()
#             elif isinstance(m, nn.Linear):
#                 m.weight.data.normal_(0, 0.01)
#                 m.bias.data.zero_()

# cfg = {
#     'A': [64, 'M', 128, 'M']
# }


# def make_layers(cfg, batch_norm=False):
#     layers = []
#     in_channels = 3
#     for v in cfg:
#         if v == 'M':
#             layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
#         else:
#             conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
#             if batch_norm:
#                 layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
#             else:
#                 layers += [conv2d, nn.ReLU(inplace=True)]
#             in_channels = v
#     return nn.Sequential(*layers)

# def vgg11(pretrained=False, **kwargs):
#     model = VGG(make_layers(cfg['A']), **kwargs)
#     if pretrained:
#         model.load_state_dict(model_zoo.load_url(model_urls['vgg11']))
#     return model

def vgg16():
    vgg16 = models.vgg16_bn(pretrained=True)
    for param in vgg16.features.parameters():
        param.requires_grad = False
    num_features = vgg16.classifier[6].in_features
    features = list(vgg16.classifier.children())[:-1]
    features.extend([nn.Linear(num_features,3),nn.Softmax(dim=1)])
    vgg16.classifier = nn.Sequential(*features)
    # print(vgg16)
    return vgg16

def resnet_new():

    model = torch.hub.load('pytorch/vision:v0.4.2', 'resnet34', pretrained=True)
    num = model.fc.in_features
    fc = list(model.fc.children())[:-1]
    fc.extend([nn.Linear(num,256),nn.Linear(256,3),nn.Softmax(dim=1)])
    model.fc = nn.Sequential(*fc)

    return model
