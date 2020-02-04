#!/usr/bin/env python3

import torch
import torchvision
from torchvision import transforms,models
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import numpy as np
import math

# from utils.parse_config import *
import os
import shutil



class VGG(nn.Module):

    def __init__(self, features, num_classes=3):
        super(VGG, self).__init__()
        self.features = features
        self.classifier = nn.Sequential(
            nn.Linear(24576, 1024),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(1024, 256),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(256, num_classes),
            nn.Softmax()
        )
        self._initialize_weights()

    def forward(self, x):
        # print(x.shape)
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2. / n))
                if m.bias is not None:
                    m.bias.data.zero_()
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()
            elif isinstance(m, nn.Linear):
                m.weight.data.normal_(0, 0.01)
                m.bias.data.zero_()

cfg = {
    'A': [64, 'M', 128, 'M']
}


def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)

def vgg11(pretrained=False, **kwargs):
    model = VGG(make_layers(cfg['A']), **kwargs)
    if pretrained:
        model.load_state_dict(model_zoo.load_url(model_urls['vgg11']))
    return model



def vgg16():
    vgg16 = models.vgg16_bn(pretrained=True)
    for param in vgg16.features.parameters():
        param.requires_grad = False
        # print(param)
    # print(vgg16.classifier)
    num_features = vgg16.classifier[6].in_features
    features = list(vgg16.classifier.children())[:-1]
    features.extend([nn.Linear(num_features,3),nn.Softmax(dim=1)])
    vgg16.classifier = nn.Sequential(*features)
    # print(vgg16)
    return vgg16

def resnet_new():

    model = torch.hub.load('pytorch/vision:v0.4.2', 'resnet34', pretrained=True)
    # print(model)
    # for param in model.features.parameters():
    #     param.requires_grad = False
        # print(param)
    # print(vgg16.classifier)
    # print(model.fc.in_features)
    # print(fc)
    num = model.fc.in_features
    fc = list(model.fc.children())[:-1]
    fc.extend([nn.Linear(num,256),nn.Linear(256,3),nn.Softmax(dim=1)])
    model.fc = nn.Sequential(*fc)

    return model


def loadtraindata():
    path = "/home/athomews1920/dataset_gesture/"
    trainset = torchvision.datasets.ImageFolder(path,transform = torchvision.transforms.Compose([transforms.Resize((128,96)),transforms.ToTensor()]))
    trainloader = torch.utils.data.DataLoader(trainset,batch_size = 32,shuffle = True)
    return trainloader


def traindata():
    device = torch.device("cuda:0" if torch.cuda.is_available() else 'cpu')
    print(device)
    trainloader = loadtraindata()
    # net = vgg16()
   
    net = resnet_new()
    # net.to(device)
    # if torch.cuda.is_available():
    #     net.cuda()
    optimizer = torch.optim.Adam(net.parameters(),lr=0.0001)
    criterion = nn.CrossEntropyLoss()
    for epoch in range(15):
        new_loss = 0.0
        accuracy = 0.0
        print("epoch is ",epoch)
        for i,data in enumerate(trainloader,0):
            inputs,labels = data
            inputs,labels = Variable(inputs).to(device),Variable(labels).to(device)
            optimizer.zero_grad()
            outputs = net(inputs)
            loss = criterion(outputs,labels)
            loss.backward()
            optimizer.step()
            new_loss += loss.data
            if i%32 == 31:
                # print('[%d of %5d] loss is %.3f'%(epoch+1,i+1,new_loss/50)
                idx,outputs_total = torch.max(outputs.detach(),axis=1)
                # print(torch.sum(outputs_total==labels))
                accuracy = np.sum(outputs_total.numpy()==labels.numpy())/32.0
                print('accuracy is ',accuracy)
                print('loss is',loss)
                accuracy = 0.0
        
    print('finish training')
    torch.save(net,'net.pkl')
    torch.save(net.state_dict(),'net_param.pkl')



if __name__=="__main__":
    traindata()    





