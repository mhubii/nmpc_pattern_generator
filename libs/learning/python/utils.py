import pandas as pd
import numpy as np
import skimage.io
import cv2
import torch
from torch.utils.data import Dataset
import os

# INPUT_SHAPE as input for CNN (cropped shapes).
IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 60, 80, 4 # 4 = RGBD
CROPPED_IMAGE_HEIGHT, CROPPED_IMAGE_WIDTH = 38, 73
INPUT_SHAPE = (IMAGE_CHANNELS, CROPPED_IMAGE_HEIGHT, CROPPED_IMAGE_WIDTH)


class DataSetGenerator(Dataset):
    def __init__(self, data_dir, transform=None):
        """
            Load all image paths and velocities
            from the .txt file.
        """
        self.data_dir = data_dir
        self.image_paths, self.velocities = load_data(data_dir)
        self.transform = transform

    def __getitem__(self, index):
        """
            Get an image and the velocity by index.
        """
        img_left, img_wls_disp = load_rgbd(self.data_dir, self.image_paths[index][0], self.image_paths[index][1])

        vel = self.velocities[index]

        sample = {'img_left': img_left, 'img_wls_disp': img_wls_disp, 'vel': vel}

        if self.transform is not None:
            sample = self.transform(sample)

        return sample

    def __len__(self):
        """
            Return the length of the whole data set.
        """
        return self.velocities.shape[0]


class PreProcessData(object):
    """
        Pre-process the data.
    """
    def __call__(self, sample):
        img_left, img_wls_disp, vel = sample['img_left'], sample['img_wls_disp'], sample['vel']

        img_left = crop(img_left)
        img_wls_disp = crop(img_wls_disp)

        img_left = normalize(img_left)
        img_wls_disp = normalize(img_wls_disp)

        # Change HxWxC to CxHxW.
        img_left = np.transpose(img_left, (2, 0, 1))
        #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

        # Concatenate rgb, d to rgbd image.    
        img_wls_disp = np.expand_dims(img_wls_disp, 0)
        img_rgbd = np.concatenate([img_left, img_wls_disp], axis=0)
        

        return {'img_rgbd': img_rgbd, 'vel': vel}


class ToTensor(object):
    """
        Convert data to tensor.
    """
    def __call__(self, sample):
        img_rgbd, vel = sample['img_rgbd'], sample['vel']

        return {'img_rgbd': torch.from_numpy(img_rgbd).float(),
                'vel': torch.from_numpy(vel).float()}


def load_data(data_dir):
    """
        Loads the input data and separates it into image_paths
        and velocities.
    :return:
        image_paths: np.ndarray
                     Location of recorded images.
        labels: float
                Velocities.
    """
    data_df = pd.read_csv(os.path.join(
                          data_dir, 'log.txt'),
                          delimiter=', ',
                          names=['left', 'right', 'l_disp', 'wls_disp', 'vel0', 'vel1', 'vel2'],
                          engine='python')

    # Randomly shuffle data to remove correlations.
    data_df = data_df.iloc[np.random.permutation(len(data_df))]

    image_paths = data_df[['left', 'wls_disp']].values
    velocities = data_df[['vel0', 'vel1']].values#, 'vel2']].values

    return image_paths, velocities


def load_rgbd(data_dir, image_file_rgb, image_file_d):
    """
        Load RGB image from a file.
    """

    image_rgb = cv2.imread(os.path.join(data_dir, image_file_rgb), cv2.IMREAD_COLOR)
    image_d = cv2.imread(os.path.join(data_dir, image_file_d), cv2.IMREAD_GRAYSCALE)

    return image_rgb, image_d


def normalize(image):
    """
        Normalize image to [-1, 1].
    """
    image = image/127.5 - 1.

    return image

def crop(image):
    """
        Crop of the sky since it does not add
        useful information for the training. Also
        remove the border because of the depth images. 
    """
    image = image[20:-2, 5:-2]
    
    return image
    

import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out"
    file_rgb = "data/left_epoch_1_00000145.png"
    file_d = "data/wls_disp_epoch_1_00000145.png"

    rgb, d = load_rgbd(data_dir, file_rgb, file_d)

    rgb = normalize(rgb)
    d = normalize(d)

    d = crop(d)

    plt.imshow(d)
    plt.show()

    rgb = crop(rgb)

    plt.imshow(rgb)
    plt.show()

    rgb = np.transpose(rgb, (2, 0, 1))

    d = np.expand_dims(d, 0)

    print(rgb.shape)
    print(d.shape)

    rgbd = np.concatenate([rgb, d], axis=0)
    print(rgbd.shape)
