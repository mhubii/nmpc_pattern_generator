import pandas as pd
import numpy as np
import skimage.io
import cv2
import torch
from torch.utils.data import Dataset
import os

# INPUT_SHAPE as input for CNN (cropped shapes).
IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 240, 320, 4 # 4 = RGBD
CROPPED_IMAGE_HEIGHT, CROPPED_IMAGE_WIDTH = 185, 276
#RESIZED_IMAGE_HEIGHT, RESIZED_IMAGE_WIDTH = 60, 80
RESIZED_IMAGE_HEIGHT, RESIZED_IMAGE_WIDTH = 32, 32
RGBD_INPUT_SHAPE = (IMAGE_CHANNELS, RESIZED_IMAGE_HEIGHT, RESIZED_IMAGE_WIDTH)
GD_INPUT_SHAPE = (2, CROPPED_IMAGE_HEIGHT, CROPPED_IMAGE_WIDTH)

# sizes as chosen for depth map computation
num_disparities = 32
block_size = 13
l_offs = int(num_disparities) + int(block_size/2.)
r_offs = t_offs = b_offs = int(block_size/2.)


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


class SequenceDataSetGenerator(Dataset):
    def __init__(self, sequence_length, data_dir, transform=None):
        """
            Load all image paths and velocities
            from the .txt file. Always load sequence_length number of
            images that are correlated in time.
        """
        self.sequence_length = sequence_length
        self.data_dir = data_dir
        self.image_paths, self.velocities = load_unshuffeled_epoch_data(data_dir) # shuffled by the sequence sampler
        self.transform = transform

    def __getitem__(self, index):

        sample = {'img_left': [], 'img_wls_disp': [], 'vel': []}        

        for i in range(-self.sequence_length+1,1): # for the correct ordering in temporal dimension
            img_left, img_wls_disp = load_rgbd(self.data_dir, self.image_paths[index+i][0], self.image_paths[index+i][1])

            vel = self.velocities[index+i]

            sample['img_left'].append(img_left)
            sample['img_wls_disp'].append(img_wls_disp)
            sample['vel'].append(vel)

        if self.transform is not None:
            sample = self.transform(sample)

        return sample

    def __len__(self):
        """
            Return the length of the whole data set.
        """
        return self.velocities.shape[0]


class SequenceSampler(torch.utils.data.Sampler):
    def __init__(self, sequence_length, data_dir, split_indices):
        """
            Creates a list of indices from which we can sample a sequence
            images without hitting lower boundaries.
        """
        epoch_idx = load_epoch_indices(data_dir)[split_indices] # split indices since we split in train and validation set
        indices = []

        for idx, e in enumerate(epoch_idx):
            if e >= sequence_length - 1:
                indices.append(idx)
        
        self.indices = np.array(indices)

    def __iter__(self):
        indices = self.indices[np.random.permutation(len(self.indices))]
        return iter(indices)

    def __len__(self):
        return len(self.indices)


class PreProcessRGBDData(object):
    """
        Pre-process the data.
    """
    def __call__(self, sample):
        img_left, img_wls_disp, vel = sample['img_left'], sample['img_wls_disp'], sample['vel']

        img_left = crop(img_left)
        img_wls_disp = crop(img_wls_disp)

        # Resize.
        img_left = resize(img_left)
        img_wls_disp = resize(img_wls_disp)

        img_left = normalize(img_left)
        img_wls_disp = normalize(img_wls_disp)

        # Change HxWxC to CxHxW.
        img_left = np.transpose(img_left, (2, 0, 1))
        #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

        # Concatenate rgb, d to rgbd image.    
        img_wls_disp = np.expand_dims(img_wls_disp, 0)
        img_rgbd = np.concatenate([img_left, img_wls_disp], axis=0)
        
        return {'img': torch.from_numpy(img_rgbd).float(),
                'vel': torch.from_numpy(vel).float()}


class PreProcessSequenceRGBDData(object):
    """
        Pre-process the data.
    """
    def __call__(self, sample):
        vels = np.array(sample['vel'])

        sequence_length = np.array(sample['img_left']).shape[0]

        for i in range(sequence_length):

            sample['img_left'][i] = crop(sample['img_left'][i])
            sample['img_wls_disp'][i] = crop(sample['img_wls_disp'][i])

            # Resize.
            sample['img_left'][i] = resize(sample['img_left'][i])
            sample['img_wls_disp'][i]	 = resize(sample['img_wls_disp'][i])

            sample['img_left'][i] = normalize(sample['img_left'][i])
            sample['img_wls_disp'][i] = normalize(sample['img_wls_disp'][i])

            # Change HxWxC to CxHxW.
            sample['img_left'][i] = np.transpose(sample['img_left'][i], (2, 0, 1))
            #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

            # Concatenate rgb, d to rgbd image.    
            sample['img_wls_disp'][i]  = np.expand_dims(sample['img_wls_disp'][i], 0)
            sample['img_left'][i] = np.concatenate([sample['img_left'][i], sample['img_wls_disp'][i]], axis=0)

        imgs_rgbd = np.array(sample['img_left'])
        vels = np.array(sample['vel'])
        
        # Returns TxCxHxW sequence images.
        return {'imgs': torch.from_numpy(imgs_rgbd).float(),
                'vels': torch.from_numpy(vels).float()}


class PreProcessGDData(object):
    """
        Pre-process gray depth data.
    """
    def __call__(self, sample):
        img_left, img_wls_disp, vel = sample['img_left'], sample['img_wls_disp'], sample['vel']

        # Crop.
        img_left = crop(img_left)
        img_wls_disp = crop(img_wls_disp)

        # Filter the red channel.
        img_left = filter(img_left)

        # Normalize.
        img_left = normalize(img_left)
        img_wls_disp = normalize(img_wls_disp)

        # Change HxWxC to CxHxW.
        #img_left = np.transpose(img_left, (2, 0, 1))
        #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

        # Concatenate gray, d to gd image.    
        img_left = np.expand_dims(img_left, 0)
        img_wls_disp = np.expand_dims(img_wls_disp, 0)
        img_gd = np.concatenate([img_left, img_wls_disp], axis=0)
        
        return {'img': torch.from_numpy(img_gd).float(),
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
                          #names=['left', 'right', 'l_disp', 'wls_disp', 'vel0', 'vel1', 'vel2'], 
                          names=['left', 'wls_disp', 'vel0', 'vel1', 'vel2'], # changed for behavioural cloning external data
                          engine='python')

    # Randomly shuffle data to remove correlations.
    data_df = data_df.iloc[np.random.permutation(len(data_df))]

    image_paths = data_df[['left', 'wls_disp']].values
    #velocities = data_df[['vel0', 'vel1', 'vel2']].values
    velocities = data_df[['vel0', 'vel2']].values

    return image_paths, velocities


def load_unshuffeled_data(data_dir):
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
                          #names=['left', 'right', 'l_disp', 'wls_disp', 'vel0', 'vel1', 'vel2'], 
                          names=['left', 'wls_disp', 'vel0', 'vel1', 'vel2'], # changed for behavioural cloning external data
                          engine='python')

    image_paths = data_df[['left', 'wls_disp']].values
    #velocities = data_df[['vel0', 'vel1', 'vel2']].values
    velocities = data_df[['vel0', 'vel2']].values

    return image_paths, velocities


def load_unshuffeled_epoch_data(data_dir):
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
                          data_dir, 'epoch_log.txt'),
                          delimiter=',',
                          #names=['left', 'right', 'l_disp', 'wls_disp', 'vel0', 'vel1', 'vel2'], 
                          names=['epoch', 'epoch_idx', 'left', 'wls_disp', 'vel0', 'vel1', 'vel2'], # changed for behavioural cloning external data
                          engine='python')

    image_paths = data_df[['left', 'wls_disp']].values
    #velocities = data_df[['vel0', 'vel1', 'vel2']].values
    velocities = data_df[['vel0', 'vel2']].values

    return image_paths, velocities


def load_epoch_indices(data_dir):
    """
        Loads the indices of the current epoch.
    :return:
        epoch_idx: np.ndarray
                     
    """
    data_df = pd.read_csv(os.path.join(
                          data_dir, 'epoch_log.txt'),
                          delimiter=',',
                          names=['epoch', 'epoch_idx', 'left', 'wls_disp', 'vel0', 'vel1', 'vel2'], # changed for behavioural cloning external data
                          engine='python')

    epoch_idx = data_df['epoch_idx']

    return epoch_idx


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
    # 191 is a magic number caused by the rectification of the camera image
    image = image[t_offs:191,l_offs:image.shape[1]-r_offs]
    
    return image

def resize(image):
    """
        Resize an image.
    """
    image = cv2.resize(image, (RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT))

    return image

def filter(image):
    """
        Filter a specific color channel (here red).
    """
    # Convert input image to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image, keep only the red pixels
    image = cv2.inRange(image, np.array([0, 100, 100]), np.array([10, 255, 255]))

    return image


import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out"
    file_rgb = "data/img/left_epoch_3_00239503.jpg"
    file_d = "data/img/wls_disp_epoch_3_00239503.jpg"

    rgb, d = load_rgbd(data_dir, file_rgb, file_d)

    rgb = normalize(rgb)
    d = normalize(d)

    d = crop(d)
    rgb = crop(rgb)

    plt.subplot(121)
    plt.imshow(d)
    plt.subplot(122)
    plt.imshow(rgb)
    plt.show()

    rgb = np.transpose(rgb, (2, 0, 1))

    d = np.expand_dims(d, 0)

    print(rgb.shape)
    print(d.shape)

    rgbd = np.concatenate([rgb, d], axis=0)
    print(rgbd.shape)
