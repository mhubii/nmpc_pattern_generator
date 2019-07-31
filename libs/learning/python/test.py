import numpy as np
import cv2
import torch
from torch.autograd import Variable
import utils
from model import RGBDCNN

if __name__ == '__main__':

    # Load 5 consecutive images.
    

    torch.set_printoptions(threshold=500000)

    data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out"
    file_rgb = "data/img/left_epoch_1_00000586.jpg"
    file_d = "data/img/wls_disp_epoch_1_00000586.jpg"

    img_rgb, img_d = utils.load_rgbd(data_dir, file_rgb, file_d)

    # Crop.
    img_rgb = utils.crop(img_rgb)
    img_d = utils.crop(img_d)

    # Filter.
    #img_rgb = utils.filter(img_rgb)

    # Normalize.
    img_rgb = utils.normalize(img_rgb)
    img_d = utils.normalize(img_d)

    # Change HxWxC to CxHxW.
    img_rgb = np.transpose(img_rgb, (2, 0, 1))
    img_d = np.expand_dims(img_d, 2)
    img_d = np.transpose(img_d, (2, 0, 1))

    # Concatenate rgb, d to rgbd image.    
    img_rgbd = np.concatenate([img_rgb, img_d], axis=0)

    # Check network output.
    trained_model = RGBDCNN(utils.RGBD_INPUT_SHAPE, 3, 1)

    trained_model.load_state_dict(torch.load('trained_rgbd_full_sized_imgs_unet.pt'))

    img_rgbd = torch.from_numpy(img_rgbd).float()
    img_rgbd = torch.unsqueeze(img_rgbd, 0)

    output = trained_model(img_rgbd)

    print(output)
