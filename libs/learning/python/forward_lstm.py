import numpy as np
import torch
from torch.autograd import Variable
import utils
from unet_model import UNet
from model import RGBDCNNLSTM

if __name__ == '__main__':

    batch_size = 1  

    # Test the output of a trained unet lstm for reference with cpp in forward_lstm.cpp.
    torch.set_printoptions(threshold=500000)
    data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out"

    # Load model.
    trained_model = UNet(utils.RGBD_INPUT_SHAPE, 2, batch_size)
    # trained_model = RGBDCNNLSTM(utils.RGBD_INPUT_SHAPE, 2)
    #trained_model.load_state_dict(torch.load('trained_unet_lstm.pt'))
    trained_model.load_state_dict(torch.load('trained_unet_lstm.pt'))
    trained_model.eval()
    trained_model.cuda()

    # Load locations.
    img_paths, velocities = utils.load_unshuffeled_data(data_dir)

    # Maximal velocities.
    vel_max = np.array([0.15, 0.2])
    vel_max = torch.from_numpy(vel_max).float().cuda()

    sample = {'img_left': [], 'img_wls_disp': [], 'vel': []} 

    N = 5
    sequence_length = 5

    for j in range(-sequence_length+1,1): # for the correct ordering in temporal dimension
        img_left, img_wls_disp = utils.load_rgbd(data_dir, img_paths[N+j][0], img_paths[N+j][1])
        vel = velocities[N+j]

        sample['img_left'].append(img_left)
        sample['img_wls_disp'].append(img_wls_disp)

    for j in range(sequence_length):

        sample['img_left'][j] = utils.crop(sample['img_left'][j])
        sample['img_wls_disp'][j] = utils.crop(sample['img_wls_disp'][j])

        # Resize.
        sample['img_left'][j] = utils.resize(sample['img_left'][j])
        sample['img_wls_disp'][j]	 = utils.resize(sample['img_wls_disp'][j])

        sample['img_left'][j] = utils.normalize(sample['img_left'][j])
        sample['img_wls_disp'][j] = utils.normalize(sample['img_wls_disp'][j])

        # Change HxWxC to CxHxW.
        sample['img_left'][j] = np.transpose(sample['img_left'][j], (2, 0, 1))
        #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

        # Concatenate rgb, d to rgbd image.    
        sample['img_wls_disp'][j]  = np.expand_dims(sample['img_wls_disp'][j], 0)
        sample['img_left'][j] = np.concatenate([sample['img_left'][j], sample['img_wls_disp'][j]], axis=0)

    imgs_rgbd = np.array(sample['img_left'])
    imgs_rgbd = torch.from_numpy(imgs_rgbd).float().cuda()
    imgs_rgbd = imgs_rgbd.unsqueeze(0)

    vel = trained_model(imgs_rgbd)
    vel = torch.mul(vel, vel_max)

    print(vel)

