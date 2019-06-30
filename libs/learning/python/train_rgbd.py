import argparse
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms
from torch.utils.data import DataLoader
from torch.autograd import Variable
from tqdm import tqdm
import torch.optim.lr_scheduler as lr_scheduler

from model import RGBDCNN
from model import RGBDCNNLSTM
import utils
from unet_model import UNet

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

def train(args):
    """
        Load data, build  and train model.
    """
    # Load, pre-process and augment data.
    sequence_length = 5
    data_set = utils.SequenceDataSetGenerator(sequence_length, data_dir=args.data_dir,
                                          transform=transforms.Compose([
                                          utils.PreProcessSequenceRGBDData()
                                      ]))

    train_size = int(0.9*len(data_set))
    valid_size = len(data_set) - train_size

    train_set, valid_set = torch.utils.data.random_split(data_set, [train_size, valid_size])

    train_sampler = utils.SequenceSampler(sequence_length, args.data_dir, train_set.indices.numpy())
    valid_sampler = utils.SequenceSampler(sequence_length, args.data_dir, valid_set.indices.numpy())

    # Data loader for batch generation.
    train_loader = DataLoader(train_set, batch_size=args.batch_size, sampler=train_sampler, drop_last=True)
    valid_loader = DataLoader(valid_set, 1, sampler=valid_sampler, drop_last=True)

    # Save validation indices.
    np.savetxt('validation_indices_rgbd_cnn_lstm.csv', valid_set.indices.numpy())

    # Build model.
    #model = RGBDCNN(utils.RGBD_INPUT_SHAPE, 3, args.batch_size).cuda()
    model = RGBDCNNLSTM(utils.RGBD_INPUT_SHAPE, 2).cuda()
    # model = UNet(utils.RGBD_INPUT_SHAPE, 2, args.batch_size).cuda()

    # Loss and optimizer.
    criterion = nn.MSELoss().cuda()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    #optimizer = torch.optim.Adam(model.parameters(), lr=0.5)
    #lr_lambda = lambda epoch : np.power(0.5, int(epoch))
    #scheduler = lr_scheduler.LambdaLR(optimizer, lr_lambda=lr_lambda)

    # Train model.
    best_loss = float('inf')
    history = []

    # Maximal velocities.
    #vel_max = np.array([0.15, 0.02, 0.2])
    vel_max = np.array([0.15, 0.2])
    vel_max = torch.from_numpy(vel_max).float().cuda()

    for epoch in range(args.epochs):
        #scheduler.step()
        model.train()
        for idx, sample in enumerate(tqdm(train_loader)):
            img_rgbd = Variable(sample['imgs']).cuda()
            vel = Variable(sample['vels']).cuda()
            optimizer.zero_grad()
            vel_out = model(img_rgbd)
            # scale by max vel
            vel_out = torch.mul(vel_out, vel_max)
            loss = criterion(vel_out, vel[:,-1,:])
            loss.backward()
            optimizer.step()

            if idx % 1000 == 0:
                print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                      epoch+1, idx * len(img_rgbd), len(train_loader.dataset),
                      100. * idx / len(train_loader), loss.data.item()))

        # validate
        model.eval()
        avg_loss = 0.
        N = 0
        img_rgbd = []

        for sample in tqdm(valid_loader):
            img_rgbd = Variable(sample['imgs']).cuda()
            vel = Variable(sample['vels']).cuda()
            vel_out = model(img_rgbd)
            vel_out = torch.mul(vel_out, vel_max)
            avg_loss += criterion(vel_out, vel[:,-1,:]).item()
            N += 1

        avg_loss /= N
        history.append(avg_loss)
        print('Avergage Loss: ', avg_loss)

        # Save weights.
        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), 'trained_rgbd_cnn_lstm.pt')

    return history


if __name__ == '__main__':

	# Get command line arguments.
	parser = argparse.ArgumentParser(description='Behavioral Cloning Training Program')
	parser.add_argument('-d', type=str,   help='data directory',   default='data', dest='data_dir')
	parser.add_argument('-l', type=float, help='learning rate',    default=0.001,  dest='learning_rate')
	parser.add_argument('-b', type=int,   help='batch size',       default=128,  dest='batch_size')
	parser.add_argument('-e', type=int,   help='number of epochs', default=10,     dest='epochs')

	args = parser.parse_args()
	
	hist = train(args)

	# Plot results.
	plt.plot(np.linspace(1, args.epochs + 1, len(hist)), hist)
	plt.title("Behavioural Cloning Progress")
	plt.ylabel(r"$\text{MSE}\,/\,(\frac{\text{m}^2}{\text{s}^2})$")
	plt.xlabel(r"$\text{Epochs}\,/\,\#$")
	plt.savefig("behavioural_cloning_progress.png")

	np.savetxt("history.csv", hist)

	# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
	#trained_model = RGBDCNN(utils.RGBD_INPUT_SHAPE, 3, 1)
	# trained_model = UNet(utils.RGBD_INPUT_SHAPE, 2, 1)
	
	# trained_model.load_state_dict(torch.load('trained_rgbd.pt'))

	# example = torch.rand(1, utils.IMAGE_CHANNELS, utils.RESIZED_IMAGE_HEIGHT, utils.RESIZED_IMAGE_WIDTH)

	# traced_script_module = torch.jit.trace(trained_model, example)
	# traced_script_module.save('trained_script_module_rgbd.pt')
