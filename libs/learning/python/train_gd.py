import argparse
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms
from torch.utils.data import DataLoader
from torch.autograd import Variable

from model import GDCNN
import utils

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

def train(args):
    """
        Load data, build  and train model.
    """
    # Load, pre-process and augment data.
    data_set = utils.DataSetGenerator(data_dir=args.data_dir,
                                      transform=transforms.Compose([
                                          utils.PreProcessGDData()
                                      ]))

    # Data loader for batch generation.
    data_loader = DataLoader(data_set, batch_size=args.batch_size, drop_last=True)

    # Build model.
    model = GDCNN(utils.GD_INPUT_SHAPE, 2, args.batch_size).cuda()

    # Loss and optimizer.
    criterion = nn.MSELoss().cuda()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)

    # Train model.
    best_loss = float('inf')
    history = []

    for epoch in range(args.epochs):
        for idx, sample in enumerate(data_loader):
            img_gd = Variable(sample['img']).cuda()
            vel = Variable(sample['vel']).cuda()
            optimizer.zero_grad()
            vel_out = model(img_gd)
            loss = criterion(vel_out, vel)
            loss.backward()
            optimizer.step()

            # Save weights.
            if loss.data.item() < best_loss:
                best_loss = loss.data.item()
                torch.save(model.state_dict(), 'trained_gd.pt')

            if idx % 10 == 0:
                print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                      epoch+1, idx * len(img_gd), len(data_loader.dataset),
                      100. * idx / len(data_loader), loss.data.item()))

                history.append(loss.data.item())

    return history


if __name__ == '__main__':

	# Get command line arguments.
	parser = argparse.ArgumentParser(description='Behavioral Cloning Training Program')
	parser.add_argument('-d', type=str,   help='data directory',   default='data', dest='data_dir')
	parser.add_argument('-l', type=float, help='learning rate',    default=0.001,  dest='learning_rate')
	parser.add_argument('-b', type=int,   help='batch size',       default=4096,  dest='batch_size')
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
	trained_model = GDCNN(utils.GD_INPUT_SHAPE, 2, 1)
	
	trained_model.load_state_dict(torch.load('trained_gd.pt'))

	example = torch.rand(1, 2, utils.IMAGE_HEIGHT, utils.IMAGE_WIDTH)

	traced_script_module = torch.jit.trace(trained_model, example)
	traced_script_module.save('trained_script_module_gd.pt')
