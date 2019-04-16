import torch
import torch.nn as nn


class RGBDCNN(nn.Module):
    """
        Convolutional neural net for RGBD images.
    """

    def __init__(self, input_shape, dof, batch_size):
        """
            Initialize the CNN.
        """

        super(RGBDCNN, self).__init__()

        self.batch_size = batch_size
        self.cnn = nn.Sequential(
            nn.Conv2d(4, 8, 5, 2),
            nn.ReLU(),
            nn.Conv2d(8, 16, 5, 2),
            nn.ReLU(),
            nn.Conv2d(16, 32, 3, 2),
            nn.ReLU(),
        )

        n = self._get_conv_output(self.cnn, input_shape)

        self.classification = nn.Sequential(
            nn.Linear(n, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, dof)
        )

    def _get_conv_output(self, net, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        input = torch.rand(shape).unsqueeze(0)
        input = torch.autograd.Variable(input)
        output = net(input)
        n = output.numel()
        return n

    def forward(self, rgbd):
        """
            Forward rgb and depth image.
        """

        # Convolutional layers for feature extraction.
        out = self.cnn(rgbd)

        # Flatten.
        out = out.view(self.batch_size, int(out.numel()/self.batch_size))

        # Linear layers for classification.
        out = self.classification(out)

        return out
