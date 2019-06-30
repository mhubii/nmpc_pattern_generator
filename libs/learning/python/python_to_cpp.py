import torch

import utils
from unet_model import UNet
from model import RGBDCNNLSTM

batch_size = 1
sequence_length = 5

# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
trained_model = UNet(utils.RGBD_INPUT_SHAPE, 2, batch_size)
# trained_model = RGBDCNNLSTM(utils.RGBD_INPUT_SHAPE, 2)

trained_model.load_state_dict(torch.load('trained_unet_lstm.pt'))
trained_model.eval()
trained_model.cuda() # already save in eval mode!!!!!!

example = torch.rand(batch_size, sequence_length, utils.IMAGE_CHANNELS, utils.RESIZED_IMAGE_HEIGHT, utils.RESIZED_IMAGE_WIDTH).cuda()

traced_script_module = torch.jit.trace(trained_model, example)
traced_script_module.save('trained_script_module_unet_lstm.pt')
