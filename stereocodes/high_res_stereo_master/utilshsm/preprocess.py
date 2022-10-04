import torchvision.transforms as transforms
import numpy as np
import torch
from torchvision.transforms.functional import pil_to_tensor

__imagenet_stats = {'mean': [0.485, 0.456, 0.406],
                   'std': [0.229, 0.224, 0.225]}

class toTensorLegacy(object):
    def __call__(self, pic):
        """
        Args:
            pic (PIL or numpy.ndarray): Image to be converted to tensor

        Returns:
            Tensor: Converted image.
        """
        if isinstance( pic, np.ndarray ):
                # This is what TorchVision 0.2.0 returns for transforms.toTensor() for np.ndarray
        	return torch.from_numpy( pic.transpose((2, 0, 1))).float().div(255)
        else:
            # pil images are always W x H
            # pic_np = np.asarray(pil_img)  will be H x W x C
            # pil_to_tensor automatically changes to C x H x W
            return pil_to_tensor( pic ).float().div(255)
            
    def __repr__(self):
        return self.__class__.__name__ + '()'


def get_transform():
    t_list = [
        toTensorLegacy(),
        transforms.Normalize(**__imagenet_stats),
    ]

    return transforms.Compose(t_list)
