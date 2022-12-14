"""
File: logger.py
Modified by: Senthil Purushwalkam
Code referenced from https://gist.github.com/gyglim/1f8dfb1b5c82627ae3efcfbbadb9f514
Email: spurushw<at>andrew<dot>cmu<dot>edu
Github: https://github.com/senthilps8
Description: 
"""

import tensorflow as tf
import torch

from torch.autograd import Variable
import numpy as np
import scipy.misc
import os
from PIL import Image
try:
    from StringIO import StringIO  # Python 2.7
except ImportError:
    from io import BytesIO         # Python 3.x


class Logger(object):

    def __init__(self, log_dir, name=None):
        """Create a summary writer logging to log_dir."""
        if name is None:
            name = 'temp'
        self.name = name
        if name is not None:
            try:
                os.makedirs(os.path.join(log_dir, name))
            except:
                pass
            self.writer = tf.summary.create_file_writer(os.path.join(log_dir, name),
                                                filename_suffix=name)
        else:
            self.writer = tf.summary.create_file_writer(log_dir, filename_suffix=name)

    def scalar_summary(self, tag, value, step):
        """Log a scalar variable."""
        with self.writer.as_default():
            summary = tf.summary.scalar(tag, value.cpu(), step=step)
            # summary = tf.summary(value=[tf.summary.value(tag=tag, simple_value=value)])
            # self.writer.add_summary(summary, step)
            self.writer.flush()
            
    def image_summary(self, tag, images, step):
        """Log a list of images."""
        
        with self.writer.as_default():
            img_summaries = []
            for i, img in enumerate(images):
                # Write the image to a string
                try:
                    s = StringIO()
                except:
                    s = BytesIO()
                
                img2 = Image.fromarray(img.cpu())
                img2.save(s, format="png")
                # scipy.misc.toimage(img).save(s, format="png")
    
                # Create an Image object
                img_sum = tf.summary.image(s.getvalue(),img,step)
                                           # height=img.shape[0],
                                           # width=img.shape[1])
                # Create a Summary value
                
                img_summaries.append(tf.summary.value(tag='%s/%d' % (tag, i), image=img_sum))
        
        self.writer.flush()
        
        # Create and write Summary
        # summary = tf.summary(value=img_summaries)
        # self.writer.add_summary(summary, step)

    def histo_summary(self, tag, values, step, bins=1000):
        """Log a histogram of the tensor of values."""

        # Create a histogram using numpy
        counts, bin_edges = np.histogram(values, bins=bins)

        # Fill the fields of the histogram proto
        hist = tf.HistogramProto()
        hist.min = float(np.min(values))
        hist.max = float(np.max(values))
        hist.num = int(np.prod(values.shape))
        hist.sum = float(np.sum(values))
        hist.sum_squares = float(np.sum(values**2))

        # Drop the start of the first bin
        bin_edges = bin_edges[1:]

        # Add bin edges and counts
        for edge in bin_edges:
            hist.bucket_limit.append(edge)
        for c in counts:
            hist.bucket.append(c)

        # Create and write Summary
        summary = tf.summary(value=[tf.summary.Value(tag=tag, histo=hist)])
        self.writer.add_summary(summary, step)
        self.writer.flush()

    def to_np(self, x):
        return x.data.cpu().numpy()

    def to_var(self, x):
        if torch.cuda.is_available():
            x = x.cuda()
        return Variable(x)

    def model_param_histo_summary(self, model, step):
        """log histogram summary of model's parameters
        and parameter gradients
        """
        for tag, value in model.named_parameters():
            if value.grad is None:
                continue
            tag = tag.replace('.', '/')
            tag = self.name+'/'+tag
            self.histo_summary(tag, self.to_np(value), step)
            self.histo_summary(tag+'/grad', self.to_np(value.grad), step)

