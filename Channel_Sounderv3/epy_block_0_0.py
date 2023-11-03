"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
import pmt
import math

class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Find the Peak Correlation Value and Phase where it occurs within the vector"""

    def __init__(self, vector_length=4095):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Max and Phase',   # will show up in GRC
            in_sig=[(np.complex64,vector_length)],
            out_sig=[(np.float32,1),
                     (np.float32,1),
                     (np.float32,1)]
        )    
        self.vector_length = vector_length


    def work(self, input_items, output_items):
        """abs, argmax, putput the""" 
        # take the absolute value of all vector values
        abs_vec = np.abs(input_items[0])
        # find the index of the max value
        indx = np.argmax(abs_vec)
        # get the max value
        max_val = np.max(abs_vec)
        # write output items
        output_items[0][0] = max_val
        output_items[1][0] = indx*(2*math.pi)/self.vector_length # normalize the offset phase to radians
        output_items[2][0] = indx # return the raw index of the max value

        # return the output item(s) vector length to the scheduler
        # (here it is 1 vector item for each sample output)
        return 1 # or len(output_items[0]) 
