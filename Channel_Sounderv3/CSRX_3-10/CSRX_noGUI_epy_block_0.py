"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, example_param=1.0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='My Block',   # will show up in GRC
            in_sig=[(np.complex64,4095)],
            out_sig=[(np.complex64,4095)]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.example_param = example_param

    def work(self, input_items, output_items):
        """example: multiply with constant"""
#        output_items[0][:] = input_items[0] * self.example_param
        #print(output_items[0][0][indx])        
        #print(abs(output_items[0][0][indx]))        
        for i in range(0,len(input_items[0])):
            indx=abs(input_items[0][i]).argmax()
            #print(max(output_items[0][i]))
            input_items[0][i]=np.roll(input_items[0][i],-1*indx+2)
            output_items[0][i][:] = input_items[0][i] * self.example_param
        return len(output_items[0])
