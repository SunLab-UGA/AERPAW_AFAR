#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.8.5.0-6-g57bd109d

from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time


class CSTX_noGUI(gr.top_block):

    def __init__(self, args='', freq=3.5e9, gaintx=76):
        gr.top_block.__init__(self, "Not titled yet")

        ##################################################
        # Parameters
        ##################################################
        self.args = args
        self.freq = freq
        self.gaintx = gaintx

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 16
        self.samp_rate = samp_rate = 2e6
        self.alpha = alpha = 0.99

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", args)),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            '',
        )
        self.uhd_usrp_sink_0.set_center_freq(freq, 0)
        self.uhd_usrp_sink_0.set_gain(gaintx, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())
        self.root_raised_cosine_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.root_raised_cosine(
                sps,
                samp_rate,
                samp_rate/sps,
                alpha,
                10*sps+1))
        self.interp_fir_filter_xxx_0 = filter.interp_fir_filter_ccc(sps, [1]+[0]*(sps-1))
        self.interp_fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_glfsr_source_x_0 = digital.glfsr_source_b(12, True, 0, 1)
        self.digital_chunks_to_symbols_xx_0 = digital.chunks_to_symbols_bc((-1,1), 1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_cc(0.6)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.digital_chunks_to_symbols_xx_0, 0), (self.interp_fir_filter_xxx_0, 0))
        self.connect((self.digital_glfsr_source_x_0, 0), (self.digital_chunks_to_symbols_xx_0, 0))
        self.connect((self.interp_fir_filter_xxx_0, 0), (self.root_raised_cosine_filter_0, 0))
        self.connect((self.root_raised_cosine_filter_0, 0), (self.blocks_multiply_const_vxx_0, 0))


    def get_args(self):
        return self.args

    def set_args(self, args):
        self.args = args

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_sink_0.set_center_freq(self.freq, 0)

    def get_gaintx(self):
        return self.gaintx

    def set_gaintx(self, gaintx):
        self.gaintx = gaintx
        self.uhd_usrp_sink_0.set_gain(self.gaintx, 0)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.interp_fir_filter_xxx_0.set_taps([1]+[0]*(self.sps-1))
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))




def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--args", dest="args", type=str, default='',
        help="Set args [default=%(default)r]")
    parser.add_argument(
        "--freq", dest="freq", type=eng_float, default="3.5G",
        help="Set freq [default=%(default)r]")
    parser.add_argument(
        "--gaintx", dest="gaintx", type=eng_float, default="76.0",
        help="Set gaintx [default=%(default)r]")
    return parser


def main(top_block_cls=CSTX_noGUI, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(args=options.args, freq=options.freq, gaintx=options.gaintx)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
