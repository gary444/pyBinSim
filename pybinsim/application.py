# This file is part of the pyBinSim project.
#
# Copyright (c) 2017 A. Neidhardt, F. Klein, N. Knoop, T. Köllmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

""" Module contains main loop and configuration of pyBinSim """
import logging
import time
import numpy as np
import pyaudio
import zmq

from pybinsim.convolver import ConvolverFFTW
from pybinsim.filterstorage import FilterStorage
from pybinsim.osc_receiver import OscReceiver
from pybinsim.pose import Pose
from pybinsim.soundhandler import SoundHandler


def parse_boolean(any_value):

    if type(any_value) == bool:
        return any_value

    # str -> bool
    if any_value == 'True':
        return True
    if any_value == 'False':
        return False

    return None


class BinSimConfig(object):
    def __init__(self):

        self.log = logging.getLogger("pybinsim.BinSimConfig")

        # Default Configuration
        self.configurationDict = {'soundfile': '',
                                  'blockSize': 256,
                                  'filterSize': 16384,
                                  'filterList': 'brirs/filter_list_kemar5.txt',
                                  'enableCrossfading': False,
                                  'useHeadphoneFilter': False,
                                  'loudnessFactor': float(1),
                                  'maxChannels': 8,
                                  'samplingRate': 44100,
                                  'loopSound': True}

    def read_from_file(self, filepath):
        config = open(filepath, 'r')

        for line in config:
            line_content = str.split(line)
            key = line_content[0]
            value = line_content[1]

            if key in self.configurationDict:
                config_value_type = type(self.configurationDict[key])

                if config_value_type is bool:
                    # evaluate 'False' to False
                    boolean_config = parse_boolean(value)

                    if boolean_config is None:
                        self.log.warning(
                            "Cannot convert {} to bool. (key: {}".format(value, key))

                    self.configurationDict[key] = boolean_config
                else:
                    # use type(str) - ctors of int, float, ...
                    self.configurationDict[key] = config_value_type(value)

            else:
                self.log.warning('Entry ' + key + ' is unknown')

    def get(self, setting):
        return self.configurationDict[setting]


class BinSim(object):
    """
    Main pyBinSim program logic
    """

    def __init__(self, config_file):

        self.log = logging.getLogger("pybinsim.BinSim")
        self.log.info("BinSim: init")

        # Read Configuration File
        self.config = BinSimConfig()
        self.config.read_from_file(config_file)

        self.inChannels = 2 # unity sends stereo audio
        self.channelsToProcess = self.config.get('maxChannels')
        self.outChannels = 2
        self.sampleRate = self.config.get('samplingRate')
        self.blockSize = self.config.get('blockSize')

        self.result = None
        self.block = None
        self.stream = None

        self.convolverWorkers = []
        self.convolverHP, self.convolvers, self.filterStorage, self.oscReceiver = self.initialize_pybinsim()


        self.zmq_ip = "127.0.0.1";
        self.zmq_port = "12346";
        self.init_zmq()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__cleanup()


    def init_zmq(self):
        self.log.info(("BinSim: init ZMQ, IP: {}, port {}").format(self.zmq_ip, self.zmq_port))
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REP)
        self.zmq_socket.bind("tcp://" + self.zmq_ip + ":" + self.zmq_port )

    # def clean_zmq(self):


    def run_server(self):
        self.log.info("BinSim: run_server")

        while True:
            #wait for request from client
            #TODO avoid copying message
            try:
                message_frame = self.zmq_socket.recv(flags=zmq.NOBLOCK,copy=False)
                print("Received message frame with " + str(len(message_frame)) + " bytes")

                # non copying buffer view, but is read only...alternative for this?
                in_buf = memoryview(message_frame)

                stereo_audio_in = np.frombuffer(in_buf, dtype=np.float32).reshape((self.blockSize, self.inChannels))
                # take first channel of input only
                self.block[:] = stereo_audio_in[:,0]

                # get channel id from second buffer element
                convChannel = int(stereo_audio_in[0,1])

                # is this a copy? can itbe avoided?
                # self.block[:] = np.frombuffer(in_buf, dtype=np.float32)


                self.process_block(convChannel);

                # self.result[:,0] = np.multiply(self.block, 0.5)
                # self.result[:,1] = np.multiply(self.block, 1.0)

                #reply to client
                self.zmq_socket.send(self.result, copy=False);

            except zmq.ZMQError:
                pass



    def initialize_pybinsim(self):
        self.result = np.empty([self.blockSize, 2], dtype=np.float32)
        self.block = np.empty(self.blockSize, dtype=np.float32)

        # Create FilterStorage
        filterStorage = FilterStorage(self.config.get('filterSize'),
                                      self.blockSize,
                                      self.config.get('filterList'))

        # Start an oscReceiver
        oscReceiver = OscReceiver()
        oscReceiver.start_listening()
        time.sleep(1)

        # Create N convolvers depending on the number of wav channels
        self.log.info('Number of input Channels: ' + str(self.inChannels))
        self.log.info('Number of channels to process: ' + str(self.channelsToProcess))
        convolvers = [None] * self.channelsToProcess
        for n in range(self.channelsToProcess):
            convolvers[n] = ConvolverFFTW(self.config.get(
                'filterSize'), self.blockSize, False)

        # HP Equalization convolver
        convolverHP = None
        if self.config.get('useHeadphoneFilter'):
            convolverHP = ConvolverFFTW(self.config.get(
                'filterSize'), self.blockSize, True)
            hpfilter = filterStorage.get_headphone_filter()
            convolverHP.setIR(hpfilter, False)

        return convolverHP, convolvers, filterStorage, oscReceiver

    def close(self):
        self.log.info("BinSim: close")

    def __cleanup(self):
        # Close everything when BinSim is finished
        self.filterStorage.close()
        self.close()

        self.oscReceiver.close()

        for n in range(self.channelsToProcess):
            self.convolvers[n].close()

        if self.config.get('useHeadphoneFilter'):
            if self.convolverHP:
                self.convolverHP.close()

    def process_block(self, convChannel):


        # Update Filters and run each convolver with the current block
        # for n in range(binsim.soundHandler.get_sound_channels()):

        # convChannel = 0

        # Get new Filter
        if self.oscReceiver.is_filter_update_necessary(convChannel):
            filterValueList = self.oscReceiver.get_current_values(convChannel)
            filter = self.filterStorage.get_filter(
                Pose.from_filterValueList(filterValueList))
            # self.convolvers[convChannel].setIR(filter, self.config.get('enableCrossfading'))
            self.convolvers[convChannel].setIR(filter, False)

        self.result[:, 0], self.result[:,1] = self.convolvers[convChannel].process(self.block)

        # Finally apply Headphone Filter
        if self.config.get('useHeadphoneFilter'):
            self.result[:, 0], self.result[:,1] = self.convolverHP.process(self.result)

        # Scale data
        # self.result = np.divide(self.result, float(
        #     (binsim.soundHandler.get_sound_channels()) * 2))
        self.result = np.multiply(
            self.result, self.config.get('loudnessFactor'))

        if np.max(np.abs(self.result)) > 1:
            self.log.warn('Clipping occurred: Adjust loudnessFactor!')

        # When the last block is small than the blockSize, this is probably the end of the file.
        # Call pyaudio to stop after this frame
        # Should not be the case for current soundhandler implementation
        if self.block.size < self.blockSize:
            # pyaudio.paContinue = 1
            self.log.warn('Block size too small: not handled yet')


