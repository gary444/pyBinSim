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
import sys
import numpy as np
import zmq

from pybinsim.convolver import ConvolverFFTW
from pybinsim.filterstorage import FilterStorage
from pybinsim.inline_pose_parser import InlinePoseParser
from pybinsim.pose import Pose


def parse_boolean(any_value):

    if type(any_value) == bool:
        return any_value

    # str -> bool
    if any_value == 'True':
        return True
    if any_value == 'False':
        return False

    return None

def quantize_azimuth(azimuth_in):
    # assume 5 degree steps 
    azimuth_out = int(round(azimuth_in / 5.0) * 5) % 360
    return azimuth_out 

def quantize_elevation(elevation_in):
    elevation_out = int(0)
    return elevation_out

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
                                  'headphoneFilterSize': 16384,
                                  'loudnessFactor': float(1),
                                  'maxChannels': 8,
                                  'samplingRate': 44100,
                                  'loopSound': True,
                                  'useSplittedFilters': False,
                                  'lateReverbSize': 16384,
                                  'dirFilterSize': 16384,
                                  'pauseConvolution': False,
                                  'pauseAudioPlayback': False,
                                  'serverIPAddress': '127.0.0.1',
                                  'serverPort': '12346'}

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

    def set(self, setting, value):
        value = parse_boolean(value)
        if type(self.configurationDict[setting]) == type(value):
            self.configurationDict[setting] = value
        else:
            self.log.warning('New value for entry ' + setting + ' has wrong type: ' + str(type(value)))

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
        self.outChannels = 2
        self.maxChannels = self.config.get('maxChannels')
        self.blockSize = self.config.get('blockSize')

        self.result = None
        self.block = None
        self.stream = None

        self.convolverWorkers = []
        #self.convolverHP, self.convolvers, self.filterStorage, self.oscReceiver, self.soundHandler = self.initialize_pybinsim()
        self.convolverHP, self.convolvers, self.filterStorage = self.initialize_pybinsim()
        
        self.poseParser = InlinePoseParser(self.maxChannels)
        
        self.zmq_ip = self.config.get('serverIPAddress')
        self.zmq_port = self.config.get('serverPort')
        self.init_zmq()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__cleanup()
        
    def init_zmq(self):
        self.log.info(f'BinSim: init ZMQ, IP: {self.zmq_ip}, Port: {self.zmq_port}')
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REP)
        self.zmq_socket.bind('tcp://' + self.zmq_ip + ':' + self.zmq_port)

    def run_server(self):
        self.log.info('BinSim: run_server')
        
        while True:
            # wait for request from client
            # TODO: avoid copying message
            try:
                message_frame = self.zmq_socket.recv(flags=zmq.NOBLOCK, copy=False)
                
                # non copying buffer view, but is read only...alternative for this?
                in_buf = memoryview(message_frame)

                stereo_audio_in = np.frombuffer(in_buf, dtype=np.float32).reshape((self.blockSize, self.inChannels))
                # take first channel of input only as input for convolution
                self.block[:] = stereo_audio_in[:,0]

                # parse audio packet metadata from second input channel
                convChannel          = int(stereo_audio_in[0, 1])
                # lst_to_src_azimuth   = quantize_azimuth  (stereo_audio_in[1, 1])
                # lst_to_src_elevation = quantize_elevation(stereo_audio_in[2, 1])
                # src_to_lst_azimuth   = quantize_azimuth  (stereo_audio_in[3, 1])
                # src_to_lst_elevation = quantize_elevation(stereo_audio_in[4, 1])
                lst_to_src_azimuth   = stereo_audio_in[1, 1]
                lst_to_src_elevation = stereo_audio_in[2, 1]
                src_to_lst_azimuth   = stereo_audio_in[3, 1]
                src_to_lst_elevation = stereo_audio_in[4, 1]
                lst_to_src_dist      = stereo_audio_in[5, 1]

                self.log.info(f'lst->src azi: {lst_to_src_azimuth}')
                self.log.info(f'lst->src ele: {lst_to_src_elevation}')
                self.log.info(f'src->lst azi: {src_to_lst_azimuth}')
                self.log.info(f'src->lst ele: {src_to_lst_elevation}')
                self.log.info(f'distance: {lst_to_src_dist}')

                # hrtf filters are reversed...
                lst_to_src_azimuth = (360 - lst_to_src_azimuth) % 360

                # elevation filters range from 0 to 180, not -90 to 90
                lst_to_src_elevation += 90
                src_to_lst_elevation += 90


                # TODO: Change range perhaps...
                reference_dist = 1.25
                max_dist = 10
                min_dist = 0.01
                relative_dist = reference_dist / lst_to_src_dist
                relative_dist = min(max(min_dist, relative_dist), max_dist)

                # read rowmajor matrices from audio packet
                src_transform = stereo_audio_in[6:22, 1] .reshape(4, 4)
                lst_transform = stereo_audio_in[22:38, 1].reshape(4, 4)

                # correct 30 degree offset - not needed anymore
                #lst_to_src_azimuth = (lst_to_src_azimuth + 30) % 360
                
                # print("Channel: " + str(convChannel))
                # print("Listener to source Angle: " + str(lst_to_src_azimuth) + " / " + str(lst_to_src_elevation))
                # print("Source to listener Angle: " + str(src_to_lst_azimuth) + " / " + str(src_to_lst_elevation))
                # print("Source to listener distance: " + str(lst_to_src_dist))
                # print("Source transform: " + str(src_transform))
                # print("Listener transform: " + str(lst_transform))

                self.poseParser.parse_pose_input(convChannel, lst_to_src_azimuth, lst_to_src_elevation,
                                                 src_to_lst_azimuth, src_to_lst_elevation)

                self.process_block(convChannel, relative_dist)

                #reply to client
                self.zmq_socket.send(self.result, copy=False)

            except zmq.ZMQError:
                pass


    def initialize_pybinsim(self):
        self.result = np.empty([self.blockSize, 2], dtype=np.float32)
        #self.block = np.empty([self.nChannels, self.blockSize], dtype=np.float32)
        self.block = np.empty(self.blockSize, dtype=np.float32)

        # Create FilterStorage
        filterStorage = FilterStorage(self.config.get('filterSize'),
                                      self.blockSize,
                                      self.config.get('filterList'),
                                      self.config.get('useHeadphoneFilter'),
                                      self.config.get('headphoneFilterSize'),
                                      self.config.get('useSplittedFilters'),
                                      self.config.get('lateReverbSize'),
                                      self.config.get('dirFilterSize'))

        # Create N convolvers depending on the number of wav channels
        self.log.info('Number of input channels: ' + str(self.inChannels))
        self.log.info('Number of channels to process: ' + str(self.maxChannels))
        convolvers = [None] * self.maxChannels
        for n in range(self.maxChannels):
            convolvers[n] = ConvolverFFTW(self.config.get('filterSize'), self.blockSize, False, self.config.get('useSplittedFilters'), self.config.get('lateReverbSize'))

        # HP Equalization convolver
        convolverHP = None
        if self.config.get('useHeadphoneFilter'):
            convolverHP = ConvolverFFTW(self.config.get(
                'headphoneFilterSize'), self.blockSize, True)
            hpfilter = filterStorage.get_headphone_filter()
            convolverHP.setIR(hpfilter, False)

        return convolverHP, convolvers, filterStorage

    def close(self):
        self.log.info('BinSim: close')

    def __cleanup(self):
        # Close everything when BinSim is finished
        self.filterStorage.close()
        self.close()

        for n in range(self.maxChannels):
            self.convolvers[n].close()

        if self.config.get('useHeadphoneFilter'):
            if self.convolverHP:
                self.convolverHP.close()

    def process_block(self, convChannel, dist):
        # Update Filter and run convolver with the current block
        
        # Get new Filter
        # TODO: change filterValueList to something usable - where, though?
        if self.poseParser.is_filter_update_necessary(convChannel):
            filterValueList = self.poseParser.get_current_values(convChannel)
            filter = self.filterStorage.get_filter(Pose.from_filterValueList(filterValueList))
            fvl = list(filterValueList[3:]) + [ 0, 0, 0]
            dir_filter = self.filterStorage.get_directivity_filter(Pose.from_filterValueList(fvl))
            self.convolvers[convChannel].setIR(filter, self.config.get('enableCrossfading'), dist, dir_filter)

            if self.config.get('useSplittedFilters'):
                lr_filter = self.filterStorage.get_late_reverb_filter(Pose.from_filterValueList(filterValueList))
                self.convolvers[convChannel].setLateReverb(lr_filter, self.config.get('enableCrossfading'))
        
        self.result[:, 0], self.result[:, 1] = self.convolvers[convChannel].process(self.block)
        
        # Apply headphone filter
        if self.config.get('useHeadphoneFilter'):
            self.result[:, 0], self.result[:, 1] = self.convolverHP.process(self.result)
            
        # Scale data if required
        self.result = np.multiply(
            self.result, self.config.get('loudnessFactor'))

        if np.max(np.abs(self.result)) > 1:
            self.log.warn('Clipping occurred: Adjust loudnessFactor!')

        # if self.block.size < self.blockSize:
            # self.log.warn('Block size too small: not handled yet')
        
## TODO: is there still stuff to change in the process function
##       will we need to take it from below
def audio_callback(binsim):
    """ Wrapper for callback to hand over custom data """
    assert isinstance(binsim, BinSim)

    # The python-sounddevice Callback
    def callback(outdata, frame_count, time_info, status):
        # print("python-sounddevice callback")

        if "debugpy" in sys.modules:
            import debugpy
            debugpy.debug_this_thread()

        # Update config
        binsim.current_config = binsim.oscReceiver.get_current_config()

        # Update audio files
        current_soundfile_list = binsim.oscReceiver.get_sound_file_list()
        if current_soundfile_list:
            binsim.soundHandler.request_new_sound_file(current_soundfile_list)

        # Get sound block. At least one convolver should exist
        amount_channels = binsim.soundHandler.get_sound_channels()
        if amount_channels == 0:
            return

        if binsim.current_config.get('pauseAudioPlayback'):
            binsim.block[:amount_channels, :] = binsim.soundHandler.read_zeros()
        else:
            binsim.block[:amount_channels, :] = binsim.soundHandler.buffer_read()

        if binsim.current_config.get('pauseConvolution'):
            if binsim.soundHandler.get_sound_channels() == 2:
                binsim.result = np.transpose(binsim.block[:binsim.soundHandler.get_sound_channels(), :])
            else:
                mix = np.mean(binsim.block[:binsim.soundHandler.get_sound_channels(), :], 0)
                binsim.result[:, 0] = mix
                binsim.result[:, 1] = mix
        else:
            # Update Filters and run each convolver with the current block
            for n in range(amount_channels):

                # Get new Filter
                if binsim.oscReceiver.is_filter_update_necessary(n):
                    filterValueList = binsim.oscReceiver.get_current_filter_values(n)
                    filter = binsim.filterStorage.get_filter(Pose.from_filterValueList(filterValueList))
                    binsim.convolvers[n].setIR(filter, callback.config.get('enableCrossfading'))
                    
                # Get new late reverb Filter
                if binsim.oscReceiver.is_late_reverb_update_necessary(n):
                    lateReverbValueList = binsim.oscReceiver.get_current_late_reverb_values(n)
                    latereverbfilter = binsim.filterStorage.get_late_reverb_filter(Pose.from_filterValueList(lateReverbValueList))
                    binsim.convolvers[n].setLateReverb(latereverbfilter, callback.config.get('enableCrossfading'))

                left, right = binsim.convolvers[n].process(binsim.block[n, :])

                # Sum results from all convolvers
                if n == 0:
                    binsim.result[:, 0] = left
                    binsim.result[:, 1] = right
                else:
                    binsim.result[:, 0] = np.add(binsim.result[:, 0], left)
                    binsim.result[:, 1] = np.add(binsim.result[:, 1], right)

            # Finally apply Headphone Filter
            if callback.config.get('useHeadphoneFilter'):
                binsim.result[:, 0], binsim.result[:, 1] = binsim.convolverHP.process(binsim.result)

        # Scale data
        binsim.result = np.divide(binsim.result, float((amount_channels) * 2))
        binsim.result = np.multiply(binsim.result, callback.config.get('loudnessFactor'))

        outdata[:, 0] = binsim.result[:, 0]
        outdata[:, 1] = binsim.result[:, 1]
        
        # Report buffer underrun
        if status == 4:
            binsim.log.warn('Output buffer underrun occurred')

        # Report clipping
        if np.max(np.abs(binsim.result)) > 1:
            binsim.log.warn('Clipping occurred: Adjust loudnessFactor!')

    callback.config = binsim.config

    return callback
