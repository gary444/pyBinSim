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

import logging
import multiprocessing
import enum

import numpy as np
import soundfile as sf

from pybinsim.pose import Pose
from pybinsim.utility import total_size

nThreads = multiprocessing.cpu_count()

class Filter(object):

    def __init__(self, inputfilter, irBlocks, block_size):
        self.ir_blocks = irBlocks
        self.IR_left_blocked = np.reshape(inputfilter[:, 0], (irBlocks, block_size))
        self.IR_right_blocked = np.reshape(inputfilter[:, 1], (irBlocks, block_size))

    def getFilter(self):
        return self.IR_left_blocked, self.IR_right_blocked

    def apply_fadeout(self,window):
        self.IR_left_blocked[self.ir_blocks-1, :] = np.multiply(self.IR_left_blocked[self.ir_blocks-1, :], window)
        self.IR_right_blocked[self.ir_blocks-1, :] = np.multiply(self.IR_right_blocked[self.ir_blocks-1, :], window)

    def apply_fadein(self,window):
        self.IR_left_blocked[0, :] = np.multiply(self.IR_left_blocked[0, :], window)
        self.IR_right_blocked[0, :] = np.multiply(self.IR_right_blocked[0, :], window)

class FilterType(enum.Enum):
    Undefined = 0
    Filter = 1
    LateReverbFilter = 2

class FilterStorage(object):
    """ Class for storing all filters mentioned in the filter list """

    def __init__(self, irSize, block_size, filter_list_name, useHeadphoneFilter = False, headphoneFilterSize = 0, useSplittedFilters = False, lateReverbSize = 0):

        self.log = logging.getLogger("pybinsim.FilterStorage")
        self.log.info("FilterStorage: init")

        self.ir_size = irSize
        self.ir_blocks = irSize // block_size
        self.block_size = block_size
        self.default_filter = Filter(np.zeros((self.ir_size, 2), dtype='float32'),self.ir_blocks,self.block_size)

        # Calculate COSINE-Square crossfade windows
        self.crossFadeOut = np.array(range(0, self.block_size), dtype='float32')
        self.crossFadeOut = np.square(np.cos(self.crossFadeOut/(self.block_size-1)*(np.pi/2)))
        self.crossFadeIn = np.flipud(self.crossFadeOut)

        self.useHeadphoneFilter = useHeadphoneFilter
        if useHeadphoneFilter:
            self.headPhoneFilterSize = headphoneFilterSize
            self.headphone_ir_blocks = headphoneFilterSize // block_size

        self.useSplittedFilters = useSplittedFilters
        if useSplittedFilters:
            self.lateReverbSize = lateReverbSize
            self.late_ir_blocks = lateReverbSize // block_size
            self.default_late_reverb_filter = Filter(np.zeros((self.lateReverbSize, 2), dtype='float32'), self.late_ir_blocks, self.block_size)

        self.filter_list_path = filter_list_name
        self.filter_list = open(self.filter_list_path, 'r')

        self.headphone_filter = None

        # format: [key,{filter}]
        self.filter_dict = {}
        self.late_reverb_filter_dict = {}

        # Start to load filters
        self.load_filters()

    def parse_filter_list(self):
        """
        Generator for filter list lines

        Lines are assumed to have a format like
        0 0 40 1 1 0 brirWav_APA/Ref_A01_1_040.wav

        The headphone filter starts with HPFILTER instead of the positions.

        Lines can be commented with a '#' as first character.

        :return: Iterator of (Pose, filter-path) tuples
        """

        for line in self.filter_list:

            # comment out lines in the list with a '#'
            if line.startswith('#'):
                continue

            line_content = line.split()
            filter_path = line_content[-1]

            # handle headphone filter
            if line.startswith('HPFILTER') and self.useHeadphoneFilter:
                self.log.info("Loading headphone filter: {}".format(filter_path))
                self.headphone_filter = Filter(self.load_filter(filter_path), self.headphone_ir_blocks, self.block_size)
                continue
            elif line.startswith('HPFILTER') :
                self.log.info("Skipping headphone filter: {}".format(filter_path))
                continue

            # handle normal filters and late reverb filters
            filter_value_list = tuple(line_content[1:-1])
            filter_pose = Pose.from_filterValueList(filter_value_list)
            filter_type = FilterType.Undefined

            if line.startswith('FILTER'):
                filter_type = FilterType.Filter

            if line.startswith('LATEREVERB') and self.useSplittedFilters:
                self.log.info("Loading late reverb filter: {}".format(filter_path))
                filter_type = FilterType.LateReverbFilter


            yield filter_pose, filter_path, filter_type

    def load_filters(self):
        """
        Load filters from files

        :return: None
        """

        self.log.info("Start loading filters...")

        for i, (filter_pose, filter_path, filter_type) in enumerate(self.parse_filter_list()):

            if filter_type == FilterType.Undefined:
                continue

            self.log.debug('Loading {}'.format(filter_path))

            if filter_type == FilterType.Filter:
                ## preprocess filters and put them in a dict
                current_filter = Filter(self.load_filter(filter_path), self.ir_blocks, self.block_size)

                # Apply fade out to all filters
                current_filter.apply_fadeout(self.crossFadeOut)

                # create key and store in dict.
                key = filter_pose.create_key()
                self.filter_dict.update({key: current_filter})

            if filter_type == FilterType.LateReverbFilter:
                ## preprocess late reverb filters and put them in a seperate dict
                current_filter = Filter(self.load_filter(filter_path), self.late_ir_blocks, self.block_size)

                # Apply fade in to all late reverb filters
                current_filter.apply_fadein(self.crossFadeIn)

                # create key and store in dict.
                key = filter_pose.create_key()
                self.late_reverb_filter_dict.update({key: current_filter})

        self.log.info("Finished loading filters.")
        #self.log.info("filter_dict size: {}MiB".format(total_size(self.filter_dict) // 1024 // 1024))


    def get_filter(self, pose):
        """
        Searches in the dict if key is available and return corresponding filter
        When no filter is found, defaultFilter is returned which results in silence

        :param pose
        :return: corresponding filter for pose
        """

        key = pose.create_key()

        if key in self.filter_dict:
            self.log.info('Filter found: key: {}'.format(key))
            return self.filter_dict.get(key)
        else:
            self.log.warning('Filter not found: key: {}'.format(key))
            return self.default_filter

    def get_late_reverb_filter(self,pose):
        key = pose.create_key()

        if key in self.late_reverb_filter_dict:
            self.log.info('Late Reverb Filter found: key: {}'.format(key))
            return self.late_reverb_filter_dict.get(key)
        else:
            self.log.warning('Late Reverb Filter not found: key: {}'.format(key))
            return self.default_late_reverb_filter

    def close(self):
        self.log.info('FilterStorage: close()')
        # TODO: do something in here?

    def get_headphone_filter(self):
        if self.headphone_filter is None:
            raise RuntimeError("Headphone filter not loaded")

        return self.headphone_filter

    def load_filter(self, filter_path):

        current_filter, fs = sf.read(filter_path, dtype='float32')

        #TODO: Catch or warn filter sizes depending on filter type

        #filter_size = np.shape(current_filter)

        # Fill filter with zeros if to short
        #if filter_size[0] < self.ir_size:
        #    self.log.warning('Filter to short: Fill up with zeros')
        #    current_filter = np.concatenate((current_filter, np.zeros((self.ir_size - filter_size[0], 2))), 0)

        return current_filter
