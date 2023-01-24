import logging
import threading
import zmq
import numpy as np

from pythonosc import dispatcher
from pythonosc import osc_server


class PoseParser(object):
    


    def __init__(self, zmq_context):

        self.log = logging.getLogger("pybinsim.PoseParser")
        self.log.info("PoseParser: init")

        self.maxChannels = 100

        # Default values; Stores filter keys for all channles/convolvers
        self.filtersUpdated = [True] * self.maxChannels

        self.defaultValue = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.valueList = [self.defaultValue] * self.maxChannels

        self.poseParseThread = threading.Thread(target=self.parse_pose_input, args=())

        self.zmq_ip = "127.0.0.1";
        self.zmq_port = "12345";
        self.zmq_socket = zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://" + self.zmq_ip + ":" + self.zmq_port )
        self.zmq_socket.subscribe("");

        self.keepParsing = True

    def start_parsing(self):
        self.log.info("PoseParser: start parsing")
        self.poseParseThread.start()        

    def stop_parsing(self):
        self.log.info("PoseParser: stop parsing")
        self.keepParsing = False
        self.poseParseThread.join()

    def should_keep_running(self):
        return self.keepParsing

    # def parse_pose_input(self, poseData):
    def parse_pose_input(self):

        self.log.info("PoseParser: start parse thread")


        while self.keepParsing == True:

            poseMsg = self.zmq_socket.recv()
            poseBuffer = memoryview(poseMsg)

            poseVec = np.frombuffer(poseMsg, dtype=np.uint32);

            print(poseVec)

            if (poseVec[0] == 999):
                self.keepParsing = False;
                self.log.info("PoseParser: initiate end of parsing")

            # else:
                # current_channel = poseVec[0]
                # poseData = tuple(poseVec[1], poseVec[2])
                # # compare poseData with valueList for channel
                # if poseData != self.valueList[current_channel][:len(poseData)]:
                #     self.filtersUpdated[current_channel] = True
                #     self.valueList[current_channel] = poseData + self.defaultValue[len(poseData):]            

                #     # self.log.info("Channel: {}".format(str(channel)))
                #     self.log.info("Args: {}".format(str(poseData)))

    def is_filter_update_necessary(self, channel):
        """ Check if there is a new filter for channel """
        return self.filtersUpdated[channel]

    def get_current_values(self, channel):
        """ Return key for filter """
        self.filtersUpdated[channel] = False
        return self.valueList[channel]
