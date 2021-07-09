import zmq
import time
import random
import numpy

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://127.0.0.1:12344")

count = 0

while True:

	#wait for request from client
	#TODO avoid copying message
	try:
	    #  Wait for next request from client
	    message = socket.recv()
	    print("[" + str(count) + "] Received request: " + str(message))
	    count += 1

	    #  Send reply back to client
	    socket.send_string("Server reply")


	except zmq.ZMQError:
		pass