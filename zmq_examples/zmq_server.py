import zmq
import time
import random
import numpy

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://127.0.0.1:12344")



while True:

	#wait for request from client
	#TODO avoid copying message
	try:
		message_frame = socket.recv(flags=zmq.NOBLOCK,copy=False)
		print("Received message frame with " + str(len(message_frame)) + " bytes")

		# non copying buffer view, but is read only...alternative for this?
		in_buf = memoryview(message_frame)

		audio_in  = numpy.frombuffer(in_buf, dtype=numpy.float32)
		audio_out = numpy.multiply(audio_in, 1)

		#reply to client
		socket.send(audio_out, copy=False);

		# message_rx = socket.recv()
		# print("Received message request: " + str(int.from_bytes(message_rx, byteorder='little') ) )
		# message = str(random.uniform(-1.0,1.0)) + " " + str(random.uniform(-1.0,1.0))
		# message = "reply"
		# socket.send_string(message)


	except zmq.ZMQError:
		pass