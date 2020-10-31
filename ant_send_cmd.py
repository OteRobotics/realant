import zmq
import time

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.connect("tcp://127.0.0.1:3002")

time.sleep(0.1)
sock.send_multipart([b"cmd", b"attach_servos\n"])
time.sleep(0.1)
sock.send_multipart([b"cmd", b"s1 512 s2 512 s3 512 s4 512 s5 512 s6 512 s7 512 s8 512\n"])
time.sleep(1)
sock.send_multipart([b"cmd", b"s1 224 s2 512 s3 224 s4 512 s5 224 s6 512 s7 224 s8 512\n"])
time.sleep(1)
sock.send_multipart([b"cmd", b"reset\n"])
time.sleep(1)
sock.send_multipart([b"cmd", b"detach_servos\n"])

sock.close()
ctx.term()
