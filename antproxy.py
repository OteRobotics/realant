# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020
# Copyright Aalto University 2021

import zmq

ctx = zmq.Context()
frontend = ctx.socket(zmq.XSUB)
frontend.bind("tcp://127.0.0.1:3002")
backend = ctx.socket(zmq.XPUB)
backend.bind("tcp://127.0.0.1:3001")
#backend.setsockopt(zmq.XPUB_VERBOSE, True)

try:
    zmq.proxy(frontend, backend)
except KeyboardInterrupt:
    pass
finally:
    frontend.close()
    backend.close()
    ctx.term()
