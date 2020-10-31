import zmq

ctx = zmq.Context()
frontend = ctx.socket(zmq.XSUB)
frontend.bind("tcp://*:3002")
backend = ctx.socket(zmq.XPUB)
backend.bind("tcp://*:3001")
#backend.setsockopt(zmq.XPUB_VERBOSE, True)

try:
    zmq.proxy(frontend, backend)
except KeyboardInterrupt:
    pass
finally:
    frontend.close()
    backend.close()
    ctx.term()
