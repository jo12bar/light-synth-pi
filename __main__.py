# __main__.py
#
# Recieves data from the jo12bar/bluesend audio plugin over bluetooth, computes
# the necessary levels that three strips of neopixels should be, and sends data
# to an Arduino over a serial connection.
#
# Currently, just dumps all recieved packets to stdout.
# TODO: Make this work gooder!
#
# Bluetooth stuff based
# on https://github.com/pybluez/pybluez/blob/master/examples/simple/rfcomm-server.py

from bluetooth import *
from uuid import uuid4

# Maximum size, in bytes, that a buffer being recieved from Bluesend can be.
MAX_BT_BUFFER_SIZE = 1024

bt_sock = BluetoothSocket(RFCOMM)
bt_sock.bind(("", PORT_ANY))
bt_sock.listen(1)

port = bt_sock.getsockname()[1]

uuid = str(uuid4())

advertise_service(
        bt_sock,
        "light-synth-pi",
        service_id = uuid,
        service_classes = [uuid, SERIAL_PORT_CLASS],
        profiles = [SERIAL_PORT_PROFILE]
        )

while True:
    print("Waiting for connection on RFCOMM channel %d" % port)
    bluesend_sock, bluesend_info = bt_sock.accept()
    print("Accepted connection from ", bluesend_info)

    try:
        while True:
            data = bluesend_sock.recv(MAX_BT_BUFFER_SIZE)
        
            if (len(data) == 0):
                break

            print("Recieved [%s]" % data)
    except IOError:
        pass

    print("Disconnected.")
    bluesend_sock.close()

bt_sock.close()

print("All done!")
