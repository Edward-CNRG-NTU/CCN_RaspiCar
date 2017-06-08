import numpy as np
import nengo
import socket
import threading
import struct
import time
import cv2

import nengo.spa as spa

dim = 32
N = 100

TARGET_IP = ('192.168.11.3', 23232)

g_distance = np.zeros([3])


def launch_udp_listener_routine():
    stopper = threading.Event()

    def udp_listener_routine():

        HEADER_FORMAT = 'QHHHHII'
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

        last_time_stamp = 0

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print('listening to port %s' % 23232)
            sock.bind(('', 23232))

            sock.sendto(b'HELLO', ('192.168.11.3', 23232))

            while not stopper.is_set():

                packet = None
                addr = None

                try:
                    (packet, addr) = sock.recvfrom(32 * 1024)
                except socket.error as e:
                    print(e)

                if packet and addr == TARGET_IP:
                    header = struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])

                    if header[0] > last_time_stamp:
                        last_time_stamp = header[0]

                        if header[1] == len(packet) - HEADER_SIZE:
                            print(header)
                            global g_distance
                            g_distance = np.clip(header[2:5], 0, 20)
                            np_data = np.fromstring(packet[HEADER_SIZE:], dtype='uint8')
                            decoded_img = cv2.imdecode(np_data, 1)
                            # cv2.imshow('view', decoded_img)
                            # cv2.waitKey(1)
                        else:
                            print('packet size mismach.')

                    else:
                        print('skipping lagged packet.')

        finally:
            print('closing socket')
            sock.close()

        print('udp_listener_routine stopped!')

    t = threading.Thread(target=udp_listener_routine)
    t.setDaemon(True)
    t.start()

    return stopper


udp_listener_routine_stopper = launch_udp_listener_routine()

model = nengo.Network()
with model:
    stim = nengo.Node(lambda t: (g_distance - 10)/10.0)
    ens = nengo.Ensemble(n_neurons=60, dimensions=3)
    nengo.Connection(stim, ens)

    obstacle = nengo.Ensemble(n_neurons=60, dimensions=3)
    nengo.Connection(ens, obstacle, function=lambda x: x < [-0.2, 0.5, -0.2])

# udp_listener_routine_stopper.set()
