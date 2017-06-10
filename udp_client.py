import socket
import struct
import time
import sys
import cv2
import numpy as np


TARGET_IP = ('192.168.11.3', 23232)

HEADER_FORMAT = 'QHHHHII'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

count = 0
last_time_stamp = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    server_address = ('', 23232)
    print('binding to %s port %s' % server_address)
    sock.bind(server_address)

    sock.sendto(b'HELLO', ('192.168.11.3', 23232))

    while count < 100:

        t1 = time.time()

        packet = None
        addr = None

        (packet, addr) = sock.recvfrom(32 * 1024)

        if packet and addr == TARGET_IP:
            # count += 1
            header = struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])

            if header[0] > last_time_stamp:
                last_time_stamp = header[0]

                if header[1] == len(packet) - HEADER_SIZE:
                    np_data = np.fromstring(packet[HEADER_SIZE:], dtype='uint8')
                    decoded_img = cv2.imdecode(np_data, 1)
                    OSD = '%d,%3d,%3d,%3d,%5d,%5d' % header[1:]
                    cv2.putText(decoded_img, OSD, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
                    cv2.putText(decoded_img, '%3.3f ms' % (time.time()-t1), (200, 220), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 0, 255), 1)
                    cv2.imshow('view', decoded_img)
                else:
                    print('packet size mismach.')

            else:
                print('skipping lagged packet.')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except socket.error as e:
    print(e)

finally:
    print('closing socket')
    sock.close()
    cv2.destroyAllWindows()

