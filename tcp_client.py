import socket
import struct
import time
import sys
import cv2
import numpy as np


def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Connect the socket to the port where the server is listening
server_address = ('192.168.11.3', 23232)

message = b'QUERY_IMAGE'

try:
    print('connecting to %s port %s' % server_address)
    sock.connect(server_address)

    count = 0

    while count < 100:

        t1 = time.time()

        # Send data
        sock.sendall(message)

        header = None
        img_data = b''

        header = sock.recv(16)
        header = struct.unpack('HHHHII', header)
        data_len = header[0]
        while data_len:
            buf = sock.recv(data_len)
            if not buf:
                break
            else:
                img_data += buf
                data_len -= len(buf)

        print(1/(time.time()-t1))

        if len(img_data) > 3000:
            # count += 1
            np_data = np.fromstring(img_data, dtype='uint8')
            decoded_img = cv2.imdecode(np_data, 1)
            OSD = '%d,%3d,%3d,%3d,%5d,%5d' % header
            cv2.putText(decoded_img, OSD, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
            cv2.putText(decoded_img, '%3.3f ms' % (time.time() - t1), (200, 220), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 1)
            cv2.imshow('view', decoded_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                count = 100

        else:
            time.sleep(0.2)

except socket.error as e:
    print(e)

finally:
    print('closing socket')
    sock.close()
    cv2.destroyAllWindows()
