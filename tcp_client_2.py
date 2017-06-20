import socket
import struct
import time
import sys
import numpy as np


def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf


message = b'fwd:0.03'

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.11.3', 23233)

try:
    print('connecting to %s port %s' % server_address)
    sock.connect(server_address)

    count = 0

    while count < 10:

        t1 = time.time()

        # Send data
        sock.sendall(message)

        data = sock.recv(128).decode('utf-8')
        print(data)

        print('RTT= %f s' % (time.time() - t1))

        if len(data) == 0:
            print('connection closed.')
            break
        elif data.startswith('ack'):
            pass
        elif data.startswith('error'):
            print('error response.')
            break
        else:
            print('unexpected response.')
            break

        time.sleep(1)
        count += 1

except socket.error as e:
    print(e)

finally:
    print('closing socket')
    sock.close()
