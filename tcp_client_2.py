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


message = b'test'

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.11.3', 23233)
print('connecting to %s port %s' % server_address)
sock.connect(server_address)

try:
    count = 0

    while count < 100:

        t1 = time.time()

        # Send data
        sock.sendall(message)

        data = sock.recv(128)
        print(data)

        print('RTT= %f s' % (time.time() - t1))

        if len(data) == 0:
            print('connection closed.')
            break
        elif data == b'ack':
            pass
        else:
            print('unexpected response.')
            break

        time.sleep(0.2)
        count += 1

except socket.error as e:
    print(e)

finally:
    print('closing socket')
    sock.close()
