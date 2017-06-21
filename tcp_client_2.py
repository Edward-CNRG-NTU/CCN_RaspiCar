import socket
import struct
import time
import sys
import numpy as np

command = 'fwd:0.03'

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.11.3', 23233)

try:
    print('connecting to %s port %s' % server_address)
    sock.connect(server_address)

    count = 0

    while count < 1:

        t1 = time.time()

        print('"%s": ' % command, end='')

        sock.sendall(command.encode('utf-8'))
        data = sock.recv(128).decode('utf-8')

        print('"%s", RTT= %f s' % (data, (time.time() - t1)))

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
