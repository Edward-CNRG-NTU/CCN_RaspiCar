import socket
import struct
import time
import sys
import cv2
import numpy as np


TARGET_IP = ('192.168.11.3', 23232)

HEADER_FORMAT = 'qHHHHII'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

count = 0
last_time_stamp = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


def find_light(g_frame):
    x = y = w = h = frame = light = None
    HSV_YELLOW = {'lower': (22, 90, 80), 'upper': (33, 255, 255)}
    HSV_RED = {'lower': (0, 60, 80), 'upper': (15, 255, 255)}
    HSV_RED2 = {'lower': (170, 60, 80), 'upper': (179, 255, 255)}
    HSV_GREEN = {'lower': (45, 30, 170), 'upper': (80, 255, 255)}

    if g_frame is not None:
        frame = cv2.GaussianBlur(g_frame, (3, 3), 0)
        frameU = frame[:120, :]
        # frame = g_frame.copy()
        hsv_frame = cv2.cvtColor(frameU, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_frame, HSV_RED['lower'], HSV_RED['upper']) + cv2.inRange(hsv_frame,
                                                                                            HSV_RED2['lower'],
                                                                                            HSV_RED2['upper'])
        mask_green = cv2.inRange(hsv_frame, HSV_GREEN['lower'], HSV_GREEN['upper'])
        (_, contours_red, _) = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        (_, contours_green, _) = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_red = [i for i in contours_red if 10 < cv2.contourArea(i)]
        contours_green = [i for i in contours_green if 10 < cv2.contourArea(i)]

        contours_area_red = [cv2.contourArea(i) for i in contours_red]
        contours_area_green = [cv2.contourArea(i) for i in contours_green]

        c_final = None

        if len(contours_red) and not len(contours_green):
            c_final = max(contours_red, key=cv2.contourArea)
            light = 'RED'

        elif not len(contours_red) and len(contours_green):
            c_final = max(contours_green, key=cv2.contourArea)
            light = 'GREEN'

        elif len(contours_red) and len(contours_green):
            c_red = max(contours_red, key=cv2.contourArea)
            c_green = max(contours_green, key=cv2.contourArea)

            if cv2.contourArea(c_red) > cv2.contourArea(c_green):
                c_final = c_red
                light = 'RED'
            else:
                c_final = c_green
                light = 'GREEN'

                # contours = contours_red + contours_green
                # c_final = max(contours, key=cv2.contourArea)
                # c_final_idx = contours.index(c_final)
        if c_final is not None:
            x, y, w, h = cv2.boundingRect(c_final)
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 255), 2)

    return x, y, w, h, light, frame

try:
    listen_address = ('', 23232)
    print('binding to %s port %s' % listen_address)
    sock.bind(listen_address)

    sock.sendto(b'HELLO', TARGET_IP)

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

                    (x, y, w, h, light, decoded_img) = find_light(decoded_img)

                    print(x, y, w, h, light)

                    # print('_'*int((time.time()-t1)*100))
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

