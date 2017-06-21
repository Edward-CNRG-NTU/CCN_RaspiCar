import numpy as np
import nengo
import socket
import threading
import struct
import time
import cv2
import operator

import nengo.spa as spa

TARGET_IP = '192.168.11.3'

g_vision_msg = 0
g_visual_perception = ''
g_motion_out = ''
g_distance = np.zeros([3])


def visual_perception(g_frame):
    x = y = w = h = frame = None
    light = light_position = distance = ''
    L = 160
    R = 250
    HSV_YELLOW = {'lower': (22, 90, 80), 'upper': (33, 255, 255)}
    HSV_RED = {'lower': (0, 80, 170), 'upper': (15, 255, 255)}
    HSV_RED2 = {'lower': (170, 80, 170), 'upper': (179, 255, 255)}
    HSV_GREEN = {'lower': (45, 80, 170), 'upper': (80, 255, 255)}

    if g_frame is not None:
        frame = cv2.GaussianBlur(g_frame, (3, 3), 0)
        frameU = frame[:200, :]
        # frame = g_frame.copy()
        hsv_frame = cv2.cvtColor(frameU, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_frame, HSV_RED['lower'], HSV_RED['upper']) + cv2.inRange(hsv_frame,
                                                                                            HSV_RED2['lower'],
                                                                                            HSV_RED2['upper'])
        mask_green = cv2.inRange(hsv_frame, HSV_GREEN['lower'], HSV_GREEN['upper'])
        (_, contours_red, _) = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        (_, contours_green, _) = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_dict = [{'contour': i, 'area': cv2.contourArea(i), 'color': 'RED'} for i in contours_red]
        contours_dict += [{'contour': i, 'area': cv2.contourArea(i), 'color': 'GREEN'} for i in contours_green]

        if len(contours_dict):
            c_final = max(contours_dict, key=operator.itemgetter('area'))

            if c_final['area'] > 10:
                x, y, w, h = cv2.boundingRect(c_final['contour'])
                cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 255), 2)
                light = c_final['color']

                if 7*320/h > 30:
                    distance = '+FAR'
                else:
                    distance = '+NEAR'

                if (x+w/2) < L:
                    light_position = '+LEFT'
                elif (x+w/2) > R:
                    light_position = '+RIGHT'

    perception = light + light_position + distance

    return x, y, w, h, perception, frame


def launch_udp_listener_routine():
    stopper = threading.Event()

    def udp_listener_routine():

        HEADER_FORMAT = 'QHHHHII'
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

        last_time_stamp = 0

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            print('listening to port %s' % 23232)
            sock.bind(('', 23232))

            sock.sendto(b'HELLO', (TARGET_IP, 23232))

            while not stopper.is_set():

                packet = None
                addr = None

                try:
                    (packet, addr) = sock.recvfrom(32 * 1024)
                except socket.error as e:
                    print(e)

                if packet and addr == (TARGET_IP, 23232):
                    header = struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])

                    if header[0] > last_time_stamp:
                        last_time_stamp = header[0]

                        if header[1] == len(packet) - HEADER_SIZE:
                            # print(header)
                            global g_distance
                            g_distance = np.clip(header[2:5], 0, 20)
                            np_data = np.fromstring(packet[HEADER_SIZE:], dtype='uint8')
                            # decoded_img = cv2.imdecode(np_data, 1)
                            # (x, y, w, h, perception, frame) = visual_perception(decoded_img)

                            # global g_visual_perception
                            # g_visual_perception = perception

                            # cv2.imshow('view', frame)
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


def launch_tcp_client_routine():
    stopper = threading.Event()

    def tcp_client_routine():

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            print('connecting to %s port %s' % (TARGET_IP, 23233))
            sock.connect((TARGET_IP, 23233))

            while not stopper.is_set():
                
                motion_out = g_motion_out
                
                command = ''
                
                if motion_out == 'FORWARD':
                    command = 'fwd:0.03'
                elif motion_out == 'RIGHT':
                    command = 'right:5'
                elif motion_out == 'LEFT':
                    command = 'left:5'
                elif motion_out == 'STOP':
                    command = ''
                else:
                    pass
                
                if len(command):
                    t1 = time.time()

                    print(command,)
    
                    sock.sendall(command.encode('utf-8'))
                    data = sock.recv(128).decode('utf-8')
                    print(data,)
    
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
                    
                    time.sleep(0.3)
                else:
                    time.sleep(0.3)

        finally:
            print('closing socket')
            sock.close()

        print('tcp_client_routine stopped!')

    t = threading.Thread(target=tcp_client_routine)
    t.setDaemon(True)
    t.start()

    return stopper


tcp_client_routine_stopper = launch_tcp_client_routine()
udp_listener_routine_stopper = launch_udp_listener_routine()

model = spa.SPA()
D = 48

with model:
    model.vision = spa.State(D)

    model.motion = spa.State(D)

    actions = spa.Actions(
        'dot(vision, RIGHT) --> motion=RIGHT',
        'dot(vision, LEFT) --> motion=LEFT',
        'dot(vision, NEAR)*0.6 --> motion=STOP',
        'dot(vision, FAR)*0.6 --> motion=FORWARD',
        'dot(vision, GREEN)*0.8 --> motion=FORWARD',
        'dot(vision, RED)*0.4 --> motion=STOP',
        '0.3 --> motion=0',
    )

    model.bg = spa.BasalGanglia(actions)
    model.thalamus = spa.Thalamus(model.bg)


    def input_func(t):  # get vision
        if g_visual_perception:
            return g_visual_perception
        else:
            return '0'


    model.input = spa.Input(vision=input_func)
    out_vocab = model.motion.outputs['default'][1]  # get vocab

    print(model.motion.outputs['default'][1])

    def output_func(t, x):
        similarity = spa.similarity(x, out_vocab.vectors)
        # print(out_vocab.keys[np.argmax(similarity)])

        if np.any(similarity > 0.7):
            global g_motion_out
            g_motion_out = out_vocab.keys[np.argmax(similarity)]
            # print(g_motion_out)


    model.output = nengo.Node(size_in=D, size_out=0, output=output_func)  # get output motion

    nengo.Connection(model.motion.output, model.output)

    # udp_listener_routine_stopper.set()