import cv2
import sys
import time
import socket
import serial
import struct
import threading
import numpy as np
import motor_control

Frame_Width  = 320
Frame_Height = 240
View_Center = Frame_Width / 2

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]

motor = motor_control.MotorControl(dc_level=70, t=0.3)

g_frame = None
g_quit = threading.Event()
g_state = 5
g_wheel_last = None
g_proximity = np.array([0, 0, 0])
g_wheel_count = np.array([0, 0])
g_external_sensor_condition = threading.Condition()
g_proximity_control = 15
g_obstacle_detected = False


def launch_camera_routine():
    stopper = threading.Event()

    def camera_routine():
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, Frame_Width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Frame_Height)

        next_entry_t = 0
        while not stopper.wait(timeout=(next_entry_t - time.time())):
            next_entry_t = time.time() + 0.1

            (_, frame) = camera.read()
            cv2.imshow("raw", frame)
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            global g_frame
            g_frame = np.array(imgencode).tobytes()

        camera.release()
        print('camera_routine stopped!')

    t = threading.Thread(target=camera_routine)
    t.setDaemon(True)
    t.start()

    return stopper


def launch_waitkey_routine():
    stopper = threading.Event()

    def waitkey_routine():
        next_entry_t = 0
        while not stopper.wait(timeout=(next_entry_t - time.time())):
            next_entry_t = time.time() + 0.1

            key = cv2.waitKey(1) & 0xFF

            if key is 0xFF:
                pass
            else:
                print('key in:', chr(key))
                if key == ord('q'):
                    g_quit.set()
                elif key == ord('a'):
                    motor.turn_left()
                elif key == ord('d'):
                    motor.turn_right()
                elif key == ord('w'):
                    motor.forward()
                elif key == ord('s'):
                    motor.backward()
                elif ord('0') <= key <= ord('9'):
                    global g_state
                    g_state = key - ord('0')
                else:
                    pass

                while key is not 0xFF:
                    key = cv2.waitKey(1) & 0xFF

        print('waitkey_routine stopped!')

    t = threading.Thread(target=waitkey_routine)
    t.setDaemon(True)
    t.start()

    return stopper


def launch_external_sensor_routine():
    stopper = threading.Event()

    def external_sensor_routine():
        serial_port = serial.Serial('/dev/ttyACM0', 115200)
        serial_port.write('R')
        proximity_control = 0

        while not stopper.is_set():
            line = serial_port.readline()
            # print 'received:', line
            if line.startswith('#'):
                try:
                    data = np.array(line[1:-1].split(','), dtype='|S4').astype(np.float)
                except ValueError:
                    continue

                global g_external_sensor_condition
                g_external_sensor_condition.acquire()

                global g_proximity
                g_proximity = data[0:3]

                global g_wheel_count
                g_wheel_count = data[3:5]

                g_external_sensor_condition.notifyAll()
                g_external_sensor_condition.release()

            if g_proximity_control != proximity_control:
                print('send g_proximity_control')
                serial_port.write(chr(g_proximity_control))
                proximity_control = g_proximity_control

        serial_port.close()
        print('external_sensor_routine stopped!')

    t = threading.Thread(target=external_sensor_routine)
    t.setDaemon(True)
    t.start()

    return stopper


def launch_obstacle_detection_routine():
    stopper = threading.Event()

    def obstacle_detection_routine():
        global g_proximity_control
        next_entry_t = 0
        while not stopper.wait(timeout=(next_entry_t - time.time())):
            next_entry_t = time.time() + 0.3
            global g_obstacle_detected
            if np.any(filtered_proximity(3) < [5, 25, 5]):
                g_obstacle_detected = True
                print('obstacle detected:', g_proximity)
            else:
                g_obstacle_detected = False

                if g_proximity_control == 10:
                    g_proximity_control = 15
                elif g_proximity_control == 15:
                    g_proximity_control = 20
                elif g_proximity_control == 20:
                    g_proximity_control = 10
                else:
                    g_proximity_control = 15

        g_proximity_control = 15
        print('obstacle_detection_routine stopped!')

    t = threading.Thread(target=obstacle_detection_routine)
    t.setDaemon(True)
    t.start()

    return stopper


def turn_right_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not g_quit.is_set():
        if not g_obstacle_detected:
            time.sleep(0.05)
            if g_wheel_count[0] - wheel_last[0] < count:
                motor.turn_right(speed=0.8, t=0.2)
            elif g_wheel_count[0] - wheel_last[0] > count:
                motor.turn_left(speed=0.8, t=0.15)
                break
            else:
                break
        else:
            time.sleep(0.1)


def turn_left_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not g_quit.is_set():
        if not g_obstacle_detected:
            time.sleep(0.05)
            if g_wheel_count[1] - wheel_last[1] < count:
                motor.turn_left(speed=0.8, t=0.2)
            elif g_wheel_count[1] - wheel_last[1] > count:
                motor.turn_right(speed=0.8, t=0.15)
                break
            else:
                break
        else:
            time.sleep(0.1)


def forward_controlled(distance):
    wheel_last = g_wheel_count
    diff_of_both = g_wheel_count - wheel_last
    while(not g_quit.is_set()) and np.sum(diff_of_both) / 2 < distance / 0.0113:
        if not g_obstacle_detected:
            motor.forward(speed=1.0, t=0.3)
            # time.sleep(0.1)
            diff_of_both = g_wheel_count - wheel_last
            diff_between = diff_of_both[0] - diff_of_both[1]
            print(np.sum(diff_of_both), diff_between)
            if diff_between > 0:
                motor.turn_left(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)
            elif diff_between < 0:
                motor.turn_right(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)
        else:
            time.sleep(0.1)


def cv2_find_enclosing_circle(hsv_color_range, hsv_color_range_2=None):
    center = None
    radius = None
    frame = None

    if g_frame is not None and hsv_color_range is not None:
        frame = cv2.GaussianBlur(g_frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, hsv_color_range['lower'], hsv_color_range['upper'])
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        if hsv_color_range_2 is not None:
            mask += cv2.inRange(hsv_frame, hsv_color_range_2['lower'], hsv_color_range_2['upper'])

        (_, contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the max length of contours
            c = max(contours, key=cv2.contourArea)
            # Find the x, y, radius of given contours
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            try:
                # Find the moments
                M = cv2.moments(c)
                # mass center
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # process every frame
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            except ZeroDivisionError:
                pass

    return center, radius, frame


def cv2_find_enclosing_rect(hsv_color_range, hsv_color_range_2=None):
    x = y = w = h = frame = None

    if g_frame is not None and hsv_color_range is not None:
        frame = cv2.GaussianBlur(g_frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, hsv_color_range['lower'], hsv_color_range['upper'])
        if hsv_color_range_2 is not None:
            mask += cv2.inRange(hsv_frame, hsv_color_range_2['lower'], hsv_color_range_2['upper'])

        (_, contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the max length of contours
            c = max(contours, key=cv2.contourArea)
            # Find the x, y, radius of given contours
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 255), 2)

    return x, y, w, h, frame


def filtered_proximity(median=5):
    raw_data = np.zeros([median, g_proximity.shape[0]])
    for i in range(median):
        g_external_sensor_condition.acquire()
        g_external_sensor_condition.wait()
        raw_data[i] = g_proximity
        g_external_sensor_condition.release()

    print(raw_data)
    return np.nanmedian(raw_data, axis=0)


def launch_udp_service_routine():
    stopper = threading.Event()

    def udp_service():
        target_ip = None
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1)

        try:
            server_address = ('', 23232)
            print('[udp] binding to %s port %s' % server_address)
            sock.bind(server_address)

            global g_frame
            while not stopper.is_set():

                if target_ip is None:
                    try:
                        (packet, ip) = sock.recvfrom(128)
                        if packet == b'HELLO':
                            target_ip = ip
                    except socket.timeout:
                        print('\t[udp] wait for HELLO message...')
                        pass
                elif g_frame is not None:
                    frame = g_frame
                    g_frame = None
                    time_stamp = int(time.clock() * 1000)
                    header = struct.pack('QHHHHII', time_stamp, len(frame), g_proximity[0], g_proximity[1],
                                         g_proximity[2], g_wheel_count[0], g_wheel_count[1])
                    sock.sendto((header + frame), target_ip)
                    print('\t\t[udp] sending data%06d(%d bytes) to %s. ' % (time_stamp, len(frame), target_ip[0]))
                else:
                    time.sleep(0.01)

        except socket.error as e:
            print(e)

        finally:
            sock.close()
            g_quit.set()
            print('udp_service stopped!')

    t = threading.Thread(target=udp_service)
    t.setDaemon(True)
    t.start()

    return stopper


def launch_tcp_service_routine():
    stopper = threading.Event()

    def tcp_service_routine():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(10)

        try:
            server_address = ('', 23233)
            print('[tcp] binding to %s port %s' % server_address)
            sock.bind(server_address)
            sock.listen(0)

            while not stopper.is_set():

                try:
                    print('\t[tcp] waiting for new connection...')
                    connection, client_address = sock.accept()
                    print('\t[tcp] accepted connection from %s %d.' % client_address)

                    while not stopper.is_set():
                        print('\t\t[tcp] waiting for command...')
                        data = str(connection.recv(128))
                        print('\t\t[tcp] received "%s".' % data)

                        if len(data) == 0:
                            print('\t[tcp] connection closed.')
                            break
                        elif data.startswith('test'):
                            connection.sendall(b'ack')
                            print('\t\t[tcp] sending ack to the client.')
                        elif data.startswith('fwd'):
                            try:
                                value = float(data.split(':')[1])
                            except KeyError:
                                value = 0.1
                            except ValueError:
                                value = 0
                            forward_controlled(value)
                            connection.sendall(b'ack')
                            print('\t\t[tcp] sending ack to the client.')
                        elif data.startswith('right'):
                            try:
                                value = float(data.split(':')[1])
                            except KeyError:
                                value = 10
                            except ValueError:
                                value = 0
                            turn_right_controlled(value)
                            connection.sendall(b'ack')
                            print('\t\t[tcp] sending ack to the client.')
                        elif data.startswith('left'):
                            try:
                                value = float(data.split(':')[1])
                            except KeyError:
                                value = 10
                            except ValueError:
                                value = 0
                            turn_left_controlled(value)
                            connection.sendall(b'ack')
                            print('\t\t[tcp] sending ack to the client.')
                        elif data.startswith('obstacle'):
                            global obstacle_detection_routine_stopper
                            try:
                                value = float(data.split(':')[1])
                            except KeyError:
                                if obstacle_detection_routine_stopper is None:
                                    value = 1
                                else:
                                    value = 0
                            except ValueError:
                                value = 0

                            if value > 0.0 and obstacle_detection_routine_stopper is None:
                                obstacle_detection_routine_stopper = launch_obstacle_detection_routine()
                            elif value == 0.0 and obstacle_detection_routine_stopper is not None:
                                obstacle_detection_routine_stopper.set()
                                obstacle_detection_routine_stopper = None

                            connection.sendall(b'ack')
                            print('\t\t[tcp] sending ack to the client.')
                        else:
                            connection.sendall(b'error')
                            print('\t\tunexpected command.')

                except socket.timeout:
                    print('\t[tcp] sock.accept() time out.')

        except socket.error as e:
            print(e)

        finally:
            sock.close()
            g_quit.set()
            print('tcp_service_routine stopped!')

    t = threading.Thread(target=tcp_service_routine)
    t.setDaemon(True)
    t.start()

    return stopper


external_sensor_routine_stopper = launch_external_sensor_routine()
camera_routine_stopper = launch_camera_routine()
launch_waitkey_routine_stopper = launch_waitkey_routine()
obstacle_detection_routine_stopper = None

time.sleep(0.5)

udp_service_routine_stopper = launch_udp_service_routine()
tcp_service_routine_stopper = launch_tcp_service_routine()

try:

    while not g_quit.is_set():
        time.sleep(0.1)

finally:

    if obstacle_detection_routine_stopper is not None:
        obstacle_detection_routine_stopper.set()

    tcp_service_routine_stopper.set()
    udp_service_routine_stopper.set()
    launch_waitkey_routine_stopper.set()
    camera_routine_stopper.set()
    external_sensor_routine_stopper.set()
    motor.cleanup()
    cv2.destroyAllWindows()
    time.sleep(1.0)
    print('exit main.')
