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

TARGET_IP = None

g_frame = None
g_quit = False
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

        while not stopper.wait(timeout=0.05):
            global g_frame
            (_, g_frame) = camera.read()
            cv2.imshow("raw", g_frame)

            if TARGET_IP:
                time_stamp = int(time.clock()*1000)
                result, imgencode = cv2.imencode('.jpg', g_frame, encode_param)
                data = np.array(imgencode).tobytes()
                try:
                    header = struct.pack('QHHHHII', time_stamp, len(data), g_proximity[0],  g_proximity[1], g_proximity[2],
                                         g_wheel_count[0], g_wheel_count[1])
                except :
                    print('out of range? %d' % time_stamp)
                sock.sendto(header + data, TARGET_IP)

                print('\t\tsending data%06d(%d bytes) to %s. ' % (time_stamp, len(data), TARGET_IP[0]))

        camera.release()
        print('camera_routine stopped!')

    t = threading.Thread(target=camera_routine)
    t.setDaemon(True)
    t.start()

    return stopper


def launch_waitkey_routine():
    stopper = threading.Event()

    def waitkey_routine():
        while not stopper.wait(timeout=0.1):

            key = cv2.waitKey(1) & 0xFF

            if key is 0xFF:
                pass
            else:
                print('key in:', chr(key))
                if key == ord('q'):
                    global g_quit
                    g_quit = True
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
        while not stopper.wait(timeout=0.1):
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
    while not g_quit:
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
    while not g_quit:
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
    while(not g_quit) and np.sum(diff_of_both) / 2 < distance / 0.0113:
        if not g_obstacle_detected:
            motor.forward(speed=1.0, t=0.3)
            # time.sleep(0.1)
            diff_of_both = g_wheel_count - wheel_last
            diff_between = diff_of_both[0] - diff_of_both[1]
            print np.sum(diff_of_both), diff_between
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

    print raw_data
    return np.nanmedian(raw_data, axis=0)


try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print('listening to port %s' % 23232)
    sock.bind(('', 23232))
    sock.settimeout(3)
except socket.error as e:
    print(e)
    exit(1)

external_sensor_routine_stopper = launch_external_sensor_routine()
camera_routine_stopper = launch_camera_routine()
launch_waitkey_routine_stopper = launch_waitkey_routine()
obstacle_detection_routine_stopper = None

try:

    while not g_quit:
        try:
            (packet, addr) = sock.recvfrom(1024)

            if packet == b'HELLO':
                TARGET_IP = addr

        except socket.timeout:
            pass

finally:
    sock.close()
    if obstacle_detection_routine_stopper is not None:
        obstacle_detection_routine_stopper.set()
    camera_routine_stopper.set()
    launch_waitkey_routine_stopper.set()
    external_sensor_routine_stopper.set()
    motor.cleanup()
    cv2.destroyAllWindows()


