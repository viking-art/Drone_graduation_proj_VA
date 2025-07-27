from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import numpy as np
import math
from simple_pid import PID
import socket
import json
import threading

import cv2
import numpy as np
from ultralytics import YOLO

def connect_drone():
    print("🔌 Kết nối drone...")
    return connect('192.168.1.9:14551', wait_ready=True)
    # return connect('udp:0.0.0.0:14551', wait_ready=True)
    # return connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
    
def arm_and_takeoff(vehicle, target_alt):
    print("🚁 Chuyển sang chế độ GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == 'GUIDED':
        time.sleep(1)

    print("Arm drone...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Chờ arm...")
        time.sleep(1)

    print(f"🚀 Cất cánh đến {target_alt}m...")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"🔎 Độ cao hiện tại: {alt:.2f} m")
        if alt >= target_alt * 0.97:
            print("Đã đạt độ cao mục tiêu.")
            break
        time.sleep(1)

def bay_xyz(vehicle, x, y, z, vx, vy, vz):
    
    type_mask = 0b000111000000
    frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

    print(f"Bay đến x={x}, y={y}, z={z}, vx={vx}, vy={vy}, vz={vz}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        frame,
        type_mask,
        x, y, z,
        vx, vy, vz,  # velocity
        0, 0, 0,  # accel
        0, 0      # yaw
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def bay_only_vel(vehicle, vx, vy, vz):
    
    type_mask = 0b000111000111
    frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

    print(f"vx={vx}, vy={vy}, vz={vz}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        frame,
        type_mask,
        0, 0, 0,
        vx, vy, vz,  # velocity
        0, 0, 0,  # accel
        0, 0      # yaw
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def bay_only_pos(vehicle, x, y, z):
    
    type_mask = 0b000111111000
    frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

    print(f"vx={x}, vy={y}, vz={z}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        frame,
        type_mask,
        x, y, z,
        0, 0, 0,  # velocity
        0, 0, 0,  # accel
        0, 0      # yaw
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def path_planning(x0, xf, v0, vf, t):
    T = np.array([[1,    0,   0,    0],
				  [0,    1,   0,    0],
				  [x0,   t,   t**2, t**3],
				  [0,    1,   2*t,  3*(t**2)]])
				  
    b = np.array([[x0],
				  [v0],
				  [xf],
				  [vf]])
    coef_a = np.linalg.solve(T,b)
    return coef_a
    
def pos_xyz(coef_a, t):
    a0, a1, a2, a3 = coef_a
    pos = a0 + a1*t + a2*t**2 + a3*t**3
    vel = a1 + 2*a2*t + 3*a3*t**2
    return pos, vel

def ha_canh(vehicle):
    print("Đổi mode LAND...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Độ cao hiện tại: {alt:.2f} m")
        time.sleep(1)
    print("Drone đã hạ cánh xong.")

def rtl(vehicle):
    print("Đổi mode RTL...")
    vehicle.mode = VehicleMode("RTL")
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Độ cao hiện tại: {alt:.2f} m")
        time.sleep(1)
    print("Drone đã hạ cánh xong.")

def distance():
    global flag_xy
    global dx_1, dy_1
    dx = vehicle.location.local_frame.north
    dy = vehicle.location.local_frame.east
    if not flag_xy: 
        dx_1  = dx
        dy_1  = dy
        flag_xy  = True
    dx_curr  = dx - dx_1
    dy_curr  = dy - dy_1
    dist_now = math.sqrt(dx_curr**2 + dy_curr**2)
    return dist_now
     
def servo(channel_servo, PWM):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel_servo,
        PWM,
        0,
        0,
        0, 0, 0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

## Don't use ##
# def read_servo_pwm():
#     vehicle.mav.request_data_stream_send(
#     vehicle.target_system,
#     vehicle.target_component,
#     mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
#     1,  # tần số 1Hz
#     1   # bật
#     )
#     msg = vehicle.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
#     pwm_servo = msg.servo10_raw

def load_model():
    try:
        model = YOLO('my_model_new.pt', task='detect')
        labels = model.names
        return model, labels
    except Exception as e:
        print(f"Error loading model: {e}")
        return None, None

def imgprocess(frame,model,labels):
    
    t_start = time.perf_counter()
    center_x, center_y = None, None
    avg_frame_rate = 0
    frame_rate_buffer = []
    fps_avg_len = 200
    frame_center_x = int(frame.shape[1] / 2)
    frame_center_y = int(frame.shape[0] / 2)
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)

    results = model(frame, verbose=False)
    detections = results[0].boxes

    for i in range(len(detections)):
        xyxy = detections[i].xyxy.cpu().numpy().squeeze().astype(int)
        xmin, ymin, xmax, ymax = xyxy
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]
        conf = detections[i].conf.item()

        if conf > 0.5:
            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            
            # tinh tam, ve tam cua box va duong den tam camera
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.line(frame, (center_x, center_y), (frame_center_x, frame_center_y), (0, 255, 0), 2)
            distance_img = int(np.sqrt((center_x - frame_center_x)**2 + (center_y - frame_center_y)**2))
            cv2.putText(frame, f'Dist: {distance_img}px', (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            
            # viet ten cua box len anh
            label = f'{classname}: {int(conf*100)}%'
            label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_ymin = max(ymin, label_size[1] + 10)
            cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10), (xmin + label_size[0], label_ymin + base_line - 10), color, cv2.FILLED)
            cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    t_stop = time.perf_counter()
    fps = 1 / (t_stop - t_start)
    frame_rate_buffer.append(fps)
    if len(frame_rate_buffer) > fps_avg_len:
        frame_rate_buffer.pop(0)
    avg_frame_rate = np.mean(frame_rate_buffer)

    cv2.imshow('YOLO detection results', frame)
    cv2.putText(frame, f'FPS: {avg_frame_rate:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return None
    return frame_center_x, frame_center_y, center_x, center_y

def lis_esp():
    HOST = '0.0.0.0'
    PORT = 5000
    Stop_servo = False   
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Listening on {HOST}:{PORT}...")
        while True:
            conn, addr = s.accept()
            bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
            with conn:
                print(f"Connected by {addr}")
                data = conn.recv(1024)
                if data:
                    print("Received:", data.decode())
                    sens = data.decode()
                    conn.sendall(b"NO")
                    if sens == "350":
                        Stop_servo = True
                        conn.sendall(b"OKE")
                        return Stop_servo
                else:
                    print("No data received or client disconnected.")
                    conn.close()
            time.sleep(0.1)

def lis_host(vitritram1_x, vitritram1_y, vitritram2_x, vitritram2_y):
    
    HOST = "0.0.0.0"
    PORT = 5001

    # Vị trí các trạm trong LOCAL_NED
    station_coords = {
        1: [vitritram1_x, vitritram1_y],
        2: [vitritram2_x, vitritram2_y]
    }

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print("Listening...")

        Start = False
        ids = []
        global id_flag, index
        while True:
            conn, addr = s.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                print(f"Nhận được: {data}")

                if data == "start":
                    Start = True
                    print("Nhận tín hiệu START")
                    conn.sendall("OK".encode())
                    if not id_flag:
                        index   = ids
                        id_flag = True
                    return Start, index  # Trả ra luôn cả ID đã nhận trước đó

                else:
                    try:
                        ids = list(map(int, data.split(",")))
                        print(f"Nhận ID: {ids}")
                        result = {str(i): station_coords[i] for i in ids if i in station_coords}
                        conn.sendall(json.dumps(result).encode())
                        print("Đã gửi vị trí cho GUI")
                    except Exception as e:
                        print(f"Lỗi: {e}")
                        conn.sendall("{}".encode())
                time.sleep(0.1)

def send_position_to_host(host_ip, host_port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            loc = vehicle.location.global_relative_frame
            lat = loc.lat
            lon = loc.lon
            alt = loc.alt
            msg = f"{lat},{lon},{alt}"
            sock.sendto(msg.encode(), (host_ip, host_port))
            time.sleep(0.5)

def set_pid_profile(profile):
    vehicle.parameters['ATC_RAT_PIT_P'] = profile['PP']
    vehicle.parameters['ATC_RAT_PIT_I'] = profile['IP']
    vehicle.parameters['ATC_RAT_PIT_D'] = profile['DP']
    
    vehicle.parameters['ATC_RAT_RLL_P'] = profile['PR']
    vehicle.parameters['ATC_RAT_RLL_I'] = profile['IR']
    vehicle.parameters['ATC_RAT_RLL_D'] = profile['DR']
    print("Change PID done! Wait 3 seconds before flying")

def wait_time_img(a):
    time_ini = time.time()
    while time.time() - time_ini < a:
        pass

def wait_time_servo(b):
    time_ini = time.time()
    bay_only_pos(vehicle, 0, 0, 0)
    while time.time() - time_ini < b:
        # bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
        pass

def luu_do_cao(altitude):
    if vehicle.location.global_relative_frame.alt*1.01 >= altitude:
        alt = altitude
        return alt

def PIDcontrol(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_z, ki_z, kd_z, setpoint_rll, setpoint_ptc, setpoint_z):
    pidroll  = PID(kp_roll, ki_roll, kd_roll, setpoint_rll)
    pidroll.output_limits=(-0.5,0.5)              #m/s
    pidpitch = PID(kp_pitch, ki_pitch, kd_pitch, setpoint_ptc)
    pidpitch.output_limits=(-0.5,0.5)             #m/s
    pidz  = PID(kp_z, ki_z, kd_z, setpoint_z)
    pidz.output_limits=(-1,1)                     #m/s
    return pidroll, pidpitch, pidz

def controller(center_x, center_y, altnow, pidroll, pidpitch, pidz):
    # err_x = center_x - frame_center_x
    # err_y = center_y - frame_center_y
    # print(err_x)
    # print(err_y)
    control_roll  = abs(pidroll(center_x))         
    control_pitch = abs(pidpitch(center_y))
    # control_roll  = pidroll(center_x)
    # control_pitch = pidpitch(center_y)
    control_z  = pidz(altnow)
    return control_roll, control_pitch, control_z

def PID_onlyZ(kpz, kiz, kdz, setpointz):
    pidzz  = PID(kpz, kiz, kdz, setpointz)
    pidzz.output_limits=(-1,1)
    return pidzz

def controlforZ(pidzz, altreal):
    control_zz = pidzz(altreal)
    return control_zz
# ===========================
# CHƯƠNG TRÌNH CHÍNH
# ===========================
var_ref_flag  = False
index         = []
id_flag       = False
t_for_xy_1    = 10
t_for_xy_2    = 10
pid_1 = {'PR': 0.15, 'IR': 0.15, 'DR': 0.01, 'PP' : 0.15, 'IP' : 0.15, 'DP' : 0.01}
pid_2 = {'PR': 0.135, 'IR': 0.135, 'DR': 0.0036, 'PP' : 0.135, 'IP' : 0.135, 'DP' : 0.0036}
model, labels = load_model()
print("load model")
if not model or not labels:
    print("can not load model")
cap = cv2.VideoCapture(0)
print("chuan bi camera")
while not cap.isOpened():
    print("waiting for open cam")
    
vehicle = connect_drone()
# set_pid_profile(pid_2)
# time.sleep(3)
while True:
    TT               = 10
    Start            = False
    flag_xy          = False
    flag_alt         = False
    flag_pid         = False
    dungim           = False
    Target_locked    = False
    Run_servo        = False
    khong_thay_toa_do_muc_tieu = False
    global x, y, vx, vy, dist
    # frame_center_x, frame_center_y = None, None
    
    vitritram1_x = 2
    vitritram1_y = 0
    vitritram2_x = 3
    vitritram2_y = 4
    Start, thu_tu_tram = lis_host(vitritram1_x, vitritram1_y, vitritram2_x, vitritram2_y)
    print(thu_tu_tram)

    if thu_tu_tram[0] == 1:
        tram_x = abs(vitritram1_x/t_for_xy_1)     # vel of s1
        tram_y = abs(vitritram1_y/t_for_xy_1)     # vel of s1
        vel_x  = vitritram1_x/t_for_xy_1
        vel_y  = vitritram1_y/t_for_xy_1
        vitritram_x = vitritram1_x
        vitritram_y = vitritram1_y
        t_percent   = t_for_xy_1 * 0.9
    elif thu_tu_tram[0] == 2:
        tram_x = abs(vitritram2_x/t_for_xy_2)     # vel for s2
        tram_y = abs(vitritram2_y/t_for_xy_2)     # vel for s2
        vel_x  = vitritram2_x/t_for_xy_2
        vel_y  = vitritram2_y/t_for_xy_2
        vitritram_x = vitritram2_x
        vitritram_y = vitritram2_y
        t_percent   = t_for_xy_2 * 0.9

    dist_set = math.sqrt(vitritram_x**2 + vitritram_y**2)
    
    if Start:
        # vehicle = connect_drone()
        arm_and_takeoff(vehicle, 3)
        if not flag_alt:
            alt_set = luu_do_cao(3)
            flag_alt = True
        # vehicle.airspeed = 5 #m/s
        # vehicle.groundspeed = 1 #m/s
        threading.Thread(target=send_position_to_host, args=("192.168.1.90", 9999), daemon=True).start()
        kp_roll=0.5
        ki_roll=0
        kd_roll=0

        kp_pitch=0.5
        ki_pitch=0
        kd_pitch=0

        kp_z    = 3.3
        ki_z    = 0.001
        kd_z    = 0.0001
        pidz1 = PID_onlyZ(kp_z, ki_z, kd_z, alt_set)
        while True:
            dist = distance()
            # print(dist)
            vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
            bay_xyz(vehicle, 0, 0, 0, vel_x, vel_y, -vel_z)
            
            if dist >= dist_set*0.9:
                print("bay tiep", dist)
                coef_ax = path_planning(0,0,tram_x,0,t_percent)   # x0, xf, vx0, vxf, t
                coef_ay = path_planning(0,0,tram_y,0,t_percent)   # y0, yf, vy0, vyf, t
                for ttime in np.arange(0,TT,0.1):
                    x, vx = pos_xyz(coef_ax, ttime)
                    y, vy = pos_xyz(coef_ay, ttime)
                    vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
                    if vx < 5e-2:
                        break
                    bay_xyz(vehicle, 0, 0, 0, vx, vy, -vel_z)
                    time.sleep(0.01)
                if not dungim:
                    # Sau khi bay den diem
                    bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
                    time.sleep(0.1)
                    dungim = True
                if dungim:
                    break
            time.sleep(0.01)

        if dungim:
            print("mo camera")
            counting = 0
            frame = None
            frame_center_x, frame_center_y = None, None
            # cap = cv2.VideoCapture(0)
            # while not cap.isOpened():
            #     print("waiting for open cam")
            cap.set(3, 640)
            cap.set(4, 640)
            # model, labels = load_model()
            # if not model or not labels:
            #     print("can not load model")
            # time.sleep(0.5)
            
            while True:
                bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
                (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]
                
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("No frame")

                results = imgprocess(frame,model,labels)
                frame_center_x, frame_center_y, box_center_x, box_center_y = results
                if not flag_pid:
                    pidroll, pidpitch, pidz2 = PIDcontrol(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, ki_z, kp_z, kd_z, setpoint_rll=frame_center_x, setpoint_ptc=frame_center_y, setpoint_z=alt_set)
                    flag_pid = True
                    
                if results is None:
                    print("Press q and exit")
                    khong_thay_toa_do_muc_tieu = True
                    break
                
                if None in (frame_center_x, frame_center_y, box_center_x, box_center_y):
                    print("do not know x and y")
                    counting += 1
#                     wait_time_img(10)
                    if counting == 20:
                        dungim = False
                        # rtl(vehicle)
                        break
                    continue
                else:
                    counting = 0
                    
                ctr_signal_roll, ctr_signal_pitch, ctr_signal_z = controller(box_center_x, box_center_y, vehicle.location.global_relative_frame.alt, pidroll, pidpitch, pidz2)
                # print("box_center_x:", box_center_x)
                # print("box_center_y:", box_center_y)
                # print("frame_center_x:", frame_center_x)
                # print("frame_center_y:", frame_center_y)
                if box_center_x >= frame_center_x and box_center_y > frame_center_y:
                    print("bay 44")
                    bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                    
                if box_center_x < frame_center_x and box_center_y > frame_center_y:
                    print("bay 43")
                    bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
                    
                if box_center_x < frame_center_x and box_center_y < frame_center_y:
                    print("bay 42")
                    bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
                    
                if box_center_x > frame_center_x and box_center_y < frame_center_y:
                    print("bay 41")
                    bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                if box_center_x >= (frame_center_x-(frame_center_x*0.01)) and box_center_x <= (frame_center_x+(frame_center_x*0.01)):
                    if box_center_y >= (frame_center_y-(frame_center_y*0.01)) and box_center_y <= (frame_center_y+(frame_center_y*0.01)):
                        bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
                        print("target locked")
                        Target_locked = True
                        # time_start = time.time()
                        # while time.time() - time_start < 5:
                        time.sleep(0.1)
                        Run_servo = True
                        if Target_locked:
                            bay_only_pos(vehicle, 0, 0, 0)
                            if Run_servo:
                                servo(10, 1900)
                                servo(11, 1900)
                                servo(12, 2200)
                        Stop_servo = lis_esp()
                        if Stop_servo:
                            servo(12,1500)
                            wait_time_servo(10)
                            servo(12,800)
                            wait_time_servo(10)
                            break
                time.sleep(0.01)
            # set_pid_profile(pid_1)
            # time.sleep(3)
            cap.release()
            cv2.destroyAllWindows()
                
        # ha_canh(vehicle)

        # rtl(vehicle)
        #####################################################################################################
        flag_pid = False
        dungim   = False
        while True:
            if khong_thay_toa_do_muc_tieu:
                print("out")
                dungim = False
                break

            dist = distance()
            # print(dist)
            vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
            bay_xyz(vehicle, 0, 0, 0, -vel_x, -vel_y, -vel_z)
            
            if dist >= dist_set*0.9:
                print("bay tiep", dist)
                coef_ax = path_planning(0,0,tram_x,0,t_percent)   # x0, xf, vx0, vxf, t
                coef_ay = path_planning(0,0,tram_y,0,t_percent)   # y0, yf, vy0, vyf, t
                for ttime in np.arange(0,TT,0.1):
                    x, vx = pos_xyz(coef_ax, ttime)
                    y, vy = pos_xyz(coef_ay, ttime)
                    vel_z = controlforZ(pidzz=pidz1, altreal=vehicle.location.global_relative_frame.alt)
                    if vx < 5e-2:
                        break
                    bay_xyz(vehicle, 0, 0, 0, -vx, -vy, -vel_z)
                    time.sleep(0.01)
                if not dungim:
                    # Sau khi bay den diem
                    bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
                    time.sleep(0.1)
                    dungim = True
                if dungim:
                    break
            time.sleep(0.01)

        if dungim:
            print("mo camera")
            counting = 0
            frame = None
            frame_center_x, frame_center_y = None, None
            cap = cv2.VideoCapture(0)
            while not cap.isOpened():
                print("Can not open camera")
            cap.set(3, 640)
            cap.set(4, 640)

            while True:
                bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
                (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]
                
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("No frame")

                results = imgprocess(frame,model,labels)
                frame_center_x, frame_center_y, box_center_x, box_center_y = results
                if not flag_pid:
                    pidroll, pidpitch, pidz2 = PIDcontrol(kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, ki_z, kp_z, kd_z, setpoint_rll=frame_center_x, setpoint_ptc=frame_center_y, setpoint_z=alt_set)
                    flag_pid = True
                    
                if results is None:
                    print("Press q and exit")
                    khong_thay_toa_do_muc_tieu = True
                    break
                
                if None in (frame_center_x, frame_center_y, box_center_x, box_center_y):
                    print("do not know x and y")
                    counting += 1
#                     wait_time_img(10)
                    if counting == 20:
                        dungim = False
                        # rtl(vehicle)
                        break
                    continue
                else:
                    counting = 0
                    
                ctr_signal_roll, ctr_signal_pitch, ctr_signal_z = controller(box_center_x, box_center_y, vehicle.location.global_relative_frame.alt, pidroll, pidpitch, pidz2)
                # print("box_center_x:", box_center_x)
                # print("box_center_y:", box_center_y)
                # print("frame_center_x:", frame_center_x)
                # print("frame_center_y:", frame_center_y)
                # if box_center_x >= frame_center_x and box_center_y > frame_center_y:
                #     print("bay 44")
                #     bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                    
                # if box_center_x < frame_center_x and box_center_y > frame_center_y:
                #     print("bay 43")
                #     bay_xyz(vehicle, 0, 0, 0, -ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
                    
                # if box_center_x < frame_center_x and box_center_y < frame_center_y:
                #     print("bay 42")
                #     bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, -ctr_signal_pitch, -ctr_signal_z)
                    
                # if box_center_x > frame_center_x and box_center_y < frame_center_y:
                #     print("bay 41")
                #     bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                bay_xyz(vehicle, 0, 0, 0, ctr_signal_roll, ctr_signal_pitch, -ctr_signal_z)
                if box_center_x >= (frame_center_x-(frame_center_x*0.01)) and box_center_x <= (frame_center_x+(frame_center_x*0.01)):
                    if box_center_y >= (frame_center_y-(frame_center_y*0.01)) and box_center_y <= (frame_center_y+(frame_center_y*0.01)):
                        bay_xyz(vehicle, 0, 0, 0, 0, 0, 0)
                        print("target locked")
                        # time_start = time.time()
                        # while time.time() - time_start < 5:
                        time.sleep(0.1)
                        break
                time.sleep(0.01)
            cap.release()
            cv2.destroyAllWindows()
            ha_canh(vehicle)
        if not var_ref_flag:
            var_ref_flag = True
            i = 0
        if i<len(thu_tu_tram):
            i+=1
            thu_tu_tram[0] = thu_tu_tram[i]
        else: break
        Start = False
print("Complete all tasks")