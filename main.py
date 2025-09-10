import Module.MarkerDetector as cam_detector
import Module.Calibration as cali
import Module.Network as net
import numpy as np
from Module.Data import MarkerData as data
#from pynput import keyboard
#import Module.Utilities as utils

isStopCamLoop = False
isCalibration = False

def init():
    print('init start')
    detector = cam_detector.MarkerDetector(1)
    init_offset = np.array([-0.06841773, 0.06819089, 0.00421293])
    probe_cal = cali.Calibration(init_offset, 1.5)
    
    cam_loop(detector, probe_cal)

def cam_loop(detector, calibration):
    global isCalibration, isStopCamLoop
    print('main_loop Start')
    # Init Data
    zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R = detector.init_zed_camera()
    aruco_dict, aruco_params, runtime_parameters = detector.init_aruco_marker()
    init_offset = calibration.offset

    network = net.Network(9001, 9999)
    network.init_server()

    marker_id_list = []
    marker_id_list.append(20)
    marker_id_list.append(21)
    marker_id_list.append(10)
    marker_id_list.append(22)
    marker_id_list.append(23)
    

    for i in range(5):
        marker_data = data(0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0 ,-1.0)
        network.dto_list.append(marker_data)
    
    while True:
        data_list = detector.get_marker_data(zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R,
                                                        aruco_dict, aruco_params, runtime_parameters,
                                                        init_offset)
        
        for index, value in enumerate(marker_id_list):
            network.dto_list[index].set_data(0, np.zeros(3,), np.zeros((3,3)))
            if len(data_list) > 0:
                for i in range(len(data_list)):
                    id_l, point_pos, R = data_list[i]
                    if value == id_l:
                        if id_l == 10:
                            if isCalibration == True:
                                if calibration.get_list_count() < 10:
                                    calibration.add_data(point_pos, R)
                                else:
                                    calibration.calculate_optimal_offset()
                                    calibration.reset_lists()
                            else:
                                probe_tip_pos = detector.get_probe_tip_pos(calibration.offset, point_pos, R)
                                network.dto_list[index].set_data(1, probe_tip_pos, R)
                        else:
                            network.dto_list[index].set_data(1, point_pos, R)

        print("="*45)
        print(f"network.dto_list\n{network.dto_list}")
        print("="*45)
        # network
        if isStopCamLoop == True:
            break
    calibration.check_accept_data()
    network.join_thread()

#def control_calibration(key):
#    global isCalibration, isStopCamLoop
#    
#    if key.char == 'q':
#        print('keyinput: q')
#        isStopCamLoop = True
#    elif key.char =='c':
#        print('keyinput: c')
#        isCalibration = True
#    elif key.char == 'x':
#        print('keyinput: x')
#        isCalibration = False
#def on_release(key):
#    """키를 뗐을 때 호출될 함수"""
#    print(f'{key} 에서 손을 뗌')
#    # 'esc' 키를 누르면 리스너를 종료합니다.
#    if key == keyboard.Key.esc:
#        return False

# 리스너를 생성하고 백그라운드에서 실행합니다.
#with keyboard.Listener(on_press=control_calibration, on_release=on_release) as listener:
#    listener.join() # 리스너가 종료될 때까지 메인 스레드를 대기시킵니다.


if __name__ == "__main__":
    init()
