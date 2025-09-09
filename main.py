import Module.MarkerDetector as cam_detector
import Module.Calibration as cali
import Module.Network as net
import numpy as np
#from pynput import keyboard
#import Module.Utilities as utils

isStopCamLoop = False
isCalibration = False

def init():
    print('init start')
    detector = cam_detector.MarkerDetector(1)
    init_offset = np.array([-0.06841773, 0.06819089, 0.00421293])
    probe_cal = cali.Calibration(init_offset, 1.5)
    networkManager = net.Network(1)
    cam_loop(detector, probe_cal, networkManager)

def cam_loop(detector, calibration, network):
    global isCalibration, isStopCamLoop
    print('main_loop Start')
    # Init Data
    zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R = detector.init_zed_camera()
    aruco_dict, aruco_params, runtime_parameters = detector.init_aruco_marker()
    init_offset = calibration.offset

    
    
    while True:
        # get marker data
        id_l, point_pos, R = detector.get_marker_data(zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R,
                                                                  aruco_dict, aruco_params, runtime_parameters,
                                                                  init_offset)
        if id_l == -1:
            continue
        if id_l == 10:
            if isCalibration == True:
                if calibration.get_list_count() < 10:
                    calibration.add_data(point_pos, R)
                else:
                    calibration.calculate_optimal_offset()
                    calibration.reset_lists()
            else:
                probe_tip_pos = detector.get_probe_tip_pos(calibration.offset, point_pos, R)
                print(f"Probe Tip Pos: {probe_tip_pos}")

        # probe calibration
        if isCalibration == True:
            if id_l == 10:
                if calibration.get_list_count() < 10:
                    calibration.add_data(point_pos, R)
                else:
                    calibration.calculate_optimal_offset()
                    calibration.reset_lists()
        
        # network
        if isStopCamLoop == True:
            break
    calibration.check_accept_data()

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
