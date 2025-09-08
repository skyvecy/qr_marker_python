import Module.MarkerDetector as cam_detector
import Module.Calibration as cali
import Module.Network as net
import numpy as np

def init():
    detector = cam_detector.MarkerDetector(1)
    init_offset = np.array([-0.06841773, 0.06819089, 0.00421293])
    probe_cal = cali.Calibration(init_offset)
    networkManager = net.Network(1)



    main_loop(detector, probe_cal, networkManager)

def main_loop(detector, calibration, network):
    zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R = detector.init_zed_camera()
    aruco_dict, aruco_params, runtime_parameters = detector.init_aruco_marker()
    init_offset = calibration.self.offset

    while True:
        loop_state, id_l, point_pos, R = detector.get_marker_data(zed, left_image, right_image, P1, P2, camera_matrix_L, camera_matrix_R, intrinsics_L, intrinsics_R,
                                                                  aruco_dict, aruco_params, runtime_parameters,
                                                                  init_offset)
        if loop_state == False:
            break

    

if __name__ == "__main__":
    init()