import pyzed.sl as sl
import numpy as np
import cv2
from .ComputeMath import rotationMatrixToEulerAngles
from .ComputeMath import compute_pose_from_corners
import math

class MarkerDetector:
    # 제드 카메라 초기화
    def __init__(self, temp):
        print(f"setting value: {temp}")
        print("MarkerDetector Initializing Completed.")
        self.zed = sl.Camera()
        input_type = sl.InputType()
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.HD1200
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
        init.coordinate_units = sl.UNIT.MILLIMETER

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print("ZED 열기 실패:", repr(err))
            self.zed.close()
            exit(1)
            # ZED에서 카메라 보정 파라미터 얻기

        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters
    
        self.intrinsics_L = calibration_params.left_cam  
        self.intrinsics_R = calibration_params.right_cam
    
        # 투영 행렬 계산
        self.camera_matrix_L = np.array([
            [self.intrinsics_L.fx, 0, self.intrinsics_L.cx],
            [0, self.intrinsics_L.fy, self.intrinsics_L.cy],
            [0, 0, 1]
        ])
    
        self.camera_matrix_R = np.array([
            [self.intrinsics_R.fx, 0, self.intrinsics_R.cx],
            [0, self.intrinsics_R.fy, self.intrinsics_R.cy],
            [0, 0, 1]
        ])
        stereo_transform = calibration_params.stereo_transform
        rot = stereo_transform.get_rotation_matrix()  # 3x3 numpy array
        trans = stereo_transform.get_translation()    # sl.Translation (x, y, z)
        R = np.array(rot.r).reshape((3,3))
        T = np.array(trans.get()).reshape((3, 1))
        self.P1 = self.camera_matrix_L @ np.hstack((np.eye(3), np.zeros((3, 1))))
        self.P2 = self.camera_matrix_R @ np.hstack((R, T))
        self.left_image = sl.Mat()
        self.right_image = sl.Mat()

    # arUco 마커 초기화
    def init_aruco_marker(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        #detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        #marker_size = 0.04  # 마커 크기 (단위: meter)

        self.runtime_parameters = sl.RuntimeParameters()

    def get_probe_tip_pos(self, offset: np.ndarray, probe_center: np.ndarray, R: np.ndarray) -> np.ndarray:
        """
        마커의 자세(R, t)와 캘리브레이션된 오프셋을 사용하여 프로브 팁의 월드 좌표를 계산합니다.

        Args:
            offset (np.ndarray): 프로브 팁의 로컬 오프셋 벡터 (3,)
            probe_center (np.ndarray): 마커 중심의 월드 좌표 t (3,)
            R (np.ndarray): 마커의 회전 행렬 (3, 3)

        Returns:
            np.ndarray: 프로브 팁의 최종 월드 좌표 (3,)
        """
        # R @ offset: 로컬 오프셋을 월드 좌표계의 방향으로 변환합니다.
        # probe_center + ...: 월드 좌표계의 마커 중심에 변환된 오프셋을 더합니다.
        tip_position_world = probe_center + (R @ offset)
        
        return tip_position_world
    # Clean
    def get_marker_data(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.right_image, sl.VIEW.RIGHT)

            left_np = self.left_image.get_data()
            right_np = self.right_image.get_data()

            # 그레이 변환
            gray_left = cv2.cvtColor(left_np, cv2.COLOR_BGRA2GRAY)
            gray_right = cv2.cvtColor(right_np, cv2.COLOR_BGRA2GRAY)

            # ArUco 마커 검출
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners_l, ids_l, _ = detector.detectMarkers(gray_left)
            corners_r, ids_r, _ = detector.detectMarkers(gray_right)
            
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            win_size = (5, 5)
            zero_zone = (-1, -1)

            markerinfodata_list = []
            currId = -1
            point_pos = np.zeros(3,)
            R = np.zeros((3,3))
            if ids_l is not None and ids_r is not None:
                for i, id_l in enumerate(ids_l):
                    if id_l in ids_r:
                        idx_r = np.where(ids_r == id_l)[0][0]
                        # 특정 id 값만 가져온다.
                        #왜곡
                        #undistorted_pts_l = cv2.undistortPoints(corners_l, camera_matrix_L, intrinsics_L.disto)
                        #undistorted_pts_r = cv2.undistortPoints(corners_r, camera_matrix_R, intrinsics_R.disto)

                        # 마커 중심 좌표 계산 (픽셀 단위)
                        corner_l = corners_l[i][0]
                        corner_r = corners_r[idx_r][0]

                        cv2.cornerSubPix(gray_left, corner_l, win_size, zero_zone, criteria)
                        cv2.cornerSubPix(gray_right, corner_r, win_size, zero_zone, criteria)


                        center_l = np.mean(corner_l, axis=0).reshape(2, 1)
                        center_r = np.mean(corner_r, axis=0).reshape(2, 1)

                        # 삼각측량을 통한 3D 위치 계산
                        points_4d_hom = cv2.triangulatePoints(self.P1, self.P2, center_l, center_r)
                        points_3d = (points_4d_hom / points_4d_hom[3])[:3]  # 정규화
                        points_3d_cv = points_3d.copy()
                        points_3d_cv[1] = -points_3d_cv[1]

                        corner3d_pts = np.zeros((4, 3))

                        # 회전 행렬 생성
                        for j in range(4):
                            pt_l = corner_l[j].reshape(2, 1)
                            pt_r = corner_r[j].reshape(2, 1)

                            pt_4d = cv2.triangulatePoints(self.P1, self.P2, pt_l, pt_r)
                            pt_3d = (pt_4d / pt_4d[3])[:3].ravel()  # ← 이게 중요함
                            pt_3d_cv = pt_3d.copy()
                            pt_3d_cv[1] = -pt_3d_cv[1]

                            corner3d_pts[j] = pt_3d_cv  # 정규화된 3D 좌표 저장

                        R, t = compute_pose_from_corners(corner3d_pts)

                        # 오일러 각
                        roll, pitch, yaw = rotationMatrixToEulerAngles(R)

                        point_pos = points_3d_cv.ravel()
                        currId = id_l
                        
                        #print(f"🎯 ID {int(id_l)} 위치 (cv)(mm): {point_pos}")
                        #print(f"🎯 ID {int(id_l)} 회전 값 (cv)(도): {roll * 180/math.pi}, {pitch* 180/math.pi}, {yaw* 180/math.pi}")
                        markerinfodata_list.append((currId, point_pos, R))
                        # 시각화
                        cv2.circle(left_np, tuple(center_l.ravel().astype(int)), 5, (0, 255, 0), -1)
                        cv2.circle(right_np, tuple(center_r.ravel().astype(int)), 5, (0, 0, 255), -1)


            #print(markerinfodata_list)
            # 화면 출력
            combined = np.hstack((left_np, right_np))
            resized_img = cv2.resize(combined, (1280, 480))
            cv2.imshow("Left | Right", resized_img)
            cv2.waitKey(1) 

            return markerinfodata_list
            #if target_id == currId:
            #return currId, point_pos, R
            #else:
            #    return -1, np.zeros(3,), np.zeros((3,3))
        else:
            pass
  



