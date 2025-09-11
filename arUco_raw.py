import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time
import math
centers_list = []
x_axes_list = []
y_axes_list = []
z_axes_list = []

accept_data_list = []
init_offset = None
isCollect = False

def init():
    global init_offset
    #init_offset = np.array([-0.06743418, 0.06810218, 0.00329000])
    init_offset = np.array([-0.06841773, 0.06819089, 0.00421293])


def rotationMatrixToEulerAngles(R):
    """
    R: 3x3 회전 행렬 (numpy 배열)
    반환값: (roll, pitch, yaw) - 라디안 단위
    순서: roll (X축 회전), pitch (Y축 회전), yaw (Z축 회전)
    """

    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

    singular = sy < 1e-6  # 특이점 (gimbal lock) 여부 판단

    if not singular:
        roll = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw = math.atan2(R[1,0], R[0,0])
    else:
        # 특이점 처리
        roll = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw = 0

    return roll, pitch, yaw


def compute_pose_from_corners(corner3d: np.ndarray):
    """
    corner3d: (4, 3) ndarray, 마커의 3D 코너 좌표 (order: top-left, top-right, bottom-right, bottom-left)

    Returns:
        R: (3, 3) rotation matrix
        t: (3, 1) translation vector (중심 좌표)
    """
    # 마커 중심 계산
    center = np.mean(corner3d, axis=0)

    # x축 방향: top-right - top-left
    x_axis = corner3d[1] - corner3d[0]
    x_axis = x_axis / np.linalg.norm(x_axis)

    # y축 방향: bottom-left - top-left
    y_axis = corner3d[3] - corner3d[0]
    y_axis = y_axis / np.linalg.norm(y_axis)

    # z축은 x, y의 외적
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)

    # y축을 다시 정규화 (직교 보정)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)

    # 회전 행렬 구성
    R = np.vstack((x_axis, y_axis, z_axis)).T  # 3x3

    return R, center.reshape(3, 1)

def get_probe_tip_pos(offset: np.ndarray, probe_center: np.ndarray, R: np.ndarray) -> np.ndarray:
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



# Probe Calibration
def calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes):
    global accept_data_list
    global init_offset
    """
    C++의 CalProveOffset 함수를 Python으로 변환한 함수입니다.
    여러 측정 자세에서 계산된 월드 좌표들의 편차를 최소화하는 최적의 3D 오프셋을 찾습니다.

    Args:
        prove_centers (np.ndarray): (N, 3) 형태. 각 측정 자세에서의 마커 중심 좌표들.
        x_axes (np.ndarray): (N, 3) 형태. 각 측정 자세에서의 로컬 X축 벡터들.
        y_axes (np.ndarray): (N, 3) 형태. 각 측정 자세에서의 로컬 Y축 벡터들.
        z_axes (np.ndarray): (N, 3) 형태. 각 측정 자세에서의 로컬 Z축 벡터들.
        
    Returns:
        tuple: (최적 오프셋(np.ndarray), 최소 오차(float))
    """
    # 1. 알고리즘 파라미터 초기화
    step_num = 10       # 총 반복 횟수
    side_num = 10       # 각 차원의 그리드 분할 수
    step_rate = 0.5     # 탐색 범위 축소 비율
    step_size = 0.3     # 초기 탐색 범위 (단위: meter)
    #step_size = 300
    # 시작점 (초기 추정 오프셋)
    # C++ 코드의 값을 그대로 사용 (단위: meter)
    #current_pos = np.array([-0.096295, 0.096295, 0.003867])
    # 실제 측정 길이
    #current_pos = np.array([-0.067006, 0.067650, 0.003617])
    # 1차 캘리브레이션
    #current_pos = np.array([-0.0667, 0.0692, 0.0016])
    # 2차 캘리브레이션
    #current_pos = np.array([-0.06845775, 0.06829175, 0.00501300])
    # 3차 캘리브레이션
    #current_pos = np.array([-0.06771506, 0.06812835, 0.00385960])
    # 4차 캘리브레이션
    #current_pos = np.array([-0.06743418, 0.06810218, 0.00329000])
    total_grid_points = side_num ** 3
    num_proves = len(prove_centers)
    
    print("🚀 최적 오프셋 탐색을 시작합니다...")
    accept_offset_list = []
    accept_error_list = []
    # 2. 순차 대입법 기반의 반복 최적화 루프
    for s in range(step_num):
        # 현재 탐색 공간(정육면체)의 시작점과 그리드 간격 계산
        start_point = init_offset - step_size / 2.0
        grid_step = step_size / side_num
        
        # 각 그리드 포인트의 좌표와 오차를 저장할 배열
        grid_positions = np.zeros((total_grid_points, 3))
        grid_errors = np.zeros(total_grid_points)
        
        # 3. 10x10x10 그리드 탐색
        for x in range(side_num):
            for y in range(side_num):
                for z in range(side_num):
                    idx = x + y * side_num + z * side_num * side_num
                    
                    # 현재 그리드 포인트의 오프셋 후보 계산
                    local_offset = start_point + np.array([x, y, z]) * grid_step
                    grid_positions[idx] = local_offset
                    
                    # 4. 오차 계산
                    # 이 오프셋을 모든 측정 데이터에 적용하여 월드 좌표들을 계산
                    world_positions = np.zeros((num_proves, 3))
                    for i in range(num_proves):
                        world_positions[i] = prove_centers[i] + \
                                             x_axes[i] * local_offset[0] + \
                                             y_axes[i] * local_offset[1] + \
                                             z_axes[i] * local_offset[2]
                    
                    # 계산된 월드 좌표들의 중심점(centroid)을 구함
                    centroid = np.mean(world_positions, axis=0)
                    
                    # 각 월드 좌표와 중심점 사이의 평균 거리를 오차로 정의
                    distances = np.linalg.norm(world_positions - centroid, axis=1)
                    error = np.mean(distances)
                    grid_errors[idx] = error

        # 5. 현재 그리드에서 오차가 가장 작은 지점 찾기
        min_idx = np.argmin(grid_errors)
        min_error = grid_errors[min_idx]
        best_pos_in_grid = grid_positions[min_idx]
        
        # 6. 다음 반복을 위해 중심점과 탐색 범위 업데이트
        current_pos = best_pos_in_grid
        step_size *= step_rate
        
        # 진행 상황 출력 (mm 단위로 변환)
        pos_mm = current_pos  #* 1000
        error_mm = min_error * 1000
        print(f"[{s+1}/{step_num}] 오프셋: ({pos_mm[0]:.6f}, {pos_mm[1]:.6f}, {pos_mm[2]:.6f}) m | 최소 오차: {error_mm:.4f} mm")
        
        
        accept_offset_list.append(pos_mm)
        accept_error_list.append(error_mm)


    print("\n✅ 탐색 완료!")
    
    return prove_centers, accept_offset_list, accept_error_list

def live_aruco_detection():
    global centers_list, x_axes_list, y_axes_list, z_axes_list
    global isCollect
    global accept_data_list
    global init_offset
    # ZED 카메라 객체 생성 및 초기화
    zed = sl.Camera()
    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1200
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    init.coordinate_units = sl.UNIT.MILLIMETER

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print("ZED 열기 실패:", repr(err))
        zed.close()
        exit(1)

    # ZED에서 카메라 보정 파라미터 얻기
    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
    
    
    stereo_transform = calibration_params.stereo_transform

    intrinsics_L = calibration_params.left_cam  
    intrinsics_R = calibration_params.right_cam

    # Stero Camera 설정
    # 투영 행렬 계산
    camera_matrix_L = np.array([
        [intrinsics_L.fx, 0, intrinsics_L.cx],
        [0, intrinsics_L.fy, intrinsics_L.cy],
        [0, 0, 1]
    ])

    camera_matrix_R = np.array([
        [intrinsics_R.fx, 0, intrinsics_R.cx],
        [0, intrinsics_R.fy, intrinsics_R.cy],
        [0, 0, 1]
    ])
    rot = stereo_transform.get_rotation_matrix()  # 3x3 numpy array
    trans = stereo_transform.get_translation()    # sl.Translation (x, y, z)
    R = np.array(rot.r).reshape((3,3))
    T = np.array(trans.get()).reshape((3, 1))
    P1 = camera_matrix_L @ np.hstack((np.eye(3), np.zeros((3, 1))))
    P2 = camera_matrix_R @ np.hstack((R, T))

    # ArUco 마커 탐지기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    marker_size = 0.04  # 마커 크기 (단위: meter)

    left_image = sl.Mat()
    right_image = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()
    time.sleep(1)

    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_image(right_image, sl.VIEW.RIGHT)

            left_np = left_image.get_data()
            right_np = right_image.get_data()

            # 그레이 변환
            gray_left = cv2.cvtColor(left_np, cv2.COLOR_BGRA2GRAY)
            gray_right = cv2.cvtColor(right_np, cv2.COLOR_BGRA2GRAY)

            # ArUco 마커 검출
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners_l, ids_l, _ = detector.detectMarkers(gray_left)
            corners_r, ids_r, _ = detector.detectMarkers(gray_right)
            
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            win_size = (5, 5)
            zero_zone = (-1, -1)

            # 수집 시작
            if cv2.waitKey(1) & 0xFF == ord('c'):
                isCollect = True
                print(f"isCollect: {isCollect}")
            # 수집 종료
            if cv2.waitKey(1) & 0xFF == ord('v'):
                isCollect = False
                print(f"isCollect: {isCollect}")

            if ids_l is not None and ids_r is not None:
                for i, id_l in enumerate(ids_l):
                    if id_l in ids_r:
                        idx_r = np.where(ids_r == id_l)[0][0]
                        
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
                        points_4d_hom = cv2.triangulatePoints(P1, P2, center_l, center_r)
                        points_3d = (points_4d_hom / points_4d_hom[3])[:3]  # 정규화
                        points_3d_cv = points_3d.copy()
                        points_3d_cv[1] = -points_3d_cv[1]

                        corner3d_pts = np.zeros((4, 3))

                        # 회전 행렬 생성
                        for j in range(4):
                            pt_l = corner_l[j].reshape(2, 1)
                            pt_r = corner_r[j].reshape(2, 1)

                            pt_4d = cv2.triangulatePoints(P1, P2, pt_l, pt_r)
                            pt_3d = (pt_4d / pt_4d[3])[:3].ravel()  # ← 이게 중요함
                            pt_3d_cv = pt_3d.copy()
                            pt_3d_cv[1] = -pt_3d_cv[1]

                            corner3d_pts[j] = pt_3d_cv  # 정규화된 3D 좌표 저장

                        R, t = compute_pose_from_corners(corner3d_pts)

                        # 오일러 각
                        roll, pitch, yaw = rotationMatrixToEulerAngles(R)
                        # ------Get Pointcloud Data------
                        
                        #point_cloud = sl.Mat()
                        #zed.retrieve_measure(point_cloud, sl.MEASURE.XYZBGRA)

                        # ... 마커 코너의 2D 좌표 corner_l (u, v)를 얻은 후 ...
                        #corner3d_pts_2 = np.zeros((4, 3))
                        #passCount = 0
                        # 각 코너의 3D 좌표를 SDK에서 직접 조회합니다.
                        #for j in range(4):
                        #    u = int(corner_l[j][0])
                        #    v = int(corner_l[j][1])
                        #    err, point_3d = point_cloud.get_value(u, v)
                        #    x_3d, y_3d, z_3d = point_3d[0], point_3d[1], point_3d[2]
                        #    corner3d_pts_2[j] = [x_3d, y_3d, z_3d]
                        #    #if err == sl.ERROR_CODE.SUCCESS and math.isfinite(point_3d[2]):
                        #    #    # isfinite 체크로 유효한 깊이 값인지 확인
                        #    #    x_3d, y_3d, z_3d = point_3d[0], point_3d[1], point_3d[2]
                        #    #    corner3d_pts_2[j] = [x_3d, y_3d, z_3d]
                        #    #    passCount += 1
                        #    #else:
                        #    #    # 유효하지 않은 값이면 이 측정은 건너뛰는 로직 추가
                        #    #    pass 
                        #center = np.zeros(3)
                        #if passCount > 3:
                        #center = np.mean(corner3d_pts_2, axis=0)
                        #print(f"🎯 ID {int(id_l)} 위치 (pc)(mm): {center}")
                        #R_2, t_2 = compute_pose_from_corners(corner3d_pts)

                        # ------Get Pointcloud Data------

                        #print(f"🎯 ID {int(id_l)} 위치 (raw)(mm): {points_3d.ravel()}")

                        print(f"🎯 ID {int(id_l)} 위치 (cv)(mm): {points_3d_cv.ravel()}")
                        print(f"🎯 ID {int(id_l)} 회전 값 (cv)(도): {roll * 180/math.pi}, {pitch* 180/math.pi}, {yaw* 180/math.pi}")
                        
                        if int(id_l) == 10:
                            offset = init_offset * 1000
                            probe_center = points_3d_cv.ravel()
                            #axes = np.ndarray(R[:, 0], R[:, 1], R[:, 2])
                            probe_tip_pos = get_probe_tip_pos(offset, probe_center, R) 
                            print(f"probe_centerpos: {probe_center} \n probe_tip_pos: {probe_tip_pos}")
                            # 3. cv2.projectPoints()를 위한 입력값 준비
                            # ZED의 3D 좌표는 이미 왼쪽 카메라 기준이므로 rvec, tvec은 0으로 설정
                            rvec = np.zeros((3, 1))
                            tvec = np.zeros((3, 1))

                            # 투영할 3D 포인트를 올바른 shape으로 변환: (1, 1, 3)
                            point_3d_to_project = probe_tip_pos.reshape(1, 1, 3)

                            # 4. 3D 좌표를 2D 픽셀 좌표로 투영
                            # camera_matrix_L와 intrinsics_L.disto는 미리 계산되어 있어야 함
                            point_2d_projected, _ = cv2.projectPoints(point_3d_to_project, rvec, tvec, camera_matrix_L, intrinsics_L.disto)

                            # 5. 투영된 2D 좌표를 화면에 그리기
                            if point_2d_projected is not None:
                                # pt_2d의 shape은 (1, 1, 2)이므로, 정수형 (x, y) 튜플로 변환
                                pt_2d = tuple(point_2d_projected.ravel().astype(int))

                                # 왼쪽 카메라 뷰(left_np)에 십자(+) 모양으로 그리기
                                cv2.drawMarker(left_np, pt_2d, color=(0, 0, 255), markerType=cv2.MARKER_CROSS, 
                                               markerSize=20, thickness=2)
                        if isCollect: 
                            print(f"[ADD!] center: {points_3d_cv.ravel()} \n x_axes: {R[:, 0]} \n y_axes: {R[:, 1]} \n z_axes: {R[:, 2]}")
                            centers_list.append(points_3d_cv.ravel()/1000)
                            x_axes_list.append(R[:, 0])
                            y_axes_list.append(R[:, 1]) 
                            z_axes_list.append(R[:, 2])
                        
                        if len(centers_list) > 9:
                            # 이 코드도 전역 리스트를 올바르게 참조합니다.
                            prove_centers = np.array(centers_list)
                            x_axes = np.array(x_axes_list)
                            y_axes = np.array(y_axes_list)
                            z_axes = np.array(z_axes_list)
                            # ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 여기에 추가 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
                            print("\n" + "="*25 + " 데이터 확인 " + "="*25)
                            num_points = len(prove_centers)
                            for i in range(num_points):
                                # f-string과 서식 지정자를 이용해 깔끔하게 출력
                                print(f"[{i+1:02d}] Center=[{prove_centers[i][0]: .4f}, {prove_centers[i][1]: .4f}, {prove_centers[i][2]: .4f}] | "
                                      f"X_Axis=[{x_axes[i][0]: .4f}, {x_axes[i][1]: .4f}, {x_axes[i][2]: .4f}]")
                            print("="*62 + "\n")
                            # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 여기까지 추가 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
                            center, cal_offset, errors = calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes)
                            error_avg = 0.0
                            for error in errors:
                                error_avg += error
                            error_avg /= 10
                            if error_avg < 1.5:
                                accept_data_list.append((center, cal_offset, errors))
                            # 이 할당문도 이제 전역 리스트를 비우는 동작을 올바르게 수행합니다.
                            centers_list = []
                            x_axes_list = []
                            y_axes_list = []
                            z_axes_list = [] 
                        #print(f"🎯 ID {int(id_l)} 회전 값 : {v1}, {v2}, {v3}")

                        # 시각화
                        cv2.circle(left_np, tuple(center_l.ravel().astype(int)), 5, (0, 255, 0), -1)
                        cv2.circle(right_np, tuple(center_r.ravel().astype(int)), 5, (0, 0, 255), -1)



            # 화면 출력
            combined = np.hstack((left_np, right_np))
            resized_img = cv2.resize(combined, (1280, 480))
            cv2.imshow("Left | Right", resized_img)
            # 'q' 키 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 종료 처리
    zed.close()
    cv2.destroyAllWindows()

def check_accept_data():
    global accept_data_list
    print("\n" + "="*25 + " 최종 합격 데이터 " + "="*25)
    dist = 0.0
    if not accept_data_list:
        print("수집된 데이터가 없습니다.")
    else:
        # enumerate를 사용하면 인덱스 번호(i)와 항목(data)을 한번에 가져올 수 있습니다.
        for i, data in enumerate(accept_data_list):
            # data는 (offset, error) 형태의 튜플입니다.
            pose = data[0]
            offset = data[1]
            error = data[2]

            # f-string 서식을 이용해 보기 좋게 출력
            print(f"--- 데이터 [{i+1}/{len(accept_data_list)}] ---")

            # NumPy 배열을 보기 좋게 출력하기 위한 서식
            #offset_str = np.array2string(offset, formatter={'float_kind':lambda x: "%.6f" % x})
            for j in range(len(pose)):
                dist += pose[j][2]
                print(f"  center pose: {pose[j]}")
                print(f"  offset: {offset[j]}")
                print(f"  error: {error[j]:.4f} mm")
            dist /= 10
            dist *= 1000
            print(f"--- 데이터 [{i+1}/{len(accept_data_list)} 거리: {dist * -1:.1f}mm] ---")
            dist = 0.0

    print("="*64)


def main():
    init()
    print("Starting ArUco marker detection with ZED camera...")
    live_aruco_detection()
    print("End marker detection with ZED camera...")
    check_accept_data()

if __name__ == "__main__":
    main()