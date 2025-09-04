import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time

def live_aruco_detection():
    # ZED 카메라 객체 생성 및 초기화
    zed = sl.Camera()
    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.MILLIMETER

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print("ZED 열기 실패:", repr(err))
        zed.close()
        exit(1)

    # ZED에서 카메라 보정 파라미터 얻기
    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
    intrinsics = calibration_params.left_cam

    # OpenCV용 카메라 행렬과 왜곡 계수 구성
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.cx],
        [0, intrinsics.fy, intrinsics.cy],
        [0, 0, 1]
    ])
    dist_coeffs = np.array(intrinsics.disto[:5])  # 보통 앞의 5개만 사용

    # ArUco 마커 탐지기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    marker_size = 0.04  # 마커 크기 (단위: meter)

    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    time.sleep(1)

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # 왜곡 보정
            frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            
            # 마커 검출
            corners, ids, rejected = detector.detectMarkers(frame_undistorted)
            #cv2.cornerSubPix(frame, corners, cv2.Size(5, 5), cv2.Size(-1, -1))

            if ids is not None:
                # 마커 시각화
                cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)

                # 포즈 추정
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs
                )

                for i in range(len(ids)):
                    # 좌표축 그리기
                    cv2.drawFrameAxes(frame_undistorted, camera_matrix, dist_coeffs,
                                      rvecs[i], tvecs[i], marker_size / 2)

                    # 마커 중심 위치 계산
                    pos_x = tvecs[i][0][0]
                    pos_y = tvecs[i][0][1]
                    pos_z = tvecs[i][0][2]

                    # 회전 벡터 → 오일러 각
                    rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                    euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]

                    # 텍스트 표시용 위치
                    corner = corners[i][0]
                    center_x = int(np.mean(corner[:, 0]))
                    center_y = int(np.mean(corner[:, 1]))

                    cv2.putText(frame_undistorted,
                                f"ID: {ids[i][0]}",
                                (center_x, center_y - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    cv2.putText(frame_undistorted,
                                f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",
                                (center_x, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    cv2.putText(frame_undistorted,
                                f"Rot: ({euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f})deg",
                                (center_x, center_y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                    for point in corner:
                        x, y = int(point[0]), int(point[1])
                        cv2.circle(frame_undistorted, (x, y), 4, (0, 0, 255), -1)

            # 프레임 출력
            cv2.imshow('ArUco Marker Detection', frame_undistorted)

        # 'q' 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 처리
    zed.close()
    cv2.destroyAllWindows()

def main():
    print("Starting ArUco marker detection with ZED camera...")
    live_aruco_detection()

if __name__ == "__main__":
    main()