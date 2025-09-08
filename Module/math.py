import math
import numpy as np

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