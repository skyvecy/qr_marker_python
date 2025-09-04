[1mdiff --git a/ArukoTest_2.py b/ArukoTest_2.py[m
[1mindex fd604d1..3bad753 100644[m
[1m--- a/ArukoTest_2.py[m
[1m+++ b/ArukoTest_2.py[m
[36m@@ -8,8 +8,16 @@[m [mcenters_list = [][m
 x_axes_list = [][m
 y_axes_list = [][m
 z_axes_list = [][m
[32m+[m
[32m+[m[32maccept_data_list = [][m
[32m+[m[32minit_offset = None[m
 isCollect = False[m
 [m
[32m+[m[32mdef init():[m
[32m+[m[32m    global init_offset[m
[32m+[m[32m    #init_offset = np.array([-0.06743418, 0.06810218, 0.00329000])[m
[32m+[m[32m    init_offset = np.array([-0.06841773, 0.06819089, 0.00421293])[m
[32m+[m
 [m
 def rotationMatrixToEulerAngles(R):[m
     """[m
[36m@@ -67,14 +75,6 @@[m [mdef compute_pose_from_corners(corner3d: np.ndarray):[m
 [m
     return R, center.reshape(3, 1)[m
 [m
[31m-#def get_probe_tip_pos(offset, probe_centers, axes):[m
[31m-#    tip_position = np.zeros((3))[m
[31m-#    tip_position = probe_centers + \[m
[31m-#                     axes[0] * offset[0] + \[m
[31m-#                     axes[1] * offset[1] + \[m
[31m-#                     axes[2] * offset[2][m
[31m-#    return tip_position[m
[31m-[m
 def get_probe_tip_pos(offset: np.ndarray, probe_center: np.ndarray, R: np.ndarray) -> np.ndarray:[m
     """[m
     마커의 자세(R, t)와 캘리브레이션된 오프셋을 사용하여 프로브 팁의 월드 좌표를 계산합니다.[m
[36m@@ -97,6 +97,8 @@[m [mdef get_probe_tip_pos(offset: np.ndarray, probe_center: np.ndarray, R: np.ndarra[m
 [m
 # Probe Calibration[m
 def calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes):[m
[32m+[m[32m    global accept_data_list[m
[32m+[m[32m    global init_offset[m
     """[m
     C++의 CalProveOffset 함수를 Python으로 변환한 함수입니다.[m
     여러 측정 자세에서 계산된 월드 좌표들의 편차를 최소화하는 최적의 3D 오프셋을 찾습니다.[m
[36m@@ -128,16 +130,17 @@[m [mdef calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes):[m
     # 3차 캘리브레이션[m
     #current_pos = np.array([-0.06771506, 0.06812835, 0.00385960])[m
     # 4차 캘리브레이션[m
[31m-    current_pos = np.array([-0.06743418, 0.06810218, 0.00329000])[m
[32m+[m[32m    #current_pos = np.array([-0.06743418, 0.06810218, 0.00329000])[m
     total_grid_points = side_num ** 3[m
     num_proves = len(prove_centers)[m
     [m
     print("🚀 최적 오프셋 탐색을 시작합니다...")[m
[31m-    [m
[32m+[m[32m    accept_offset_list = [][m
[32m+[m[32m    accept_error_list = [][m
     # 2. 순차 대입법 기반의 반복 최적화 루프[m
     for s in range(step_num):[m
         # 현재 탐색 공간(정육면체)의 시작점과 그리드 간격 계산[m
[31m-        start_point = current_pos - step_size / 2.0[m
[32m+[m[32m        start_point = init_offset - step_size / 2.0[m
         grid_step = step_size / side_num[m
         [m
         # 각 그리드 포인트의 좌표와 오차를 저장할 배열[m
[36m@@ -182,15 +185,23 @@[m [mdef calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes):[m
         [m
         # 진행 상황 출력 (mm 단위로 변환)[m
         pos_mm = current_pos  #* 1000[m
[31m-        error_mm = min_error  * 1000[m
[32m+[m[32m        error_mm = min_error * 1000[m
         print(f"[{s+1}/{step_num}] 오프셋: ({pos_mm[0]:.6f}, {pos_mm[1]:.6f}, {pos_mm[2]:.6f}) m | 최소 오차: {error_mm:.4f} mm")[m
[32m+[m[41m        [m
[32m+[m[41m        [m
[32m+[m[32m        accept_offset_list.append(pos_mm)[m
[32m+[m[32m        accept_error_list.append(error_mm)[m
[32m+[m
 [m
     print("\n✅ 탐색 완료!")[m
[31m-    return current_pos, min_error[m
[32m+[m[41m    [m
[32m+[m[32m    return prove_centers, accept_offset_list, accept_error_list[m
 [m
 def live_aruco_detection():[m
     global centers_list, x_axes_list, y_axes_list, z_axes_list[m
     global isCollect[m
[32m+[m[32m    global accept_data_list[m
[32m+[m[32m    global init_offset[m
     # ZED 카메라 객체 생성 및 초기화[m
     zed = sl.Camera()[m
     input_type = sl.InputType()[m
[36m@@ -268,9 +279,11 @@[m [mdef live_aruco_detection():[m
             win_size = (5, 5)[m
             zero_zone = (-1, -1)[m
 [m
[32m+[m[32m            # 수집 시작[m
             if cv2.waitKey(1) & 0xFF == ord('c'):[m
                 isCollect = True[m
                 print(f"isCollect: {isCollect}")[m
[32m+[m[32m            # 수집 종료[m
             if cv2.waitKey(1) & 0xFF == ord('v'):[m
                 isCollect = False[m
                 print(f"isCollect: {isCollect}")[m
[36m@@ -356,13 +369,31 @@[m [mdef live_aruco_detection():[m
                         print(f"🎯 ID {int(id_l)} 회전 값 (cv)(도): {roll * 180/math.pi}, {pitch* 180/math.pi}, {yaw* 180/math.pi}")[m
                         [m
                         if int(id_l) == 10:[m
[31m-                            offset = np.array([-0.06743418, 0.06810218, 0.00329000])[m
[32m+[m[32m                            offset = init_offset * 1000[m
                             probe_center = points_3d_cv.ravel()[m
                             #axes = np.ndarray(R[:, 0], R[:, 1], R[:, 2])[m
[31m-                            probe_tip_pos = get_probe_tip_pos(offset, probe_center, R)[m
[31m-                            print(f"probe_centerpos: {probe_center} \n probe_tip_pos: {probe_tip_pos} \n delta: {probe_tip_pos - probe_center}")[m
[31m-[m
[31m-[m
[32m+[m[32m                            probe_tip_pos = get_probe_tip_pos(offset, probe_center, R)[m[41m [m
[32m+[m[32m                            print(f"probe_centerpos: {probe_center} \n probe_tip_pos: {probe_tip_pos}")[m
[32m+[m[32m                            # 3. cv2.projectPoints()를 위한 입력값 준비[m
[32m+[m[32m                            # ZED의 3D 좌표는 이미 왼쪽 카메라 기준이므로 rvec, tvec은 0으로 설정[m
[32m+[m[32m                            rvec = np.zeros((3, 1))[m
[32m+[m[32m                            tvec = np.zeros((3, 1))[m
[32m+[m
[32m+[m[32m                            # 투영할 3D 포인트를 올바른 shape으로 변환: (1, 1, 3)[m
[32m+[m[32m                            point_3d_to_project = probe_tip_pos.reshape(1, 1, 3)[m
[32m+[m
[32m+[m[32m                            # 4. 3D 좌표를 2D 픽셀 좌표로 투영[m
[32m+[m[32m                            # camera_matrix_L와 intrinsics_L.disto는 미리 계산되어 있어야 함[m
[32m+[m[32m                            point_2d_projected, _ = cv2.projectPoints(point_3d_to_project, rvec, tvec, camera_matrix_L, intrinsics_L.disto)[m
[32m+[m
[32m+[m[32m                            # 5. 투영된 2D 좌표를 화면에 그리기[m
[32m+[m[32m                            if point_2d_projected is not None:[m
[32m+[m[32m                                # pt_2d의 shape은 (1, 1, 2)이므로, 정수형 (x, y) 튜플로 변환[m
[32m+[m[32m                                pt_2d = tuple(point_2d_projected.ravel().astype(int))[m
[32m+[m
[32m+[m[32m                                # 왼쪽 카메라 뷰(left_np)에 십자(+) 모양으로 그리기[m
[32m+[m[32m                                cv2.drawMarker(left_np, pt_2d, color=(0, 0, 255), markerType=cv2.MARKER_CROSS,[m[41m [m
[32m+[m[32m                                               markerSize=20, thickness=2)[m
                         if isCollect: [m
                             print(f"[ADD!] center: {points_3d_cv.ravel()} \n x_axes: {R[:, 0]} \n y_axes: {R[:, 1]} \n z_axes: {R[:, 1]}")[m
                             centers_list.append(points_3d_cv.ravel()/1000)[m
[36m@@ -370,13 +401,6 @@[m [mdef live_aruco_detection():[m
                             y_axes_list.append(R[:, 1]) [m
                             z_axes_list.append(R[:, 2])[m
                         [m
[31m-[m
[31m-                            #print(f"[ADD!] center: {points_3d_cv.ravel()} \n x_axes: {R[:, 0]} \n y_axes: {R[:, 1]} \n z_axes: {R[:, 1]}")[m
[31m-                            #centers_list.append(points_3d_cv.ravel()/1000)[m
[31m-                            #x_axes_list.append(R[:, 0])[m
[31m-                            #y_axes_list.append(R[:, 1])[m
[31m-                            #z_axes_list.append(R[:, 2])[m
[31m-                        [m
                         if len(centers_list) > 9:[m
                             # 이 코드도 전역 리스트를 올바르게 참조합니다.[m
                             prove_centers = np.array(centers_list)[m
[36m@@ -392,8 +416,13 @@[m [mdef live_aruco_detection():[m
                                       f"X_Axis=[{x_axes[i][0]: .4f}, {x_axes[i][1]: .4f}, {x_axes[i][2]: .4f}]")[m
                             print("="*62 + "\n")[m
                             # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 여기까지 추가 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲[m
[31m-                            calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes)[m
[31m-[m
[32m+[m[32m                            center, cal_offset, errors = calculate_optimal_offset(prove_centers, x_axes, y_axes, z_axes)[m
[32m+[m[32m                            error_avg = 0.0[m
[32m+[m[32m                            for error in errors:[m
[32m+[m[32m                                error_avg += error[m
[32m+[m[32m                            error_avg /= 10[m
[32m+[m[32m                            if error_avg < 1.5:[m
[32m+[m[32m                                accept_data_list.append((center, cal_offset, errors))[m
                             # 이 할당문도 이제 전역 리스트를 비우는 동작을 올바르게 수행합니다.[m
                             centers_list = [][m
                             x_axes_list = [][m
[36m@@ -405,6 +434,8 @@[m [mdef live_aruco_detection():[m
                         cv2.circle(left_np, tuple(center_l.ravel().astype(int)), 5, (0, 255, 0), -1)[m
                         cv2.circle(right_np, tuple(center_r.ravel().astype(int)), 5, (0, 0, 255), -1)[m
 [m
[32m+[m
[32m+[m
             # 화면 출력[m
             combined = np.hstack((left_np, right_np))[m
             resized_img = cv2.resize(combined, (1280, 480))[m
[36m@@ -417,9 +448,44 @@[m [mdef live_aruco_detection():[m
     zed.close()[m
     cv2.destroyAllWindows()[m
 [m
[32m+[m[32mdef check_accept_data():[m
[32m+[m[32m    global accept_data_list[m
[32m+[m[32m    print("\n" + "="*25 + " 최종 합격 데이터 " + "="*25)[m
[32m+[m[32m    dist = 0.0[m
[32m+[m[32m    if not accept_data_list:[m
[32m+[m[32m        print("수집된 데이터가 없습니다.")[m
[32m+[m[32m    else:[m
[32m+[m[32m        # enumerate를 사용하면 인덱스 번호(i)와 항목(data)을 한번에 가져올 수 있습니다.[m
[32m+[m[32m        for i, data in enumerate(accept_data_list):[m
[32m+[m[32m            # data는 (offset, error) 형태의 튜플입니다.[m
[32m+[m[32m            pose = data[0][m
[32m+[m[32m            offset = data[1][m
[32m+[m[32m            error = data[2][m
[32m+[m
[32m+[m[32m            # f-string 서식을 이용해 보기 좋게 출력[m
[32m+[m[32m            print(f"--- 데이터 [{i+1}/{len(accept_data_list)}] ---")[m
[32m+[m
[32m+[m[32m            # NumPy 배열을 보기 좋게 출력하기 위한 서식[m
[32m+[m[32m            #offset_str = np.array2string(offset, formatter={'float_kind':lambda x: "%.6f" % x})[m
[32m+[m[32m            for j in range(len(pose)):[m
[32m+[m[32m                dist += pose[j][2][m
[32m+[m[32m                print(f"  center pose: {pose[j]}")[m
[32m+[m[32m                print(f"  offset: {offset[j]}")[m
[32m+[m[32m                print(f"  error: {error[j]:.4f} mm")[m
[32m+[m[32m            dist /= 10[m
[32m+[m[32m            dist *= 1000[m
[32m+[m[32m            print(f"--- 데이터 [{i+1}/{len(accept_data_list)} 거리: {dist * -1:.1f}mm] ---")[m
[32m+[m[32m            dist = 0.0[m
[32m+[m
[32m+[m[32m    print("="*64)[m
[32m+[m
[32m+[m
 def main():[m
[32m+[m[32m    init()[m
     print("Starting ArUco marker detection with ZED camera...")[m
     live_aruco_detection()[m
[32m+[m[32m    print("End marker detection with ZED camera...")[m
[32m+[m[32m    check_accept_data()[m
 [m
 if __name__ == "__main__":[m
     main()[m
\ No newline at end of file[m
