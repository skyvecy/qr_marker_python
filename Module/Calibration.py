import numpy as np


class Calibration:
    probe_center_list = []
    x_axes_list = []
    y_axes_list = []
    z_axes_list = []

    # tuple(center, offset, errors)
    accept_data_list = [[],[],[]]
    def __init__(self, offset, accept_error):
        self.offset = offset
        self.accept_error = accept_error
        print(f"setting value: {offset}")
        print("Calibration Initializing Completed.")

    def add_data(self, probe_center, R):
        # m단위로 변환
        self.probe_center_list.append(probe_center/1000)
        self.x_axes_list.append(R[:, 0])
        self.y_axes_list.append(R[:, 1]) 
        self.z_axes_list.append(R[:, 2])
        
    def get_list_count(self):
        if not self.probe_center_list:
            return -1
        else:
            return len(self.probe_center_list)

    def reset_lists(self):
        self.probe_center_list = []
        self.x_axes_list = []
        self.y_axes_list = []
        self.z_axes_list = []
    # Probe Calibration
    def calculate_optimal_offset(self):
        init_offset = self.offset
        error_avg = 0.0
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
        total_grid_points = side_num ** 3
        num_proves = len(self.probe_center_list)
        
        print("🚀 최적 오프셋 탐색을 시작합니다...")
        offset_list = []
        error_list = []
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
                            world_positions[i] = self.probe_center_list[i] + \
                                                 self.x_axes_list[i] * local_offset[0] + \
                                                 self.y_axes_list[i] * local_offset[1] + \
                                                 self.z_axes_list[i] * local_offset[2]
                        
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
            # m단위로 출력(오프셋을 바로 적용할 수 있게 수정)
            pos_mm = current_pos  #* 1000
            error_mm = min_error * 1000
            print(f"[{s+1}/{step_num}] 오프셋: ({pos_mm[0]:.6f}, {pos_mm[1]:.6f}, {pos_mm[2]:.6f}) m | 최소 오차: {error_mm:.4f} mm")
            
            error_avg += error_mm
            offset_list.append(pos_mm)
            error_list.append(error_mm)
    
        print("\n✅ 탐색 완료!")

        error_avg /= step_num
        
        if error_avg < self.accept_error:
            self.accept_data_list.append(self.probe_center_list, offset_list, error_list)

        #return error_avg, probe_center_list, offset_list, error_list

    def check_accept_data(self):
        #global accept_data_list
        print("\n" + "="*25 + " 최종 합격 데이터 " + "="*25)
        dist = 0.0
        if not self.accept_data_list:
            print("수집된 데이터가 없습니다.")
        else:
            # enumerate를 사용하면 인덱스 번호(i)와 항목(data)을 한번에 가져올 수 있습니다.
            for i, data in enumerate(self.accept_data_list):
                # data는 (offset, error) 형태의 튜플입니다.
                pose = data[0]
                offset = data[1]
                error = data[2]
    
                print(f"--- 데이터 [{i+1}/{len(self.accept_data_list)}] ---")
    
                for j in range(len(pose)):
                    dist += pose[j][2]
                    print(f"  center pose: {pose[j]}")
                    print(f"  offset: {offset[j]}")
                    print(f"  error: {error[j]:.4f} mm")
                dist /= 10
                dist *= 1000
                print(f"--- 데이터 [{i+1}/{len(self.accept_data_list)} 거리: {dist * -1:.1f}mm] ---")
                dist = 0.0
    
        print("="*64)