import numpy as np

class Calibration:
    def __init__(self, offset):
        self.offset = offset
        print(f"setting value: {offset}")
        print("Calibration Initializing Completed.")
    # Probe Calibration
    def calculate_optimal_offset(self, prove_centers, x_axes, y_axes, z_axes):
        global accept_data_list
        init_offset = self.offset
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
