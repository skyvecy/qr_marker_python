from dataclasses import dataclass

@dataclass
class MarkerData:
    id: int
    
    x_pos: float
    y_pos: float
    z_pos: float
    
    x_axis_x: float
    x_axis_y: float
    x_axis_z: float
    
    y_axis_x: float
    y_axis_y: float
    y_axis_z: float
    
    z_axis_x: float
    z_axis_y: float
    z_axis_z: float

    def set_data(self, marker_id, marker_pos, marker_r):
        self.id = marker_id
        
        self.x_pos = marker_pos[0]
        self.y_pos = marker_pos[1]
        self.z_pos = marker_pos[2]
        self.x_axis_x = marker_r[:, 0][0]
        self.x_axis_y = marker_r[:, 0][1]
        self.x_axis_z = marker_r[:, 0][2]
        self.y_axis_x = marker_r[:, 1][0]
        self.y_axis_y = marker_r[:, 1][1]
        self.y_axis_z = marker_r[:, 1][2]
        self.z_axis_x = marker_r[:, 2][0]
        self.z_axis_y = marker_r[:, 2][1]
        self.z_axis_z = marker_r[:, 2][2]
