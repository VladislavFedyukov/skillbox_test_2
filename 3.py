from pid import PID
class HeightPID:
    def __init__(self, kp_pos, ki_pos, kd_pos, kp_vel, ki_vel, kd_vel):
        self.pid_position = PID(kp_pos, ki_pos, kd_pos)
        self.pid_velocity = PID(kp_vel, ki_vel, kd_vel)

    def calculate_outputs(self, desired_z, desired_w, actual_z, actual_w):
        position_output = self.pid_position(desired_z - actual_z)
        velocity_output = self.pid_velocity(desired_w - actual_w + position_output)
        return velocity_output

       