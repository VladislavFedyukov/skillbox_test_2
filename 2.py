from simple_pid import PID

class AngularVelocityPID:
    def __init__(self, kp, ki, kd):
        self.pid_roll = PID(kp, ki, kd)
        self.pid_pitch = PID(kp, ki, kd)
        self.pid_yaw = PID(kp, ki, kd)

    def calculate_outputs(self, desired_rates, actual_rates):
        roll_output = self.pid_roll(desired_rates[0] - actual_rates[0])
        pitch_output = self.pid_pitch(desired_rates[1] - actual_rates[1])
        yaw_output = self.pid_yaw(desired_rates[2] - actual_rates[2])
        return [roll_output, pitch_output, yaw_output]

class AngularPositionPID:
    def __init__(self, kp, ki, kd):
        self.pid_roll = PID(kp, ki, kd)
        self.pid_pitch = PID(kp, ki, kd)
        self.pid_yaw = PID(kp, ki, kd)

    def calculate_outputs(self, desired_angles, actual_angles):
        roll_output = self.pid_roll(desired_angles[0] - actual_angles[0])
        pitch_output = self.pid_pitch(desired_angles[1] - actual_angles[1])
        yaw_output = self.pid_yaw(desired_angles[2] - actual_angles[2])
        return [roll_output, pitch_output, yaw_output]