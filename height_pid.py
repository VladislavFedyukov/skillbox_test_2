from simple_pid import PID

class HeightPID:
    
    def __init__(self, kp_pos, ki_pos, kd_pos, kp_vel, ki_vel, kd_vel):
        self.pid_position = PID(kp_pos, ki_pos, kd_pos)
        self.pid_velocity = PID(kp_vel, ki_vel, kd_vel)

    def calculate_outputs(self, desired_z, desired_w, actual_z, actual_w):
        position_output = self.pid_position(desired_z - actual_z)
        velocity_output = self.pid_velocity(desired_w - actual_w + position_output)
        return velocity_output


if __name__ == '__main__':
    # Создаем экземпляр класса HeightPID с определенными коэффициентами
    my_pid = HeightPID(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)

    # Задаем желаемые и актуальные значения для высоты (z) и скорости (w)
    desired_z = 10.0
    actual_z = 5.0
    desired_w = 0.0
    actual_w = 2.0

    # Расчет выходного значения
    output = my_pid.calculate_outputs(desired_z, desired_w, actual_z, actual_w)

    # Выводим результат
    print(f'Output: {output}')
