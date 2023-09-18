from math import*
class MathModelQuadrotor:
    def __init__(self, Ix, Iy, Iz, m, L, b, k):
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        self.m = m
        self.L = L
        self.b = b
        self.k = k
        

    def dynamics(self, state, control_input):
        # Unpack state and control input
        x, y, z, phi, theta, psi, u, v, w, p, q, r = state
        u1, u2, u3, u4 = control_input

        # Calculate motor forces
        f1 = self.k * u1
        f2 = self.k * u2
        f3 = self.k * u3
        f4 = self.k * u4
        
        # Calculate total force in the body frame
        fb_x = 0
        fb_y = 0
        fb_z = -(f1 + f2 + f3 + f4)
        
        # Calculate torques
        tau_phi = self.L * (f1 - f3)
        tau_theta = self.L * (f2 - f4)
        tau_psi = self.b * (u1 - u2 + u3 - u4)
        
        # Dynamics equations
        g=9.8
        dx = (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * fb_z / self.m
        dy = (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * fb_z / self.m
        dz = -u * sin(phi) * cos(theta) - v * cos(phi) * sin(theta) + w * cos(phi) * cos(theta) - g / self.m
        dphi = p + cos(phi) * tan(theta) * q + sin(phi) * tan(theta) * r
        dtheta = -sin(phi) * q + cos(phi) * r
        dpsi = (cos(phi) / cos(theta)) * q + (-sin(phi) / cos(theta)) * r
        du = r * v - q * w - g * sin(theta)
        dv = p * w - r * u + g * sin(phi) * cos(theta)
        dw = q * u - p * v + (1 / self.m) * fb_z - g * cos(phi) * cos(theta)
        dp = (1 / self.Ix) * (tau_phi - (self.Iy - self.Iz) * q * r)
        dq = (1 / self.Iy) * (tau_theta - (self.Iz - self.Ix) * p * r)
        dr = (1 / self.Iz) * (tau_psi - (self.Ix - self.Iy) * p * q)

        return [dx, dy, dz, dphi, dtheta, dpsi, du, dv, dw, dp, dq, dr]
    
def main():
    # Parameters (Пример значений параметров)
    Ix = 0.1
    Iy = 0.1
    Iz = 0.1
    m = 1
    L = 0.1
    b = 0.1
    k = 0.5

    # Создаем объект класса MathModelQuadrotor
    quadrotor = MathModelQuadrotor(Ix, Iy, Iz, m, L, b, k)
    
    # Начальные условия состояния (x, y, z, phi, theta, psi, u, v, w, p, q, r)
    initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    # Управляющие входы (u1, u2, u3, u4)
    control_input = [0, 0, 0, 0]

    # Вызываем функцию dynamics и выводим результат
    result = quadrotor.dynamics(initial_state, control_input)
    print(result)


if __name__ == "__main__":
    main()  
   
   
   
   
   
   
   