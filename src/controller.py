from control.pid import PIDController
from rocket import Rocket
from time import sleep
import numpy as np
import logging
from numpy import sin, cos
from numpy.linalg import inv
class Controller:
    
    def __init__(self, rocket: Rocket):
        self._rocket = rocket

        logging.info('Initializing PID Controllers...')

        MAX_TILT = 15*np.pi/180
        self._uLoop = PIDController(MAX_TILT, -MAX_TILT)
        self._vLoop = PIDController(MAX_TILT, -MAX_TILT)
        self._wLoop = PIDController(None, None)

        MAX_GIMBAL = 85 * np.pi/180

        self._dxLoop = PIDController(MAX_GIMBAL, -MAX_GIMBAL)
        self._dyLoop = PIDController(MAX_GIMBAL, -MAX_GIMBAL)
        self._dtLoop = PIDController(1, 0)

        self._pnLoop = PIDController(None, None)
        self._peLoop = PIDController(None, None)
        self._pdLoop = PIDController(None, None)

        u_gains = (-0.02, 0, 0.0005)
        v_gains = (0.02, 0, 0.0005)
        # w_gains = (0, 0, 0)

        dx_gains = (4, 0, -8)
        dy_gains = (4, 0, -8)
        dt_gains = (-0.5, -0.5, 0)

        pn_gains = (0.15, 0, -0.4)
        pe_gains = pn_gains
        pd_gains = (0, 0, 0)

        logging.info('Setting Gains...')

        self._pnLoop.set_gains(pn_gains)
        self._peLoop.set_gains(pe_gains)
        self._pdLoop.set_gains(pd_gains)

        self._uLoop.set_gains(u_gains)
        self._vLoop.set_gains(v_gains)
       #  self._wLoop.set_gains(w_gains)

        self._dxLoop.set_gains(dx_gains)
        self._dyLoop.set_gains(dy_gains)
        self._dtLoop.set_gains(dt_gains)
    
    def cmd_loop(self, delay: float = 0.1):

        self._dtLoop.command_output(0)



        counter = 0
        counter2 = 0
        landing = False

        while True:

            states = self._rocket.get_states()

            Pn, Pe, Pd = states[0:3]
            phi, theta, psi = states[6:9]
            p, q, r = states[9:12]

            R_b_w=np.array([
            [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)],
            [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)],
            [        -sin(theta),                              sin(phi)*cos(theta),                              cos(phi)*cos(theta)]])


            u, v, w = np.matmul(states[3:6], inv(R_b_w))

            h = -Pd
            dh = -w
            
            alt = h - self._rocket._com_height

            if landing:
                if alt <= 5:
                    self._dtLoop.command_output(5)
                elif alt <= 20:
                    self._dtLoop.command_output(5)
                elif alt <= 250:
                    self._dtLoop.command_output(20)
                else:
                    self._dtLoop.command_output(200)
            else:
                if alt < 100:
                    self._dtLoop.command_output(-100)

                else:
                    self._dtLoop.command_output(0)

                    n_coord = 100
                    e_coord = 100
                    tol = 1

                    self._pnLoop.command_output(n_coord)
                    self._peLoop.command_output(e_coord)

                    #logging.info(f'Pn: {Pn}, Pe: {Pe}')
                    if Pn > n_coord - tol and Pn < n_coord + tol and Pe > e_coord - tol and Pe < e_coord + tol:
                        landing = True


            delta_t = self._dtLoop.step(w, 0, delay)
            delta_x = self._dxLoop.step(theta, q, delay)
            delta_y = self._dyLoop.step(phi, p, delay)

            v_c = 10

            if counter >= 10:
                counter = 0

                theta_c = self._uLoop.step(u, 0, delay)
                phi_c = self._vLoop.step(v, 0, delay)

                self._dxLoop.command_output(theta_c)
                self._dyLoop.command_output(phi_c)
                

            if counter2 >= 20:
                u_c = self._pnLoop.step(Pn, u, delay)
                v_c = self._peLoop.step(Pe, v, delay)
                
                self._uLoop.command_output(u_c)
                self._vLoop.command_output(v_c)

            deltas = np.array([delta_x, delta_y, delta_t])
            self._rocket.set_deltas(deltas)




            counter += 1
            counter2 += 1
            sleep(delay)
