
import numpy as np
import logging

class PIDController():
    class ControlLoop():
        
        def __init__(self, uLimit, dLimit):
            logging.info('  PID loop initialized')
            self._P, self._I, self._D, self._error = 0, 0, 0, 0
            self._Kp, self._Ki, self._Kd = 0, 0, 0
            self._uLim, self._dLim = uLimit, dLimit
            self._target_val = 0

        def command_output(self, commanded_output):
            self._target_val = commanded_output
            # logging.info(f'Delta commanded: {self._target_val}')

        def step(self, current_val, val_rate, dt):
            limit1 = self._uLim
            limit2 = self._dLim
            
            kp = self._Kp
            ki = self._Ki
            kd = self._Kd
            
            error = self._target_val-current_val

            self._I = self._I+(dt/2)*(error+self._error)
            self._D = val_rate
            self._error = error
            
            u = kp*error+ki*self._I+kd*self._D
            u_sat = self._sat(u, limit1, limit2)

            if ki!=0:
                self._I = self._I + dt/ki*(u_sat-u)
                
            return u_sat

        def _sat(self, inn,up_limit,low_limit):
            if inn>up_limit and up_limit is not None:
                out=up_limit
            elif inn<low_limit and low_limit is not None:
                out=low_limit
            else:
                out=inn
            return out
        
        def set_gains(self, gains):
            self._Kp, self._Ki, self._Kd = gains
            logging.info(f' PID set with gains {gains}')
        
        def reset(self):
            self._P, self._I, self._D, self._error = 0, 0, 0, 0

        def reset_integrator(self):
            self._I = 0