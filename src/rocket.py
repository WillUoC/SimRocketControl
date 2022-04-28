import numpy as np
from numpy import sin, cos, tan
from scipy.spatial import ConvexHull
from scipy.integrate import solve_ivp
import math
class Rocket:
    _gravity = 9.81

    def __init__(self, dt):
        self._states = np.zeros(12)
        self._deltas = np.zeros(3)
        self._dt = dt


        MATRIX_TIME = 10  # s
        self._state_array = np.zeros((12, math.floor(MATRIX_TIME//dt)))


        self._com_height = 0
        self._mass = 549054*0.25  # kg -- Mass of Falcon 9 (fuel weight removed... approximately)

        self._radius = 10
        self._height = 150

        self._thrust = 5885000  # N

        cylinder_inertia = 1/4*self._mass*self._radius**2 + 1.0/12.0*self._mass*self._height**2
        cylinder_axis_inertia = 1/2*self._mass*self._radius**2

        self._inertias = np.array([cylinder_inertia, cylinder_inertia, cylinder_axis_inertia])
        self._vertices, self._simplices = self._generate_body_matrix()
    
    def _generate_body_matrix(self):
        def generate_points():
            height_resolution = 25
            diameter_pts = 6
            cylinder_height = 100
            cylinder_diameter = self._radius

            cone_height = self._height - cylinder_height

            com_ratio = 0.5

            
            com_height = self._height * com_ratio

            self._com_height = com_height
            points = np.zeros((1, 3))

            heights = np.arange(-com_height, self._height-com_height + height_resolution, height_resolution)

            for i in heights:
                if i > cylinder_diameter - com_height:
                    norm_height =  i - (cylinder_height-com_height)
                    diameter = cylinder_diameter*np.sqrt(cone_height*(cone_height-norm_height))/cone_height
                else:
                    diameter = cylinder_diameter

                # diameter =  cylinder_diameter - cylinder_diameter/cone_height * (i-(cylinder_height-com_height)) if i > cylinder_height - com_height else cylinder_diameter
                for j in range(diameter_pts):
                    new_point = np.zeros((1, 3))
                    new_point[0] = [diameter * np.cos(j*2*np.pi/diameter_pts), diameter * np.sin(j*2*np.pi/diameter_pts), i]
                    points = np.append(points, new_point, 0)
            
            points_new = np.zeros(np.shape(points))
            points_new[:, 0] = points[:, 1]
            points_new[:, 1] = points[:, 0]
            points_new[:, 2] = -points[:, 2]

            return(points_new)

        def convex_hull(points):
            hull = ConvexHull(points)
            for s in hull.simplices:
                s = np.append(s, s[0])

            return hull.simplices
        points = generate_points()
        hull = convex_hull(points)
        return(points, hull)

    def _rotate_matrix(self, angles):
        phi, theta, psi = angles

        # Rotational array based off work done in class
        R_b_w=np.array([
            [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)],
            [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)],
            [        -sin(theta),                              sin(phi)*cos(theta),                              cos(phi)*cos(theta)]])

        return R_b_w
    
    def set_state(self, states):
        self._states = states

    def set_deltas(self, deltas):
        self._deltas = deltas

    def get_pos(self):
        R_plot = np.array([ 
                    [0,  1,  0],
                    [1,  0,  0],
                    [0,  0, -1]])   
        com = np.matmul(R_plot, self._states[0:3])
        return(com)

    def _update_state_array(self):
        self._state_array = self._state_array[:, 1:]
        reshape_arr = np.reshape(self._states, (-1,1))
        self._state_array = np.append(self._state_array, reshape_arr, 1)

    def get_states(self):
        return(self._states)

    def get_state_array(self):
        return(self._state_array)
    
    def _compute_forces_moments(self, states, deltas):
        distance_vector = np.array([0, 0, self._com_height])

        deltax, deltay, deltat = deltas
        phi, theta, psi = states[6:9]

        thrust_vector = deltat * self._thrust * np.array([sin(deltax), sin(-deltay)*cos(deltax), -cos(-deltay)*cos(deltax)])

        # R_plot = np.array([ 
        #             [1,  0,  0],
        #             [0,  1,  0],
        #             [0,  0, -1]])

        # thrust_vector = np.matmul(R_plot, thrust_vector)

       
        gravity_force = np.array([
        -self._mass*self._gravity*sin(theta),
        self._mass*self._gravity*cos(theta)*sin(phi),
        self._mass*self._gravity*cos(theta)*cos(phi)
        ])

        forces = thrust_vector + gravity_force

        moments = np.cross(distance_vector, thrust_vector)

        return(forces, moments)
        
# __________████████_____██████
# _________█░░░░░░░░██_██░░░░░░█
# ________█░░░░░░░░░░░█░░░░░░░░░█
# _______█░░░░░░░███░░░█░░░░░░░░░█
# _______█░░░░███░░░███░█░░░████░█
# ______█░░░██░░░░░░░░███░██░░░░██
# _____█░░░░░░░░░░░░░░░░░█░░░░░░░░███
# ____█░░░░░░░░░░░░░██████░░░░░████░░█
# ____█░░░░░░░░░█████░░░████░░██░░██░░█
# ___██░░░░░░░███░░░░░░░░░░█░░░░░░░░███
# __█░░░░░░░░░░░░░░█████████░░█████████
# _█░░░░░░░░░░█████_████___████_█████___█
# _█░░░░░░░░░░█______█_███__█_____███_█___█
# █░░░░░░░░░░░░█___████_████____██_██████
# ░░░░░░░░░░░░░█████████░░░████████░░░█
# ░░░░░░░░░░░░░░░░█░░░░░█░░░░░░░░░░░░█
# ░░░░░░░░░░░░░░░░░░░░██░░░░█░░░░░░██
# ░░░░░░░░░░░░░░░░░░██░░░░░░░███████
# ░░░░░░░░░░░░░░░░██░░░░░░░░░░█░░░░░█
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█
# ░░░░░░░░░░░█████████░░░░░░░░░░░░░░██
# ░░░░░░░░░░█▒▒▒▒▒▒▒▒███████████████▒▒█
# ░░░░░░░░░█▒▒███████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█
# ░░░░░░░░░█▒▒▒▒▒▒▒▒▒█████████████████
# ░░░░░░░░░░████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█
# ░░░░░░░░░░░░░░░░░░██████████████████
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░█
# ██░░░░░░░░░░░░░░░░░░░░░░░░░░░██
# ▓██░░░░░░░░░░░░░░░░░░░░░░░░██
# ▓▓▓███░░░░░░░░░░░░░░░░░░░░█
# ▓▓▓▓▓▓███░░░░░░░░░░░░░░░██
# ▓▓▓▓▓▓▓▓▓███████████████▓▓█
# ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓██
# ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓█
# ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓█

    def dynamics(self, t, x, states, deltas):
        def _rotate_dyn_matrix(moments, roll_rates):
            p, q, r = roll_rates
            M1 = moments - np.array([
                q*r*(self._inertias[2] - self._inertias[1]), 
                r*p*(self._inertias[0] - self._inertias[2]), 
                p*q*(self._inertias[1] - self._inertias[0])])

            roll_ddot = np.divide(M1, self._inertias)
            return(roll_ddot)

        def _rotate_kin_matrix(angles, roll_rates):
            # theta, phi, psi = angles
            # p, q, r = roll_rates

            # ang_dot = np.array([p + (q*np.sin(phi) + r*np.cos(phi))*np.tan(theta), q*np.cos(phi)-r*np.sin(phi), 0])
            
            phi, theta, psi = angles
            p, q, r = roll_rates

            # Matrix derived in class
            M=np.array([
                [ 1, sin(phi)*tan(theta),  cos(phi)*tan(theta)],
                [ 0,            cos(phi),            -sin(phi)],
                [ 0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
    
    
            # Multiplying the derived matrix with the roll rates returns the derivative of the angles.
            angdot_M=np.array([p, q, r])
            ang_dot = np.matmul(M, angdot_M)
            return(ang_dot)
                    
        def _translate_dyn_matrix(velocities, roll_rates, forces):
            u, v, w = velocities
            p, q, r = roll_rates

            vel_dot = forces/self._mass - np.array([q*w-r*v, r*u-p*w, p*v-q*u])
            return(vel_dot)

        def _translate_kin_matrix(angles, velocities):
            phi, theta, psi = angles
            u, v, w = velocities


            # Rotational array also based off work done in class
            R_b_w=np.array([
                [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)],
                [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)],
                [        -sin(theta),                              sin(phi)*cos(theta),                              cos(phi)*cos(theta)]])
            
            # Multiply current velocities by rotational array to find the derivative array of ground position
            speed =  np.array([u, v, w])
            pos_ned_dot = np.matmul(R_b_w, speed)

            return pos_ned_dot

        velocities = states[3:6]
        angles = states[6:9]
        roll_rates = states[9:12]

        forces, moments = self._compute_forces_moments(states, deltas)


        roll_rate_dot = _rotate_dyn_matrix(moments, roll_rates)
        angle_dot = _rotate_kin_matrix(angles, roll_rates)
        vel_dot = _translate_dyn_matrix(velocities, roll_rates, forces)
        pos_dot = _translate_kin_matrix(angles, velocities)

        states_dot = np.concatenate((pos_dot, vel_dot, angle_dot, roll_rate_dot))

        return(states_dot)

    def get_vertices(self):
        updated_states = solve_ivp(lambda t, y: self.dynamics(t, y, self._states, self._deltas), [0, self._dt], self._states)
        updated_states = updated_states.y[:, -1].T

        if updated_states[2] > -self._com_height:
            updated_states[2] = -self._com_height
            updated_states[5] = 0
        self._states = updated_states

        pos_ned = self._states[0:3]

        angles = self._states[6:9]

        v = self._vertices                                      # Get the default body frame
        ned_rep = np.tile(pos_ned.T, (len(v), 1))               # Create MxN Matric Copies of pos_ned for Translation

        R = self._rotate_matrix(angles)                         # Get the rotation matrix for the current state of the aircraft
        vr = np.matmul(R, v.T).T                                # Multiply the body frame by the rotation matrix to get the rotated vertex position
        vr = vr + ned_rep
        R_plot = np.array([ [0,  1,  0],
                            [1,  0,  0],
                            [0,  0, -1]])                       # Reference frame shift for NWU plotting   

        vr = np.matmul(R_plot, vr.T).T                          # Refactor the vertex array for NWU plotting
        self._update_state_array()

        return(vr, self._simplices)