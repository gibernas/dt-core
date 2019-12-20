import rospy
import numpy as np


class LaneController:
    def __init__(self, parameters):
        self.parameters = parameters
        self.d_err = 0
        self.phi_err = 0
        self.d_I = 0
        self.phi_I = 0
        self.prev_d_err = 0
        self.prev_phi_err = 0
        self.fsm_state = None

    def update_parameters(self, parameters):
        self.parameters = parameters

    def compute_velocity(self, stop_line_distance):
        """Linearly decrease velocity if approaching a stop line."""
        # 60cm -> v_bar, 15cm -> v_bar/2
        d1, d2 = 0.8, 0.25
        a = self.parameters['~v_bar'] / (2 * (d1 - d2))
        b = self.parameters['~v_bar'] - a * d1
        v_new = a * stop_line_distance + b
        v = np.max([self.parameters['~v_bar'] / 2.0, np.min([self.parameters['~v_bar'], v_new])])
        return v

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec):

        if dt is not None:
            self.integrate_errors(d_err, phi_err, dt)

        self.d_I = self.adjust_integral(d_err, self.d_I, self.parameters['integral_bounds']['d'],
                                        self.parameters['d_resolution'])
        self.phi_I = self.adjust_integral(phi_err, self.phi_I, self.parameters['integral_bounds']['phi'],
                                          self.parameters['phi_resolution'])

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)

        # Scale the parameters linear such that their real value is at 0.22m/s
        omega = self.parameters['~k_d'] * (0.22 / self.parameters['~v_bar']) * self.d_err + \
            self.parameters['~k_theta'] * (0.22 / self.parameters['~v_bar']) * self.phi_err

        if not self.fsm_state == "SAFE_JOYSTICK_CONTROL":
            omega -= self.parameters['~k_Id'] * (0.22/self.parameters['~v_bar']) * self.d_I
            omega -= self.parameters['~k_Iphi'] * (0.22/self.parameters['~v_bar']) * self.phi_I

        # apply magic conversion factors
        omega = omega * self.parameters['~omega_to_rad_per_s']

        self.prev_d_err = d_err
        self.prev_phi_err = phi_err

        return omega

    def integrate_errors(self, d_err, phi_err, dt):
        self.d_I += d_err * dt
        self.phi_I += phi_err * dt

    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_err = 0

    @staticmethod
    def adjust_integral(error, integral, bounds, resolution):
        if integral > bounds['top']:
            integral = bounds['top']
        elif integral < bounds['bot']:
            integral = bounds['bot']
        elif abs(error) < resolution:
            integral = 0
        return integral


