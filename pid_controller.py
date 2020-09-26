import rospy
import numpy
import sys
import tf
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from maths import sqrt, cos, sin, pi, tan2
class PID :
    def __init__(self ,kp , ki , kd , dt):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.curr_error = 0
        self.prev_error = 0
        self.curr_eroor_der =  0
        self.prev_error_der = 0
        self.control = 0
        self.dt = dt
        self.sum_error = 0
    def update_controller(self , curr_error , reset_prev=False):
        self.curr_error_der = (self.curr_error - self.prev_error)/(self.dt)
        prop_gain = self.kp * self.curr_error
        integral_gain = self.sum_error + self.ki * self.curr_error * self.dt
        self.sum_error = integral_gain
        a = 0.5 
        filtered = a * self.prev_error_der + (1-a) * self.curr_eroor_der
        deriv_gain = self.kd * filtered
        cntrol_x = prop_gain + integral_gain + deriv_gain
        self.control = cntrol_x
        self.prev_error = self.curr_error
        self.prev_error_der = self.curr_eroor_der
        return self.control 
