import matplotlib.pyplot as plt
import rospy
# import tf
# from nav_msgs.msg import Odometry
# from thesis.msg import Autonomous_Game
from marvelmind_nav.msg import hedge_pos_a
# from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from shapely.geometry import Polygon
# from RobustDecisionMaking import get_params
# 
# params = get_params.get_params()

# roadbound coordinate
# upper bound
# o1 = [[125, 121.4],
#         [125, 122.4],
#         [230, 122.4],
#         [230, 121.4]]
# # lower bound
# o2 = [[125, 114],
#         [125, 115],
#         [230, 115],
#         [230, 114]]
# obs = [o1,o2]

# x_br = params.x_br
# y_br = params.y_br

# x1,y1 = Polygon(obs[0]).exterior.xy
# x2,y2 = Polygon(obs[1]).exterior.xy

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()

        self.mb1, = plt.plot([], [], "o", c="g")
        self.mb2, = plt.plot([], [], "o", c="y")
        self.st1, = plt.plot([], [], "o", c="r")
        self.st2, = plt.plot([], [], "o", c="b")
        self.st3, = plt.plot([], [], "o", c="k")


        self.oth, = plt.plot([], [], "o", c="g")
        self.waypoint, = plt.plot([],[], "--")
        
        self.x_data1, self.y_data1 = [] , []
        self.x_data2, self.y_data2 = [] , []
        self.xstation1, self.ystation1 = [2.467] , [0]
        self.xstation2, self.ystation2 = [0] , [0]
        self.xstation3, self.ystation3 = [0.019] , [-2.047]

        self.x_other, self.y_other = 225.8 , 116.6
        self.yaw_data = 3.14
        self.AV_sz, = plt.plot([], [])
        self.oth_sz, = plt.plot([], [])
        self.BR_sz, = plt.plot([], [])
        self.x_wp, self.y_wp = [], []
        self.upp_obstacle, = plt.plot([], [])
        self.low_obstacle, = plt.plot([], [])
    
    def create_vehicle_safe_zone(self, x, y, yaw):
        X_others = x #self.x_data[-1]
        Y_others = y #self.y_data[-1]
        yaw_others = yaw #self.yaw_data
        # print(type(X_others),type(Y_others),type(yaw_others))
        l_car = 5
        w_car = 2
        l_car_safe = l_car #1.1 * l_car
        w_car_safe = w_car #1.25 * w_car

        safe_zone = [[X_others-l_car_safe/2*np.cos(yaw_others)+w_car_safe/2*np.sin(yaw_others),
                Y_others-l_car_safe/2*np.sin(yaw_others)-w_car_safe/2*np.cos(yaw_others)],
                [X_others-l_car_safe/2*np.cos(yaw_others)-w_car_safe/2*np.sin(yaw_others),
                Y_others-l_car_safe/2*np.sin(yaw_others)+w_car_safe/2*np.cos(yaw_others)],
                [X_others+l_car_safe/2*np.cos(yaw_others)-w_car_safe/2*np.sin(yaw_others),
                Y_others+l_car_safe/2*np.sin(yaw_others)+w_car_safe/2*np.cos(yaw_others)],
                [X_others+l_car_safe/2*np.cos(yaw_others)+w_car_safe/2*np.sin(yaw_others),
                Y_others+l_car_safe/2*np.sin(yaw_others)-w_car_safe/2*np.cos(yaw_others)]]
        return safe_zone

    def plot_init(self):
        pass
        self.ax.set_xlim(-2, 3)
        self.ax.set_ylim(-3, 1)
        # self.ax.axis("square")
        # return self.ln
    
    # def getYaw(self, pose):
    #     quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
    #             pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     yaw = euler[2] 
    #     return yaw   

    def pos_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        # if msg.actual_y < 100: 
        #     self.x_data.append(self.x_data[-1])
        #     self.y_data.append(self.y_data[-1])
        # else:
        #     self.x_data.append(msg.actual_x)
        #     self.y_data.append(msg.actual_y)
        # if msg.error_yaw < 100: 
        #     self.x_other = self.x_other
        #     self.y_other = self.y_other
        # else:
        #     self.x_other = msg.error_lateral
        #     self.y_other = msg.error_yaw
        # self.x_wp = msg.potential_x
        # self.y_wp = msg.potential_y
        # self.yaw_data = msg.target_brake
        # self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # self.x_data.append(x_index+1)
        # self.x_data.append(float(msg.x_m))
        # self.y_data.append(float(msg.y_m))
        if (msg.address == 4):
            self.x_data1 = [msg.x_m]
            self.y_data1 = [msg.y_m]
        else: 
            self.x_data2 = [msg.x_m]
            self.y_data2 = [msg.y_m]
    
    def update_plot(self, frame):
        # self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        self.mb1.set_data(self.x_data1, self.y_data1)
        self.mb2.set_data(self.x_data2, self.y_data2)
        self.st1.set_data(self.xstation1, self.ystation1)
        self.st2.set_data(self.xstation2, self.ystation2)
        self.st3.set_data(self.xstation3, self.ystation3)
        # print("receiving?")
        # print(self.x_data)
        # print(self.y_data)
        # self.oth.set_data(self.x_other, self.y_other)
        # self.waypoint.set_data(self.x_wp, self.y_wp)
        # self.upp_obstacle.set_data(x1, y1)
        # self.low_obstacle.set_data(x2, y2)
        # AV_br = vis.create_vehicle_safe_zone(self.x_data[-1], self.y_data[-1], self.yaw_data)
        # x3,y3 = Polygon(AV_br).exterior.xy
        # self.AV_sz.set_data(x3,y3)
        # oth_br = vis.create_vehicle_safe_zone(self.x_other, self.y_other, 3.14)
        # x4,y4 = Polygon(oth_br).exterior.xy
        # self.oth_sz.set_data(x4,y4)
        # BR_br = vis.create_vehicle_safe_zone(x_br, y_br, 3.14)
        # x5,y5 = Polygon(BR_br).exterior.xy
        # self.BR_sz.set_data(x5,y5)
        # # self.set, = plt.axis("equal")
        # # self.ax.set_xlim(120, 235)
        # # self.ax.set_ylim(110, 130)
        # if len(self.x_data)>500 : 
        #     self.x_data = list(np.array(self.x_data)[400:])
        #     self.y_data = list(np.array(self.y_data)[400:])
        return self.mb1, self.mb2, self.st1, self.st2, self.st3 #, self.waypoint, self.upp_obstacle, self.low_obstacle, self.oth, self.AV_sz, self.oth_sz, self.BR_sz


rospy.init_node('visualization')
vis = Visualiser()
# sub = rospy.Subscriber('/dji_sdk/odometry', Odometry, vis.odom_callback)
# sub = rospy.Subscriber('/game_theory_AV', Autonomous_Game, vis.pos_callback)
sub = rospy.Subscriber('/hedge_pos_a', hedge_pos_a, vis.pos_callback)
# rospy.spin()

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 

# while True : 
#     ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)


