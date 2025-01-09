import numpy as np
import math

class Environment:
    def __init__(self, ):
        self.margin = 5
        #coordinates are in [x,y] format
        self.car_length = 80
        self.car_width = 40
        self.wheel_length = 15
        self.wheel_width = 7
        self.overhang = 15
        self.l = 50 # wheelbase
        self.ackerman_steering = 36.5 # maximum ackerman deg
        self.wheel_positions = np.array([[25,15],[25,-15],[-25,15],[-25,-15]])
        
        self.color = np.array([0,0,255])/255
        self.wheel_color = np.array([20,20,20])/255

        self.car_struct = np.array([[+self.car_length/2, +self.car_width/2],
                                    [+self.car_length/2, -self.car_width/2],  
                                    [-self.car_length/2, -self.car_width/2],
                                    [-self.car_length/2, +self.car_width/2]], 
                                    np.int32)
        
        self.wheel_struct = np.array([[+self.wheel_length/2, +self.wheel_width/2],
                                      [+self.wheel_length/2, -self.wheel_width/2],  
                                      [-self.wheel_length/2, -self.wheel_width/2],
                                      [-self.wheel_length/2, +self.wheel_width/2]], 
                                      np.int32)
        
        self.wall_coords = np.array([
            [-400, 200], [400, 200],  # 1. (-400, 200) to (400, 200)
            [-400, -200], [-30, -200],  # 2. (-400, -200) to (-30, -200)
            [30, -200], [400, -200],  # 3. (30, -200) to (400, -200)
            [-30, -200], [-30, -300],  # 4. (-30, -200) to (-30, -300)
            [-30, -300], [30, -300],  # 5. (-30, -300) to (30, -300)
            [30, -300], [30, -200]  # 6. (30, -300) to (30, -200)
        ], np.int32)

    def rotate_car(self, pts, angle=0):
        R = np.array([[math.cos(angle), -math.sin(angle)],
                    [math.sin(angle),  math.cos(angle)]])
        return ((R @ pts.T).T).astype(int)
    
    def random_start(self, ):
        x = np.random.randint(-380, -150)
        y = np.random.randint(-100, 150)
        return np.array([x, y])
    
    def calc_delta_y(self, car_init_pose, parking_coord):
        distance_list = []
        for i, parking_pose in enumerate(parking_coord):
            cx, cy = car_init_pose[0], car_init_pose[1]
            px, py = parking_pose[0], parking_pose[1]
            dis = math.sqrt(pow((px -cx), 2) + pow((py - cy), 2))
            distance_list.append([px, py, dis])
        distance_list.sort(key=lambda x: x[2])
        x1, y1 = distance_list[0][0], distance_list[0][1]
        x2, y2 = distance_list[1][0], distance_list[1][1]
        delta_y = abs(y2 - car_init_pose[1])
        width = abs(x2 - x1)
        J = np.array([(x2+x1)/2, y1])

        return delta_y, width, J 
    
    def calc_turning_radius(self, ackerman_steering):
        radius = self.l/math.tan(math.radians(ackerman_steering))
        return radius
    
    def calc_Ra_radius(self, R):
        Ra = math.sqrt(pow((R + self.car_width/2),2) + pow(self.overhang, 2))
        return Ra
    
    def calc_Rb_radius(self, R):
        Rb = math.sqrt(pow((R + self.car_width/2),2) + pow(self.l + self.overhang, 2))
        return Rb
    
    def calc_Rc_radius(self, R):
        Rc = R - self.car_width/2
        return Rc

    def parking_pose(self,):
        return [-30, -200], [-30, -300], [30, -300], [30, -200]

    def calc_delta_x(self, R, theta):
        delta_x = R*math.tan(theta/2)
        return delta_x

    def calc_delta_s(self, R, theta):
        delta_s = R*(math.tan((math.pi/4 - theta/2)) - math.tan(theta/2))
        return delta_s
    
    def calc_parking_path(self, ):
        is_parking = False
        car_init_pose = self.random_start()
        parking_coord = self.parking_pose()
        delta_y, parking_width, J = self.calc_delta_y(car_init_pose, parking_coord)
        R = self.calc_turning_radius(self.ackerman_steering)
        Ra = self.calc_Ra_radius(R)
        Rb = self.calc_Rb_radius(R)
        Rc = self.calc_Rc_radius(R)
        delta_y = R*0.7

        O = np.array([J[0], J[1] + delta_y]) 
        O2 = np.array([J[0] + R, J[1]])
        theta_candidate = []
        for theta in range(0, 90):
            radians = math.radians(theta)
            delta_x = self.calc_delta_x(R, radians)
            delta_s = self.calc_delta_s(R, radians)
            S1 = np.array([O[0] - delta_x, O[1]])
            S2 = np.array([O[0] + delta_s*math.cos(radians), O[1] + delta_s*math.sin(radians)])
            dis = math.sqrt(pow(O2[0] - S2[0],2) + pow(O[1] - S1[1],2))

            car_front = np.array([S2[0] + self.car_length/2, S2[1] + self.car_width/2])
            if R*0.95 < dis < R*1.05: # 접선 판단
                if O[0] < S2[0] and O[1] < S2[1]:
                    theta_candidate.append([theta, dis, S1, S2])
                    # print("S2 = ", S2)
                    # print("car_front = ", car_front)
                    # print("회전 = ", self.rotate_car(car_front, -radians))
                    # print("theta =", theta)
                    # print("----------------")

        waypoint_1 = theta_candidate[0][2]
        waypoint_2 = O
        waypoint_3 = theta_candidate[0][3] # 차량 중심이 아니고 뒷바퀴 중심으로 다시 구해야 해
        waypoint_4 = np.array([J[0], J[1] - self.car_length/2])

        print("waypoint_1 =", waypoint_1)
        print("waypoint_2 =", waypoint_2)
        print("waypoint_3 =", waypoint_3)
        print("waypoint_4 =", waypoint_4)

env = Environment()
env.calc_parking_path()