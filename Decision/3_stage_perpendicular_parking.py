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
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
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
    
    def calc_parking_path(self, ):
        is_parking = False
        car_init_pose = self.random_start()
        parking_coord = self.parking_pose()
        delta_y, parking_width, J = self.calc_delta_y(car_init_pose, parking_coord)
        R = self.calc_turning_radius(self.ackerman_steering)
        Ra = self.calc_Ra_radius(R)
        Rb = self.calc_Rb_radius(R)
        Rc = self.calc_Rc_radius(R)

        if delta_y >= Rc + self.overhang:
            e = -abs(Rc - delta_y)
            Wmin = self.car_width
        elif Rc <= delta_y < Rc + self.overhang:
            e = -abs(Rc - delta_y)
            Wmin = math.sqrt(pow(Ra, 2) - pow(e, 2) - Rc)
        elif delta_y < Rc:
            pass

        if parking_width >= Wmin:
            is_parking = True
        
        if is_parking:
            print("car init pose = ", car_init_pose)
            Jx, Jy = J[0], J[1]
            Dx, Dy = Jx + R, Jy + R
            Kx , Ky = Dx + self.l/2, Dy
            Gx, Gy = Jx, Jy - self.car_length/2 

        else:
            print("parking Fail")
            print("parking_width = ", parking_width)
            print("Wmin = ", Wmin)
            Kx , Ky = None, None
            Gx, Gy = None, None
        
        return Kx, Ky, Gx, Gy


