import pymurapi as mur
import cv2 as cv
from math import degrees, sqrt
import time

auv = mur.mur_init()

lower_yellow_circle = (14, 116, 95)
upper_yellow_circle = (56, 255, 255)

lower_blue_circle = (125, 144, 56)
upper_blue_circle = (146, 255, 255)

lower_green_circle = (65, 137, 95)
upper_green_circle = (88, 255, 255)

lower_red_circle = (0, 190, 32)
upper_red_circle = (16, 255, 255)

lower_orange_circle = (0, 0, 150)
upper_orange_circle = (104, 255, 255)

class PID:
    e = 0
    kp = 0
    ki = 0
    kd = 0
    tp = time.time()
    ip = 0
    ep = None
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd


    def apply(self, e):
        self.ip = self.I_regulator(e)
        d = 0
        if self.ep is not None:
            d = self.D_regulator(e)
        self.ep = e
        self.tp = time.time()
        time.sleep(0.00001)
        return self.P_regulator(e, self.kp) + self.ip + d

       
    def P_regulator(self, e, kp):
        return e * kp 

          
    def I_regulator(self, e):
        i = e * self.ki * (time.time() - self.tp) + self.ip
        return i
   

    def D_regulator(self, e):
        d = self.kd * (e - self.ep) / (time.time() - self.tp)
        return d   
            
class AutoPilot:
    yaw_pid = PID(1.15, 0, 0.25)
    depth_pid = PID(60, 0, 0)


    def regulate_yaw(self, yaw, speed):
        error_yaw = clamp_180(to_360(yaw - clamp_360(auv.get_yaw())))
        u_yaw = self.yaw_pid.apply(error_yaw)
        auv.set_motor_power(0, speed + u_yaw)
        auv.set_motor_power(1, speed - u_yaw)
        time.sleep(0.034)
        
        
    def regulate_depth(self, depth):
        error_depth = depth - auv.get_depth()
        u_depth = self.depth_pid.apply(error_depth)
        #print("error", error_depth, "power", u_depth)
        auv.set_motor_power(2, -u_depth)
        auv.set_motor_power(3, -u_depth)
        time.sleep(0.034)

r = AutoPilot()        
def clamp_360(yaw):
    if yaw < 0:
        yaw += 360
    return yaw


# > 0 and < 360
def to_360(yaw):
    return yaw % 360

 
#[0;360] -> [-180;180] 
def clamp_180(yaw):
    if yaw > 180:
        return yaw - 360
    return yaw
 
def clamp(v, min, max):
	if v < min:
		return min
	if v > max:
		return max
	return v   

def stopAllMotors():
    for i in range(5):
        auv.set_motor_power(i, 0)    
        time.sleep(0.032)
    time.sleep(0.032)
  

def length_line(x, y, x1, y1):
    return sqrt((x1 - x) ** 2 + (y1 - y) ** 2)  

def detect_hsv(color, camera):
    if camera == 0:
        img = auv.get_image_bottom().copy()
    else:
        img = auv.get_image_front().copy()
        
    img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    if color == 0:
        img = cv.inRange(img, lower_blue_circle, upper_blue_circle)
    elif color == 1:
        img = cv.inRange(img, lower_green_circle, upper_green_circle)
    elif color == 2:
        img = cv.inRange(img, lower_red_circle, upper_red_circle)
    elif color == 3:
        img = cv.inRange(img, lower_yellow_circle, upper_yellow_circle)
    elif color == 4:
        img = cv.inRange(img, lower_orange_circle, upper_orange_circle)
    cv.imshow("img_hsv", img)
    cv.waitKey(1)    
    #img = cv.GaussianBlur(img, (3, 3), 5)
    contour, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   
    time.sleep(0.034)
    return contour, hierarchy    
    
def detect_color(color):
    contour, hierarchy = detect_hsv(color, 0)
    #img = auv.get_image_bottom().copy()
    max_area = 0
    max_cnt = None
    for cnt in contour:
        if len(cnt) > 4:
            area = cv.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_cnt = cnt
    if max_cnt is not None:
        mass = cv.moments(max_cnt)
        x = int(mass["m10"] / (0.01 + mass["m00"]))
        y = int(mass["m01"] / (0.01 + mass["m00"]))
        #time.sleep(0.034)
        return True, (x, y), mass["m00"]
    #time.sleep(0.034)   
    return False, (None, None), max_area
    
def move_to_center(x, y):
    error_x = 170 - x
    error_y = 130 - y
    
    auv.set_motor_power(0, error_y * 0.2)
    auv.set_motor_power(1, error_y * 0.2)
    auv.set_motor_power(4, error_x * 0.2)
        
    time.sleep(0.032)
    return (error_x, error_y)    
    
def centrate(color, depth, mark):
    error_x = 255
    error_y = 255
    if mark == -1:
        while abs(error_x) > 5 or abs(error_y) > 5:
            r.regulate_depth(depth)
            find, (x, y), area = detect_color(color)
            if not find: return 0
            #print(x, y)
            if x is not None and y is not None:
                if 0 <= x <= 320 and 0 <= y < 240: 
                    error_x, error_y = move_to_center(x, y)       
    else:
        t = time.time()
        while time.time() - t < mark:
            r.regulate_depth(depth)
            find, (x, y), area = detect_color(color)
            #print(x, y)
            if x is not None and y is not None:
                if 0 <= x <= 320 and 0 <= y < 240: 
                    error_x, error_y = move_to_center(x, y)    
    stopAllMotors()    


def find_angle_arrow(color, depth):   
    
    t = time.time()
    while time.time() - t < 10:
        centrate(color, depth, -1)
    stopAllMotors()
    centrate(color, depth, -1)
    print(time.time() - t)  
    
    #contours, hierarchy = detect_hsv(color, 0)
    
    x_min, y_min, x_centre, y_centre = 0, 0, 255, 255  
    while abs(x_centre - x_min) > 2 or y_min > y_centre:
        r.regulate_depth(depth)
        img = auv.get_image_bottom().copy()
        contours, hierarchy = detect_hsv(color, 0)
        if len(contours) == 0: continue
        hull = cv.convexHull(contours[0])
        hull = cv.approxPolyDP(contours[0], 5, True)
            #for i in range(len(hull)):
            #    cv.circle(img, (hull[i][0][0], hull[i][0][1]), 3, (0, 255, 0), 3)
            #print(x_centre, x_min, y_centre, y_min)
        mass = cv.moments(contours[0])
        if mass["m00"] > 500:
            x_centre = int(mass["m10"] / (0.01 + mass["m00"]))
            y_centre = int(mass["m01"] / (0.01 + mass["m00"]))
        min_length = 100000000
        for j in range(len(hull)):
            x = hull[j][0][0]
            y = hull[j][0][1]
            length = sqrt((x - x_centre) ** 2 + (y - y_centre) ** 2)
            if length < min_length:
                min_length = length
                x_min = x
                y_min = y
        if min_length > 40: continue        
        #print("min length", min_length)    
        cv.circle(img, (x_centre, y_centre), 3, (0, 255, 0), 3)
        cv.circle(img, (x_min, y_min), 3, (0, 255, 0), 3) 
        cv.imshow("img", img)
        cv.waitKey(1) 
 
        if x_centre - x_min < 0:
            #print("a", clamp(x_centre - x_min, -100, -4))
            auv.set_motor_power(1, clamp(x_centre - x_min, -100, -4))
            auv.set_motor_power(0, clamp(x_min - x_centre, 4, 100))
        elif x_min - x_centre < 0: 
            #print("b", x_min - x_centre)
            auv.set_motor_power(0, clamp(x_min - x_centre, -100, -4))
            auv.set_motor_power(1, clamp(x_centre - x_min, 4, 100))
        else:  
            auv.set_motor_power(1, 10)
            auv.set_motor_power(0, -10)
        #stopAllMotors()     
            #if y_min > y_centre:
            #    t = time.time()
            #    while time.time() - t < 2:
            #        auv.set_motor_power(1, 20)
            #        auv.set_motor_power(0, -20) 
    stopAllMotors()    
    a = length_line(x_min, y_min, 160, 10)
    b = length_line(160, 120, 160, 10)
    c = length_line(x_min, y_min, 160, 120)
        #print((b ** 2 + c ** 2 - a ** 2) / 2 * b * c)
        #angle += (b ** 2 + c ** 2 - a ** 2) / max(0.00001, (2 * b * c))
        #k += 1
    return auv.get_yaw()

def grab_cube(color_platform, color_cube, yaw):
    auv.open_grabber()
    centrate(color_platform, 3, 7)
    find, (x, y), area = detect_color(color_cube)
    #print(find)
    while find:
        #print(find)
        find, (x, y), area = detect_color(color_cube)
        centrate(color_cube, auv.get_depth() + 0.25, -1)
    t = time.time()    
    while time.time() - t < 1:
        r.regulate_yaw(yaw, -10)
    print("take cube")
    stopAllMotors()
    auv.close_grabber()
    time.sleep(1)    
    #while auv.get_depth() <

def go_to_platform(color_arrow, color_platform, depth):
    t = time.time()
    while time.time() - t < 5:
        r.regulate_depth(depth)
    angle = find_angle_arrow(color_arrow, 3)
    print(angle)
    find, (x, y), area = detect_color(color_platform)
    while area < 8500:
        #print(find, area)
        find, (x, y), area = detect_color(color_platform)
        r.regulate_yaw(angle, 50)
        r.regulate_depth(3)
    stopAllMotors()    
    return angle  
    
if __name__ == "__main__":
    time.sleep(1)
    #centrate(4, 2.5)
    #time.sleep(500)
    angle = find_angle_arrow(4, 3)
    print(angle)
    find, (x, y), area = detect_color(0)
    while area < 8500:
        #print(find, area)
        find, (x, y), area = detect_color(0)
        r.regulate_yaw(angle, 50)
        r.regulate_depth(3)
    stopAllMotors()    
    print("see blue circle")    
    grab_cube(0, 3, angle)
    
    angle = find_angle_arrow(1, 3)
    print(angle)
    find, (x, y), area = detect_color(1)
    while area < 2000:
        #print(find, area)
        find, (x, y), area = detect_color(1)
        r.regulate_yaw(angle, 50)
        r.regulate_depth(2)
    stopAllMotors()    
    
    #go_to_platform(1, 1, 2.5)
    #print("a")    
    centrate(1, 2, -1) 
    auv.open_grabber()
    while True:
        auv.set_motor_power(2, 45)
        auv.set_motor_power(3, 45)

    
    