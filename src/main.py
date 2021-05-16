#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import numpy as np
import time
import math
from math import pi

# Importovanje potrebnih struktura poruka
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

rospy.init_node('ekstrakcija_linija', anonymous=False)

vel = Twist()
pub_velocities = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=0)

# Promenljiva za cuvanje tacaka
selected_algorithm = None

# Funkcija za formiranje markera u RVIZ-u
def make_marker(points):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "linije"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    
    marker.lifetime = rospy.Duration()

    # marker scale
    marker.scale.x = 0.2

    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []

    # points za RVIZ
    for i in range(len(points)):
        p = Point()
        p.x, p.y, p.z = points[i][0], points[i][1], 0
        marker.points.append(p)

    return marker

# Funkcija za rekurzivni/iterativni Split and Merge algoritam
def split_and_merge(points, selected_algorithm, threshold_split = 0.5, threshold_merge = 0.2):

    result = []
    marker_points = []

    # Split deo 
    if selected_algorithm == 'rec':
        line_parameters_with_points = split_rec(points, threshold_split = 0.05)
    else: 
        line_parameters_with_points = split_iter(points, threshold_split = 0.5)

    # Merge deo 
    merge_result, none_merged = merge(line_parameters_with_points, threshold_merge)

    while none_merged != False:
        merge_result, none_merged = merge(merge_result, threshold_merge)
    
    for line_segment in merge_result:
        result.append([line_segment[0], line_segment[1]])
        marker_points.append(line_segment[2][0][0:2]) 
        marker_points.append(line_segment[2][-1][0:2])

    return result, marker_points

# Funkcija za merge-ovanje tacaka koje potencijalno pripadaju
# jednoj istoj liniji 
def merge(line_parameters_with_points, threshold_merge, r_tolerancy = 0.2, abs_angle_tolerancy = 5*pi/180):

    result = []

    isMerged = False
    lastMerged = False

    num_segments = len(line_parameters_with_points)

    if num_segments == 1:

        result = line_parameters_with_points
        return result, False

    for i in range(1, num_segments):

        if lastMerged:
            r1 = result[-1][0]
            alpha1 = result[-1][1]
            points_1 = result[-1][2]
        else:
            r1 = line_parameters_with_points[i-1][0] 
            alpha1 = line_parameters_with_points[i-1][1]
            points_1 = line_parameters_with_points[i-1][2]

        r2 = line_parameters_with_points[i][0] 
        alpha2 = line_parameters_with_points[i][1] 
        points_2 = line_parameters_with_points[i][2]

        # colinearity = (np.abs(alpha1 - alpha2) <= abs_angle_tolerancy)*(np.abs(r1-r2) <= r_tolerancy)
        colinearity = (np.abs(alpha1 - alpha2) <= abs_angle_tolerancy)

        # Provera kolineranosti 
        if colinearity == True:

            # Ukoliko su dva segmenta kolinearna 
            # uzimamo prosecne vrednosti parametara linija
            r_avg = (r1 + r2)/2
            alpha_avg = (alpha1 + alpha2)/2

            # Pronalazenje najudaljenije tacke 

            _, max_distance_1 = find_most_distant_point(points_1, r_avg, alpha_avg)
            _, max_distance_2 = find_most_distant_point(points_2, r_avg, alpha_avg)

            # Ukoliko su dve linije bliske jedna drugoj, merge-uju se
            if max(max_distance_1, max_distance_2) <= threshold_merge:
                if lastMerged:
                    result[-1][0] = r_avg
                    result[-1][1] = alpha_avg
                    result[-1][2] += points_2
                else:
                    result.append([r_avg, alpha_avg, points_1 + points_2])
                
                isMerged = True
                lastMerged = True

            else:
                result.append(line_parameters_with_points[i-1])
                lastMerged = False

        else:
            result.append(line_parameters_with_points[i-1])
            lastMerged = False

    if not lastMerged:
        result.append(line_parameters_with_points[-1])

    return result, isMerged

# Funkcija za iterativno splitovanje podataka, ondosno tacaka 
# na one koje pripadaju linijama posebnim
def split_iter(points, threshold_split):

    line_parameters_with_points = []

    rest_of_points = []

    while True:
        num_points = len(points)

        # Fitovanje linije na celom trenutnom setu tacaka
        [r, alpha] = fit_line(points,'iter')

        if num_points <= 2:
            line_parameters_with_points.append([r, alpha, points])

            if not rest_of_points:
                break
            else:     
                points = rest_of_points.pop()
                continue

        # Pronalazenje najudaljenije tacke 
        most_distant_point_index, max_distance = find_most_distant_point(points, r, alpha)

        # Ponovno splitovanje
        if (max_distance > threshold_split) and (most_distant_point_index not in [0, num_points-1]):

            rest_of_points.append(points[most_distant_point_index:])
            points = points[0:most_distant_point_index+1] 

        else:
            line_parameters_with_points.append([r, alpha, points])
            if not rest_of_points:
                break
            else:     
                points = rest_of_points.pop()
                continue

    return line_parameters_with_points

# Funkcija za rekurzivno splitovanje podataka, ondosno tacaka 
# na one koje pripadaju linijama posebnim
def split_rec(points, threshold_split):

    line_parameters_with_points = []

    num_points = len(points)

    # Fitovanje linije na celom trenutnom setu tacaka
    [r, alpha] = fit_line(points,'rec')

    if num_points <= 2:
        line_parameters_with_points.append([r, alpha, points])
        return line_parameters_with_points

    # Pronalazenje najudaljenije tacke 
    most_distant_point_index, max_distance = find_most_distant_point(points, r, alpha)

    # Ponovno splitovanje
    if ((max_distance > threshold_split) and (most_distant_point_index not in [0, num_points-1])):

        left_points = points[0:most_distant_point_index+1] 
        right_points = points[most_distant_point_index:]

        if(left_points):
            left = split_rec(left_points, threshold_split)
            for item in left:
                line_parameters_with_points.append(item)

        if(right_points):
            right = split_rec(right_points, threshold_split)
            for item in right:
                line_parameters_with_points.append(item)
    else:
        
        line_parameters_with_points.append([r, alpha, points])

    return line_parameters_with_points

# Funkcija za pronalazenje najudaljenije tacke od trenutne
# fitovane linije
def find_most_distant_point(points, r, alpha):

    if alpha < 0:
        alpha = 2*pi + alpha 

    x1 = r*math.cos(alpha)
    y1 = r*math.sin(alpha)

    if alpha == 0 or alpha == pi:
        x2 = x1
        y2 = y1 + 0.1
    elif alpha == pi/2 or alpha == 3*pi/2:
        x2 = x1 + 0.1
        y2 = y1
    else:
        k = math.tan(pi/2 + alpha)
        n = r/math.sin(alpha)

        x2 = x1 + 0.1
        y2 = k*x2 + n

    num_points = len(points)
    # print(num_points)

    # Pronalazenje najudaljenije tacke 
    max_distance = 0

    for i in range(num_points):
        x0 = points[i][0]
        y0 = points[i][1]

        distance = np.abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/np.sqrt((x2-x1)**2 + (y2-y1)**2)

        if distance > max_distance:
            max_distance = distance
            most_distant_point_index = i

    return most_distant_point_index, max_distance

# Funkcija za odredjivanja parametara fitovane linije
# nad prosledjenim skupom tacaka
def fit_line(points, selected_algorithm):

    if selected_algorithm == 'iter':
        points = [points[0], points[-1]]

    xc = np.mean(np.array(points)[:,0])
    yc = np.mean(np.array(points)[:,1])

    x_cent = xc - np.array(points)[:,0]
    y_cent = yc - np.array(points)[:,1]

    sum_1 = np.sum(x_cent*y_cent)

    sum_2 = np.sum(y_cent**2 - x_cent**2)

    alpha = 1/2 * math.atan2(-2*sum_1, sum_2)
    r = xc*math.cos(alpha) + yc*math.sin(alpha)

    return [r, alpha]

# Funkcija za citanje podataka sa lidara
def lidar_callback(lidar_data):

    global selected_algorithm

    points = []

    for i in range(len(lidar_data.ranges)):

        theta = lidar_data.angle_min + i*lidar_data.angle_increment

        if theta > 2*pi :
            theta = 2*pi

        rho = lidar_data.ranges[i] 

        if rho != math.inf:
            x = rho*math.cos(theta)
            y = rho*math.sin(theta)
            points.append([x, y, rho, theta])

    if selected_algorithm in ['rec', 'iter']:

        start = time.time()

        line_parameters, marker_points = split_and_merge(points, selected_algorithm)

        end = time.time()

        time_diff = end-start

        started_algorithm = False

        print('\nParametri detektovanih linija su:')

        for line in line_parameters:
            print("r = {:.5f} m, alpha = {:.5f} deg".format(line[0], 180/pi*line[1]))

        print('Vreme izvrsavanja ' + ('rekurzivnog' if selected_algorithm == 'rec' else 'iterativnog') + \
            ' Split and Merge algoritma je {:.5f}'.format(time_diff) + 's')

        print('-----------------------------------------')

        marker = make_marker(marker_points)
        pub_marker.publish(marker)

# Funkcija za pomeranje robota
def move_robot(linear_vel, angular_vel):

    global pub_velocities, vel

    vel.linear.x = linear_vel
    vel.angular.z = angular_vel

    pub_velocities.publish(vel)

    return         


# Callback funkcija preko koje se u odredjenom trenutku 
# pokrece odredjeni algoritam (S&Mr ili S&Mi)
def callback(data):

    global selected_algorithm

    if data.data in ['rec','iter']:
        selected_algorithm = data.data

    else:

        try:
            inputs = data.data
            linear_vel, angular_vel = inputs.split(' ')
            linear_vel = float(linear_vel)
            angular_vel = float(angular_vel)

            selected_algorithm = None

            move_robot(linear_vel, angular_vel)

        except:
            pass


# Funkcija za subscribe-ovanje na sve potrebne topic-e 
def listener():

    rospy.Subscriber('scan', LaserScan, lidar_callback)
    rospy.Subscriber('alg_control', String, callback)
    rospy.spin()

# main program za pokretanje svih gore navedenih funkcionalnosti
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
