#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import numpy as np
import time
import math
from math import pi

# Importovanje potrebnih struktura poruka
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Promenljiva za cuvanje tacaka
points = []

# Promenljiva za indikaciju zapocetog algoritma kako ne bi dolazilo do preklapanja 
started_algorithm = False

# Funkcija za rekurzivni Split and Merge algoritam
def split_and_merge_recursive(points, threshold_split = 0.1, threshold_merge = 0.1):

    result = []

    # Split deo 
    line_parameters_with_points = split(points, threshold_split, True)

    # Merge deo 

    merge_result, none_merged = merge(line_parameters_with_points, threshold_merge)

    while none_merged != False:
        merge_result, none_merged = merge(merge_result, threshold_merge)
    
    for line_segment in merge_result:
        result.append([line_segment[0], line_segment[1]])

    return result

# Funkcija za merge-ovanje tacaka koje potencijalno pripadaju
# jednoj istoj liniji 
def merge(line_parameters_with_points, threshold_merge, abs_angle_tolerancy = 3*pi/180):

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

        # Provera kolineranosti 
        if np.abs(alpha1 - alpha2) <= abs_angle_tolerancy :

            # Ukoliko su dva segmenta kolinearna 
            # uzimamo prosecne vrednosti parametara linija
            r_avg = (r1 + r2)/2
            alpha_avg = (alpha1 + alpha2)/2

            # Pronalazenje najudaljenije tacke 

            _, max_distance_1 = find_max_distant_point(points_1, r_avg, alpha_avg)
            _, max_distance_2 = find_max_distant_point(points_2, r_avg, alpha_avg)

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

# Funkcija za splitovanje podataka, ondosno tacaka 
# na one koje pripadaju linijama posebnim
def split(points, threshold_split, first_split = False):

    line_parameters_with_points = []

    num_points = len(points)

    # Fitovanje linije na celom trenutnom setu tacaka
    [r, alpha] = fit_line(points)

    # Pronalazenje najudaljenije tacke 
    most_distant_point_index, max_distance = find_max_distant_point(points, r, alpha)

    # if first_split == True:
    # Ponovno splitovanje
    if max_distance > threshold_split and most_distant_point_index not in [0, num_points-1]:

        left_points = points[0:most_distant_point_index] 
        right_points = points[most_distant_point_index:]

        if(left_points):
            left = split(left_points, threshold_split)
            for item in left:
                line_parameters_with_points.append(item)

        if(right_points):
            right = split(right_points, threshold_split)
            for item in right:
                line_parameters_with_points.append(item)

    else:
        
        line_parameters_with_points.append([r, alpha, points])

    return line_parameters_with_points

# Funkcija za pronalazenje najudaljenije tacke od trenutne
# fitovane linije
def find_max_distant_point(points, r, alpha):

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
    max_distance = -1

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
def fit_line(points):

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

    global points, started_algorithm

    if started_algorithm == False:

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
    
# Callback funkcija preko koje se u odredjenom trenutku 
# pokrece odredjeni algoritam (S&Mr ili S&Mi)
def start_algorithm_callback(data):

    global points, started_algorithm

    if ((data.data =='rec') and (points != []) and (started_algorithm == False)):

        started_algorithm = True

        start = time.time()

        line_parameters = split_and_merge_recursive(points)

        end = time.time()

        time_diff = end-start

        started_algorithm = False

        print('\nParametri detektovanih linija su: \n')
        print(line_parameters)
        print('\nVreme izvrsavanja rekurzivnog S&M algoritma je ' + str(time_diff) + ' s')

# Funkcija za subscribe-ovanje na sve potrebne topic-e 
def listener():
    rospy.init_node('lidar_values')
    rospy.Subscriber('scan', LaserScan, lidar_callback)
    rospy.Subscriber('alg_select', String, start_algorithm_callback)
    rospy.spin()

# main program za pokretanje svih gore navedenih funkcionalnosti
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass