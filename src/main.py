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
points = [[3.199741196401497, -0.9201893218574223, 3.329428195953369, 6.003159357234836], [3.209520408092776, -0.8624816133408421, 3.3233861923217773, 6.020661279559135], [3.2120279310801085, -0.803155242932764, 3.3109185695648193, 6.038163201883435], [3.19368266731298, -0.7394304947107166, 3.278165102005005, 6.055665124207735], [3.2075711882244935, -0.6837307970781504, 3.2796342372894287, 6.073167046532035], [3.1933008747034655, -0.6224717885569702, 3.2534046173095703, 6.090668968856335], [3.1922457915664992, -0.5644640221004115, 3.241766929626465, 6.1081708911806345], [3.2058137130746247, -0.5091735906833983, 3.245997428894043, 6.125672813504934], [3.1894809286953114, -0.44950208329292163, 3.2209999561309814, 6.143174735829234], [3.2055024022010628, -0.3946782393944295, 3.229708433151245, 6.160676658153534], [3.212404331304098, -0.3385693812902584, 3.230196714401245, 6.178178580477834], [3.2085770381946506, -0.2814847244382805, 3.220900535583496, 6.195680502802134], [3.1892037025053432, -0.223618844207031, 3.1970338821411133, 6.213182425126433], [3.189502936145173, -0.16760598714111957, 3.193903684616089, 6.230684347450733], [3.2111748927019046, -0.11243394193261609, 3.2131426334381104, 6.248186269775033], [3.198276594429586, -0.05596632509328222, 3.1987662315368652, 6.265688192099333], [3.1942358016598686, 1.5355471039630377e-05, 3.1942358016967773, 6.283190114423633]]

# Promenljiva za indikaciju zapocetog algoritma kako ne bi dolazilo do preklapanja 
started_algorithm = False

# Funkcija za rekurzivni Split and Merge algoritam
def split_and_merge_recursive(points, threshold_split = 0.2, threshold_merge = 0.1):

    # Split deo 
    line_parametars_and_points = split(points, threshold_split)

    # Merge deo 
    while True:
        final_line_parametars, merge_result = merge(line_parametars_and_points, threshold_merge)

        if merge_result == False:
            break

    return final_line_parametars

def merge(line_parametars_and_points, threshold_merge, abs_angle_tolerancy = 3*pi/180):

    merged = []

    cnt_not_merged = 0

    if len(line_parametars_and_points) == 1:
        merged = [line_parametars_and_points[0][0], line_parametars_and_points[0][1]]

        return merged, False

    for i in range(1, len(line_parametars_and_points)):

        r1 = line_parametars_and_points[i-1][0] 
        alpha1 = line_parametars_and_points[i-1][1]

        r2 = line_parametars_and_points[i][0] 
        alpha2 = line_parametars_and_points[i][1] 

        # Provera kolineranosti 
        if np.abs(alpha1 - alpha2) <= abs_angle_tolerancy :

            # Ukoliko su dva segmenta kolinearna 
            # uzimamo prosecne vrednosti parametara linija
            r_avg = (r1 + r2)/2
            alpha_avg = (alpha1 + alpha2)/2

            # Pronalazenje najudaljenije tacke 
            max_distance = 0

            points_1 = line_parametars_and_points[i-1][2]
            points_2 = line_parametars_and_points[i][2]

            for i in range(len(points_1)):
                if np.abs(r_avg - points_1[i][2]) > max_distance:
                    max_distance = points_1[i][2]

            for i in range(len(points_2)):
                if np.abs(r_avg - points_2[i][2]) > max_distance:
                    max_distance = points_2[i][2]

            # Ukoliko su dve linije bliske jedna drugoj, merge-uju se
            if max_distance <= threshold_merge:
                merged.append([r_avg, alpha_avg, points_1 + points_2])

            else:
                cnt_not_merged += 1
                merged.append(line_parametars_and_points[i-1])
                merged.append(line_parametars_and_points[i])

        else:
            cnt_not_merged += 1
            merged.append(line_parametars_and_points[i-1])
            merged.append(line_parametars_and_points[i])

    if cnt_not_merged == len(line_parametars_and_points):
        return merged, False

    return merged, True

def split(points, threshold_split):

    line_parametars_and_points = []

    # Fitovanje linije na celom trenutnom setu tacaka
    xc = np.mean(np.array(points)[:,0])
    yc = np.mean(np.array(points)[:,1])

    x_cent = xc - np.array(points)[:,0]
    y_cent = yc - np.array(points)[:,1]

    sum_1 = np.sum(x_cent*y_cent)
    sum_2 = np.sum(y_cent**2 - x_cent**2)

    alpha = 1/2 * math.atan2(-2*sum_1, sum_2)
    r = xc*math.cos(alpha) + yc*math.sin(alpha)

    # Pronalazenje najudaljenije tacke 
    max_distance = 0

    for i in range(len(points)):
        if np.abs(r - points[i][2]) > max_distance:
            max_distance = points[i][2]
            most_distant_point_index = i

    # Ponovno splitovanje
    if np.abs(r - points[most_distant_point_index][2]) > threshold_split:
        left_points = points[0:most_distant_point_index+1] 
        right_points = points[most_distant_point_index+1:]

        split(left_points, threshold_split)
        split(right_points, threshold_split)
    else:
        line_parametars_and_points.append([r, alpha, points])

    return line_parametars_and_points

    
# Funkcija za citanje podataka sa lidara
def lidar_callback(lidar_data):

    global points, started_algorithm

    if started_algorithm == False:

        for i in range(len(lidar_data.ranges)):
            theta = lidar_data.angle_min + i*lidar_data.angle_increment
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

        line_parametars = split_and_merge_recursive(points)

        end = time.time()

        time_diff = end-start

        started_algorithm = False

        print('\nParametri detektovanih linija su: \n')
        print(line_parametars)
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