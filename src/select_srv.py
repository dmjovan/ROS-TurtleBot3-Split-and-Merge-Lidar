#!/usr/bin/env python3

# Ova skripta predstavlja ROS servis kojim je implementiran
# unos komandi kada zelimo da izvrsimo Split and Merge algoritam 

# Ucitavanje potrebnih biblioteka i struktura poruka
import rospy
from domaci_3.srv import select, selectResponse
from std_msgs.msg import String

# Inicijalizacija globalnog publisher-a na topic /alg_select
# preko kojeg se komunicira sa ostalim skriptama o tome koja selekcija
# je upisana
pub = rospy.Publisher('alg_select', String, queue_size=1)
rospy.init_node('select', anonymous=False)

# Callback funkcija za ROS servis
# koja publish-uje selekcije koje su zadate preko konzole
def response_callback(req):

    pub.publish(req.select)

    rospy.loginfo('Izabrani algoritam : ' + req.select)

    # Povratna vrednost servisa
    return selectResponse(True)

# Inicijalizacija ROS servisa za unos komandi
s = rospy.Service('select', select, response_callback)
rospy.loginfo("Servis za izbor trenutka u kojem se pokrece algoritam i za selekciju algoitma je spreman!")
rospy.spin()