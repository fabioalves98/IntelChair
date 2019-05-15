#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy
from intelchair.msg import ChairMsg

chair_battery = -1

def chair_info_callback(data):
    chair_battery = data.battery

    print(chair_battery)

    url = 'http://' + server_address + api_call
    print(url)
    values = {'battery' : chair_battery, 'status':'Online','username': None, 'chair_ip' : ip}
    data = urllib.urlencode(values)
    req = urllib2.Request(url)
    req.add_data(data)

    html = urllib2.urlopen(req)
    rospy.signal_shutdown('')


rospy.init_node("server_connection", anonymous=True)

rospy.Subscriber("chair_info", ChairMsg, chair_info_callback)

ip = ni.ifaddresses('wlo1')[ni.AF_INET][0]['addr']

localhost = '127.0.0.1:8080'
api_call = "/chair/123123"
server_address = 'localhost:5000'



rospy.spin()
