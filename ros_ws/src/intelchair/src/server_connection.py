#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy

## Chair's max speed in km/h 
CHAIR_MAX_SPEED = 6

rospy.init_node("server_connection", anonymous=True)

ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr']

localhost = '127.0.0.1:8080'
api_call = "/api/set_chair_info"
server_address = '192.168.1.209:8080'


url = 'http://' + server_address + api_call
# print(url)
values = {'chair_ip' : ip, 'chair_maxspeed': CHAIR_MAX_SPEED}
data = urllib.urlencode(values)
req = urllib2.Request(url)
req.add_data(data)
    
html = urllib2.urlopen(req)
