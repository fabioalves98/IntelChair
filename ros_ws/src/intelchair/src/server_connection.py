#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy

rospy.init_node("server_connection", anonymous=True)

ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr']

#url = 'http://127.0.0.1:8080/api/setip'
url = 'http://192.168.1.209:8080/api/setip'
values = {'chair_ip' : ip}
data = urllib.urlencode(values)
req = urllib2.Request(url)
req.add_data(data)
    
html = urllib2.urlopen(req)
