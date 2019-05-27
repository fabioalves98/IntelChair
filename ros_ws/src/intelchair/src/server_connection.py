#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy
from intelchair.msg import ChairMsg
import time

def chair_info_callback(data):
    global connected_sent
    chair_battery = data.battery
    url = 'http://' + server_address + api_call
    values = {'battery' : chair_battery, 'status':'Online', 'ip' : ip}
    data = urllib.urlencode(values)
    req = urllib2.Request(url)
    req.add_data(data)
    req.get_method = lambda: 'PUT'
    
    try:
        html = urllib2.urlopen(req)
        sub.unregister()
        connected_sent = 1
    except Exception:
        print('Server not up! - ' + str(time.time()))

def ping_server(event):
    global connected_sent, sub
    if(connected_sent == 1):
        print ('Conecting to server at ' + str(event.current_real))
        url = 'http://' + server_address + '/chair_active'
        values = {'timestamp' : time.time(), 'id': '123123'}
        data = urllib.urlencode(values)
        req = urllib2.Request(url)
        req.add_data(data)
        try:
            html = urllib2.urlopen(req)
        except Exception:
            print('Connection to server was closed! Make sure the server is running!')
            connected_sent = 0
            sub = rospy.Subscriber("chair_info", ChairMsg, chair_info_callback)

rospy.init_node("server_connection", anonymous=True)

sub = rospy.Subscriber("chair_info", ChairMsg, chair_info_callback)

ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr']

localhost = '127.0.0.1:5000'
api_call = "/chairs/123123"
server_address = '192.168.1.209:5000'

chair_battery = -1
connected_sent = 0

rospy.Timer(rospy.Duration(5), ping_server)

rospy.spin()
