#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy
from intelchair.msg import ChairMsg
import time

chair_battery = -1
connected_sent = 0

def chair_info_callback(data):
    global connected_sent
    chair_battery = data.battery
    url = 'http://' + server_address + api_call
    print(url)
    values = {'battery' : chair_battery, 'status':'Online', 'ip' : ip}
    data = urllib.urlencode(values)
    req = urllib2.Request(url)
    req.add_data(data)
    req.get_method = lambda: 'PUT'
    
    connected_sent = 1
    try:
        html = urllib2.urlopen(req)
    except Exception:
        print('Server not up!')

    sub.unregister()




def ping_server(event):
    if(connected_sent == 1):
        print ('Timer called at ' + str(event.current_real))
        url = 'http://' + server_address + '/chair_active'
        values = {'timestamp' : time.time(), 'id': '123123'}
        data = urllib.urlencode(values)
        req = urllib2.Request(url)
        req.add_data(data)
        try:
            html = urllib2.urlopen(req)
        except Exception:
            print('Connection to server was closed! Make sure the server is running!')


rospy.init_node("server_connection", anonymous=True)

sub = rospy.Subscriber("chair_info", ChairMsg, chair_info_callback)

ip = ni.ifaddresses('wlo1')[ni.AF_INET][0]['addr']

localhost = '127.0.0.1:5000'
api_call = "/chairs/123123"
server_address = '192.168.43.122:5000'

rospy.Timer(rospy.Duration(5), ping_server)

rospy.spin()
