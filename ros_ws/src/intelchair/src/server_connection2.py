#!/usr/bin/env python

import netifaces as ni
import urllib
import urllib2
import rospy
from intelchair.msg import ChairMsg
import time

def engine(event):
    global sub, canSub
    if canSub:
        sub = rospy.Subscriber("chair_info", ChairMsg, chair_info_callback)
        print ("Listening to Chair Info...")
        canSub = False

def chair_info_callback(data):
    global connected, canSub
    chair_battery = data.battery
    url = 'http://' + server_address + api_call
    values = {'battery' : chair_battery, 'status':'Online', 'ip' : ip}
    data = urllib.urlencode(values)
    req = urllib2.Request(url)
    req.add_data(data)
    req.get_method = lambda: 'PUT'
    
    try:
        html = urllib2.urlopen(req)
        print('Server up! - ' + str(time.time()))
        connected = 1
    except Exception:
        print('Server not up! - ' + str(time.time()))
        connected = 0
    
    sub.unregister()
    canSub = True

def ping_server(event):
    global connected_sent, sub
    if(connected == 1):
        #print ('Timer called at ' + str(event.current_real))
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

canSub = True
sub = None

ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr']

localhost = '127.0.0.1:5000'
api_call = "/chairs/123123"
server_address = '192.168.1.213:5000'

chair_battery = -1
connected = 0

rospy.Timer(rospy.Duration(5), ping_server)
rospy.Timer(rospy.Duration(5), engine)

rospy.spin()
