import netifaces as ni
import requests

SERVER = '127.0.0.1'
ip = ni.ifaddresses('wlo1')[ni.AF_INET][0]['addr']
r = requests.post(url = SERVER, data={'chair_ip': ip})

