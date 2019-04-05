import os, os.path
import random
import string
import json

import cherrypy


class Root(object):
	def __init__(self):
		self.api = api()

	@cherrypy.expose	
	def index(self):
		raise cherrypy.HTTPRedirect("/static/index.html" , status=302)


class api(object):
	def __init__(self):
		self.chairIP = ""
	
	@cherrypy.expose
	def setip(self, chair_ip):
		print (chair_ip)
		self.chairIP = chair_ip

	@cherrypy.expose
	def getip(self):
		return json.dumps({"ip" : self.chairIP})


if __name__ == '__main__':
	conf = {
		# 'global': {
		# 	'server.socket_host' : '192.168.1.209',
		# 	'server.socket_port' : 8080
		# },
		'/': {
			'tools.sessions.on': True,
			'tools.staticdir.root':  os.path.abspath(os.getcwd())
		},
		'/static':{
			'tools.staticdir.on': True,
			'tools.staticdir.dir': './../mobile'
		}
	}
	cherrypy.quickstart(Root(), '/', conf)