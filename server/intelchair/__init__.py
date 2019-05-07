import markdown
import os
import shelve
import json
from os.path import dirname

# Import the framework
from flask import Flask, g, request, render_template
from flask_cors import CORS
from flask_restful import Resource, Api, reqparse


# Create an instance of Flask
app = Flask(__name__)
CORS(app)

# Create the API
api = Api(app)

def get_db(name):
    db = getattr(g, '_database', None)
    if db is None:
        if name == "users.db":
            db = g._database = shelve.open("users.db")
        elif name == "chairs.db":
            db = g._database = shelve.open("chairs.db")
        elif name == "history.db":
            db = g._database = shelve.open("history.db")
        elif name == "maps.db":
            db = g._database = shelve.open("maps.db")
    return db

@app.teardown_appcontext
def teardown_db(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

@app.route("/index.html")
def index():
    return open(app.root_path + '/index.html').read()

# @app.route("/")
# @app.route("/auth.html")
# def auth():
#     return open(app.root_path + '/auth.html').read()

class Authentication(Resource):
    @app.route("/")
    @app.route("/auth.html")
    def auth():
        return open(app.root_path + '/auth.html').read()

    def get(self):
        # username = request.json.get('username')
        # password = request.json.get('password')
        # print(username)
        return ""
        
    def post(self):
        username = request.json.get('username')
        password = request.json.get('password')
        print(username)
        return ""


    # @auth.verify_password
    # def verify_password(username, password):
    #     user = User.query.filter_by(username = username).first()
    #     if not user or not user.verify_password(password):
    #         return False
    #     g.user = user
    #     return True



############## USERS ##############
class UserList(Resource):
    def get(self):
        shelf = get_db("users.db")
        keys = list(shelf.keys())

        users = []

        for key in keys:
            users.append(shelf[key])

        return json.dumps(users), 200

    def post(self):
        parser = reqparse.RequestParser()

        parser.add_argument('first-name', required=True)
        parser.add_argument('last-name', required=True)
        parser.add_argument('username', required=True)
        parser.add_argument('password', required=True)
        parser.add_argument('email', required=True)
        parser.add_argument('age', required=True)
        parser.add_argument('gender', required=True)
        parser.add_argument('chair', required=False)

        # Parse the arguments into an object
        args = parser.parse_args()

        shelf = get_db("users.db")
        shelf[args['username']] = args

        return {'message': 'User registered', 'data': args}, 201


class User(Resource):
    def get(self, username):
        shelf = get_db("users.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (username in shelf):
            return {'message': 'User not found', 'data': {}}, 404

        return {'message': 'User found', 'data': shelf[username]}, 200

    def delete(self, username):
        shelf = get_db("users.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (username in shelf):
            return {'message': 'User not found', 'data': {}}, 404

        del shelf[username]
        return '', 204


############## CHAIRS ##############
class ChairList(Resource):
    def get(self):
        shelf = get_db("chairs.db")
        keys = list(shelf.keys())

        chairs = []

        for key in keys:
            chairs.append(shelf[key])

        return json.dumps(chairs), 200

    def post(self):
        parser = reqparse.RequestParser()

        parser.add_argument('company', required=True)
        parser.add_argument('model', required=True)
        parser.add_argument('name', required=True)
        parser.add_argument('id', required=True)
        parser.add_argument('ip', required=False)
        parser.add_argument('user', required=False)
        parser.add_argument('status', required=False)
        parser.add_argument('battery', required=False)

        # Parse the arguments into an object
        args = parser.parse_args()

        shelf = get_db("chairs.db")
        shelf[args['name']] = args

        return {'message': 'Chair registered', 'data': args}, 201


class Chair(Resource):
    def get(self, name):
        shelf = get_db("chairs.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (name in shelf):
            return {'message': 'Chair not found', 'data': {}}, 404

        return {'message': 'Chair found', 'data': shelf[name]}, 200

    def delete(self, name):
        shelf = get_db("chairs.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (name in shelf):
            return {'message': 'Chair not found', 'data': {}}, 404

        del shelf[name]
        return '', 204


############## Chairs Use History ##############
class HistoryList(Resource):
    def get(self):
        shelf = get_db("history.db")
        keys = list(shelf.keys())

        histories = []

        for key in keys:
            histories.append(shelf[key])

        return json.dumps(histories), 200

    def post(self):
        parser = reqparse.RequestParser()
        
        parser.add_argument('start', required=True)
        parser.add_argument('end', required=True)
        parser.add_argument('username', required=True)
        parser.add_argument('chairId', required=True)

        # Parse the arguments into an object
        args = parser.parse_args()

        shelf = get_db("history.db")
        shelf[args['chairId']] = args

        return {'message': 'History saved', 'data': args}, 201

class History(Resource):
    def get(self, chairId):
        shelf = get_db("history.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (chairId in shelf):
            return {'message': 'Chair history not found', 'data': {}}, 404

        return {'message': 'Chair history found', 'data': shelf[chairId]}, 200

    def delete(self, chairId):
        shelf = get_db("history.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (chairId in shelf):
            return {'message': 'Chair history not found', 'data': {}}, 404

        del shelf[chairId]
        return '', 204

############## Maps ##############
class MapList(Resource):
    def get(self):
        shelf = get_db("maps.db")
        keys = list(shelf.keys())

        maps = []

        for key in keys:
            maps.append(shelf[key])

        return json.dumps(maps), 200

    def post(self):
        parser = reqparse.RequestParser()
        
        parser.add_argument('name', required=True)
        parser.add_argument('image', required=True)

        # Parse the arguments into an object
        args = parser.parse_args()

        shelf = get_db("maps.db")
        shelf[args['name']] = args

        return {'message': 'Map saved', 'data': args}, 201

class Map(Resource):
    def get(self, name):
        shelf = get_db("maps.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (name in shelf):
            return {'message': 'Map not found', 'data': {}}, 404

        return {'message': 'Map found', 'data': shelf[name]}, 200

    def delete(self, name):
        shelf = get_db("maps.db")

        # If the key does not exist in the data store, return a 404 error.
        if not (name in shelf):
            return {'message': 'Map not found', 'data': {}}, 404

        del shelf[name]
        return '', 204


api.add_resource(UserList, '/users')
api.add_resource(User, '/users/<string:username>')
api.add_resource(ChairList, '/chairs')
api.add_resource(Chair, '/chairs/<string:name>')
api.add_resource(HistoryList, '/chairs/history')
api.add_resource(History, '/chairs/history/<string:chairId>')
api.add_resource(Authentication, '/auth')
api.add_resource(MapList, '/maps')
api.add_resource(Map, '/maps/<string:name>')

