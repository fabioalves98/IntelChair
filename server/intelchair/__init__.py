import markdown
import os
import shelve
from os.path import dirname

# Import the framework
from flask import Flask, g
from flask_restful import Resource, Api, reqparse

# Create an instance of Flask
app = Flask(__name__)

# Create the API
api = Api(app)

def get_db(name):
    db = getattr(g, '_database', None)
    if db is None:
        if name == "users.db":
            db = g._database = shelve.open("users.db")
        elif name == "chairs.db":
            db = g._database = shelve.open("chairs.db")
    return db

@app.teardown_appcontext
def teardown_db(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()


# def index():
#     """Present some documentation"""

#     # Open the README file
#     with open(os.path.dirname(app.root_path) + '/README.md', 'r') as markdown_file:

#         # Read the content of the file
#         content = markdown_file.read()

#         # Convert to HTML
#         return markdown.markdown(content)

@app.route("/")
def index():
    return open(app.root_path + '/index.html').read()


############## USERS ##############
class UserList(Resource):
    def get(self):
        shelf = get_db("users.db")
        keys = list(shelf.keys())

        users = []

        for key in keys:
            users.append(shelf[key])

        return {'message': 'Success', 'data': users}, 200

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

        return {'message': 'Success', 'data': chairs}, 200

    def post(self):
        parser = reqparse.RequestParser()

        parser.add_argument('company', required=True)
        parser.add_argument('model', required=True)
        parser.add_argument('name', required=True)
        parser.add_argument('id', required=True)
        parser.add_argument('ip', required=False)
        parser.add_argument('user', required=False)

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



api.add_resource(UserList, '/users')
api.add_resource(User, '/users/<string:username>')
api.add_resource(ChairList, '/chairs')
api.add_resource(Chair, '/chairs/<string:name>')
