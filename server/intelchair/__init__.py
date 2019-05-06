import markdown
import os
import shelve

# Import the framework
from flask import Flask, g
from flask_restful import Resource, Api, reqparse

# Create an instance of Flask
app = Flask(__name__)

# Create the API
api = Api(app)

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = shelve.open("users.db")
    return db

@app.teardown_appcontext
def teardown_db(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

@app.route("/")
def index():
    """Present some documentation"""

    # Open the README file
    with open(os.path.dirname(app.root_path) + '/README.md', 'r') as markdown_file:

        # Read the content of the file
        content = markdown_file.read()

        # Convert to HTML
        return markdown.markdown(content)


class UserList(Resource):
    def get(self):
        shelf = get_db()
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
        parser.add_argument('email', required=True)
        parser.add_argument('age', required=True)
        parser.add_argument('gender', required=True)

        # Parse the arguments into an object
        args = parser.parse_args()

        shelf = get_db()
        shelf[args['username']] = args

        return {'message': 'User registered', 'data': args}, 201


class User(Resource):
    def get(self, username):
        shelf = get_db()

        # If the key does not exist in the data store, return a 404 error.
        if not (username in shelf):
            return {'message': 'User not found', 'data': {}}, 404

        return {'message': 'User found', 'data': shelf[username]}, 200

    def delete(self, username):
        shelf = get_db()

        # If the key does not exist in the data store, return a 404 error.
        if not (username in shelf):
            return {'message': 'User not found', 'data': {}}, 404

        del shelf[username]
        return '', 204


api.add_resource(UserList, '/users')
api.add_resource(User, '/users/<string:username>')
