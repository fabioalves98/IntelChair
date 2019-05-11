import sqlite3
import json
from flask import Flask, g
from flask import render_template
from flask import redirect, url_for, request

app = Flask(__name__)
DATABASE = "database.db"

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)

    db.row_factory = sqlite3.Row
    return db

@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

@app.route("/")
def start():
    return redirect(url_for('login'))

@app.route("/login", methods=['POST', 'GET'])
def login():
    error = None

    if request.method == 'POST':
        username = request.form['name']
        password = request.form['password']
        if valid_login(username, password):
            return log_the_user_in()
        else:
            error = "Invalid username or password"

    return render_template('auth.html') # executed if method = GET


@app.route("/index")
def index():
    return render_template('index.html')

@app.route("/users", methods=['POST', 'GET'])
def query_users():
  if request.method == 'POST':
    add_user()
  
  return get_allusers()

def get_allusers():
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        users = c.execute("SELECT * FROM users")
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text);")
        print("Table 'users' created")

    return json.dumps([dict(x) for x in users])

@app.route("/users/<username>", methods=['POST', 'GET'])
def user_update(username):
  if request.method == 'POST':
    return update_user(username)
  
  return get_user(username)

def get_user(username):
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text);")
        print("Table 'users' created")

    if user == None:
        print("User '?' not found", username)

    return json.dumps([dict(x) for x in user])

def add_user():
    db = get_db()
    c = db.cursor()

    firstname = request.form['firstname']
    lastname = request.form['lastname']
    username = request.form['username']
    password = request.form['password']
    email = request.form['email']
    age = request.form['age']
    gender = request.form['gender']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("INSERT INTO users(firstname, lastname, username, password, email, age, gender, chair) \
                        VALUES(?, ?, ?, ?, ?, ?, ?, ?)", (firstname, lastname, username, password, email, age, gender))
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        print("Table 'users' created")

def update_user(username):
    db = get_db()
    c = db.cursor()
    # firstname = 
    # lastname = 
    # password
    # email
    # age
    # gender
    # chair

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("UPDATE users SET firstname = ?...") # ??
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        print("Table 'users' created")

def valid_login(username, password):
    db = get_db()
    user = db.execute('select * from users where username = ?', [username])
    if user is None:
        return False

    return True

def log_the_user_in():
    return redirect(url_for('index'))

