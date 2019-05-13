import sqlite3
import json
from flask import Flask, g
from flask import render_template
from flask import redirect, url_for, request
from flask_cors import CORS

app = Flask(__name__)
cors = CORS(app)
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

@app.route("/users/<username>", methods=['POST', 'GET', 'DELETE'])
def user_update(username):
    if request.method == 'POST':
        return update_user(username)
    elif request.method == 'DELETE':
        return remove_user(username)
  
    return get_user(username)

@app.route("/remove/users/<username>", methods=['POST'])
def remove(username):
    return remove_user(username)


@app.route("/chairs", methods=['POST', 'GET'])
def query_chairs():
    if request.method == 'POST':
        add_chair()
  
    return get_allchairs()

@app.route("/chairs/<id>", methods=['POST', 'GET'])
def chair_update(id):
    if request.method == 'POST':
        return update_chair(id)
  
    return get_chair(id)

def get_allusers():
    db = get_db()
    c = db.cursor()

    users=[]
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        users = c.execute("SELECT * FROM users")
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text);")
        db.commit()
        print("Table 'users' created")

    return json.dumps([dict(x) for x in users])

def get_user(username):
    db = get_db()
    c = db.cursor()

    users=[]
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text);")
        db.commit()
        print("Table 'users' created")

    if user == None:
        print("User '?' not found", username)

    return json.dumps([dict(x) for x in user])

def add_user():
    db = get_db()
    c = db.cursor()

    firstname = request.form['first-name']
    lastname = request.form['last-name']
    username = request.form['username']
    password = request.form['password']
    email = request.form['email']
    age = request.form['age']
    gender = request.form['gender']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("INSERT INTO users(firstname, lastname, username, password, email, age, gender, chair) \
                        VALUES(?, ?, ?, ?, ?, ?, ?, ?)", (firstname, lastname, username, password, email, age, gender, None))
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
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
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")

def remove_user(username):
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("DELETE FROM users WHERE username = ?", [username,])
        db.commit()
        return 200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")


def valid_login(username, password):
    db = get_db()
    c = db.cursor()
    user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
    rows = user.fetchone()

    if rows is None:
        return False

    return password == rows['password']

def log_the_user_in():
    return redirect(url_for('index'))


#### Chairs ####
def get_allchairs():
    db = get_db()
    c = db.cursor()
    chairs = []

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        chairs = c.execute("SELECT * FROM chairs")
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")

    return json.dumps([dict(x) for x in chairs])

def get_chair(id):
    db = get_db()
    c = db.cursor()
    chairs = []

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        chair = c.execute("SELECT * FROM chairs WHERE id = ?", [id,])
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")

    if chair == None:
        print("Chair '?' not found", id)

    return json.dumps([dict(x) for x in chair])

def add_chair():
    db = get_db()
    c = db.cursor()

    company = request.form['company']
    model = request.form['model']
    name = request.form['name']
    id = request.form['id']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("INSERT INTO chairs(company, model, name, id, ip, user, status, battery) \
                        VALUES(?, ?, ?, ?, ?, ?, ?, ?)", (company, model, name, id, None, None, None, None))
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")