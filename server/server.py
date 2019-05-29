import sqlite3
import json
import os
from flask import Flask, g
from flask import render_template
from flask import redirect, url_for, request
from flask_cors import CORS
from PIL import Image
import time
import threading

chair_connection = {}
current_timestamp = 0

def set_interval(func, sec):
    def func_wrapper():
        set_interval(func, sec) 
        func()  
    t = threading.Timer(sec, func_wrapper)
    t.start()
    return t

def check_connected():
    current_timestamp = time.time()

    for key in chair_connection.keys():
        if(float(current_timestamp) > float(chair_connection[key]) + 10):
            print('Chair disconnected | ID = ', key)

            conn = sqlite3.connect(DATABASE)
            c = conn.cursor()

            c.execute("UPDATE chairs SET ip = ?, user = ?, status = ?,battery = ? WHERE id = ?", (None, None, 'Offline', None, str(key)))

            conn.commit()
            conn.close()

            chair_connection.pop(key)

def init_db():
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    c.execute("CREATE TABLE IF NOT EXISTS users \
             (firstname text, lastname text, username text UNIQUE, password text, email text, age integer, role text, status text);")
    try:
        c.execute("INSERT INTO users VALUES \
                (?, ?, ?, ?, ?, ?, ?, ?);", ("Fabio", "Alves", "fmcalves", "123", "fabioalves98@ua.pt", "20", "admin", "Offline"))
    except sqlite3.IntegrityError:
        print ("User Already Inserted")

    c.execute("CREATE TABLE IF NOT EXISTS chairs \
             (company text, model text, name text, id text UNIQUE, ip text, user text, status text, battery integer);")
    try:
        c.execute("INSERT INTO chairs (company, model, name, id) VALUES \
                (?, ?, ?, ?);", ("Karma", "RX123", "IntelChair", "123123"))
    except sqlite3.IntegrityError:
        print("Chair Already Inserted")
             
    c.execute("CREATE TABLE IF NOT EXISTS history \
             (startTime text, endTime text, username text, chair text, UNIQUE(startTime, chair));")
    try:
        c.execute("INSERT INTO history VALUES \
                (?, ?, ?, ?);", ("1558380289", "1558383169", "fmcalves", "123123"))
    except sqlite3.IntegrityError:
        print ("History Already Inserted")

    c.execute("CREATE TABLE IF NOT EXISTS maps \
             (name text, pgm_path text, yaml_path text, UNIQUE(name));")
    try:
        pgm_path = prev_dir + "/ros_ws/maps/iris.pgm"
        yaml_path = prev_dir + "/ros_ws/maps/iris.yaml"
        c.execute("INSERT INTO maps VALUES \
                (?, ?, ?);", ("iris", pgm_path, yaml_path))
    except sqlite3.IntegrityError:
        print ("Map Already Inserted")
    except FileNotFoundError:
        print("iris.pgm or iris.yaml not found!") 
    
    conn.commit()
    conn.close()

cwd = os.getcwd()
prev_dir = os.path.abspath(os.path.join(cwd, os.pardir))
app = Flask(__name__)
cors = CORS(app)
DATABASE = "database.db"

stoper = set_interval(check_connected, 5)

init_db()

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
        username = request.form['username']
        password = request.form['password']
        if valid_login(username, password):
            return log_the_user_in(username)
        else:
            error = "Invalid username or password"

    return render_template('auth.html')

@app.route("/logout", methods=['POST', 'GET'])
def logout():
    if request.method == 'POST':
        username = request.form['username']
        log_the_user_out(username)

    return redirect(url_for('login'))

@app.route("/index")
def index():
    return render_template('index.html')

@app.route("/users", methods=['POST', 'GET'])
def query_users():
    if request.method == 'POST':
        add_user()
  
    return get_users()

@app.route("/users/<username>", methods=['GET', 'delete', 'put'])
def user_update(username):
    if request.method == 'PUT':
        update_user(username)
    
    if request.method == 'DELETE':
        delete_user(username)
        
    return get_user(username)

@app.route("/chairs", methods=['GET', 'POST'])
def query_chairs():
    if request.method == 'POST':
        add_chair()
  
    return get_chairs()

@app.route("/chairs/<id>", methods=['GET', 'put', 'delete'])
def chair_update(id):
    if request.method == 'PUT':
        update_chair(id)

    if request.method == 'DELETE':
        delete_chair(id)

    return get_chair(id)

@app.route("/history", methods=['GET', 'POST', 'delete'])
def query_history():
    if request.method == 'POST':
        add_history()
    
    if request.method == 'DELETE':
        delete_history()
  
    return get_history()

@app.route("/maps", methods=['POST', 'GET'])
def query_maps():
    if request.method == 'POST':
        add_map()
    
    return get_allmaps()

@app.route("/maps/<name>", methods=['POST', 'GET', 'delete'])
def map_update(name):
    if request.method == 'POST':
        update_map(name)

    if request.method == 'DELETE':
        delete_map(name)
    
    return get_map(name)

@app.route("/chair_active", methods=['POST'])
def publish_chair_active():
    global chair_connection
    if request.method == 'POST':
        ts = request.form['timestamp']
        id = request.form['id']
        chair_connection[id] = ts
        return '', 200

# Log In 
def valid_login(username, password):
    db = get_db()
    c = db.cursor()
    user = c.execute("SELECT * FROM users WHERE username = ?", [username])
    rows = user.fetchone()

    if rows is None:
        return False

    return password == rows['password']

def log_the_user_in(username):
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    c.execute("UPDATE users SET status = ? WHERE username = ?", ('Online', username))

    conn.commit()
    conn.close()

    return redirect(url_for('index'))

def log_the_user_out(username):
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    c.execute("UPDATE users SET status = ? WHERE username = ?", ('Offline', username))

    conn.commit()
    conn.close()

# Users
def get_users():
    db = get_db()
    c = db.cursor()
    
    users = c.execute("SELECT * FROM users")
    db.commit()

    return json.dumps([dict(x) for x in users])

def get_user(username):
    db = get_db()
    c = db.cursor()
    
    user = c.execute("SELECT * FROM users WHERE username = ?", [username])
    db.commit()

    rtUser = [dict(x) for x in user]

    if(len(rtUser) != 0):
        return json.dumps(rtUser[0])

    return json.dumps(None)

def add_user():
    db = get_db()
    c = db.cursor()
    
    args = ['firstname', 'lastname', 'username', 'password', 'email', 'age', 'role', 'status']

    insert = ''
    checks = ''
    values = []

    for arg in args :
        try:
            values += [request.form[arg]]
            insert += arg + ','
            checks += '?,'
        except:
            if arg == 'status':
                values += ['Offline']
                insert += arg + ','
                checks += '?,'
            pass

    query = "INSERT INTO users (" + insert[:-1] + ") VALUES (" + checks[:-1] + ")"

    print (query)
    
    c.execute(query, values)
    db.commit()

def update_user(username):
    db = get_db()
    c = db.cursor()

    args = ['firstname', 'lastname', 'password', 'email', 'age', 'role', 'status']

    update = ''
    values = []

    for arg in args:
        try:
            values += [request.form[arg]]
            update += arg + " = ?,"
        except:
            pass

    values += [username]
    query = "UPDATE users SET " + update[:-1] + " WHERE username = ?;" 

    print(query)

    c.execute(query, values)
    db.commit()

def delete_user(username):
    db = get_db()
    c = db.cursor()

    c.execute("DELETE FROM users WHERE username = ?;", [username])
    
    db.commit()

# Chairs
def get_chairs():
    db = get_db()
    c = db.cursor()

    chairs = c.execute("SELECT * FROM chairs")
    db.commit()

    return json.dumps([dict(x) for x in chairs])

def get_chair(id):
    db = get_db()
    c = db.cursor()
    
    user = c.execute("SELECT * FROM chairs WHERE id = ?", [id])
    db.commit()

    rtUser = [dict(x) for x in user]

    if(len(rtUser) != 0):
        return json.dumps(rtUser[0])

    return json.dumps(None)

def add_chair():
    db = get_db()
    c = db.cursor()
    
    args = ['company', 'model', 'name', 'id', 'ip', 'user', 'status', 'battery']

    insert = ''
    checks = ''
    values = []

    for arg in args :
        try:
            values += [request.form[arg]]
            insert += arg + ','
            checks += '?,'
        except:
            pass

    query = "INSERT INTO chairs (" + insert[:-1] + ") VALUES (" + checks[:-1] + ")"

    print (query)
    
    c.execute(query, values)
    db.commit()

def update_chair(id):
    db = get_db()
    c = db.cursor()

    args = ['company', 'model', 'name', 'ip', 'user', 'status', 'battery']

    update = ''
    values = []

    for arg in args:
        try:
            values += [request.form[arg]]
            update += arg + " = ?,"
        except:
            pass

    values += [id]
    query = "UPDATE chairs SET " + update[:-1] + " WHERE id = ?;" 
    print(values)

    print(query)

    c.execute(query, values)
    db.commit()

def delete_chair(id):
    db = get_db()
    c = db.cursor()

    c.execute("DELETE FROM chairs WHERE id = ?", [id])
    db.commit()

# History
def get_history():
    db = get_db()
    c = db.cursor()
    
    history = c.execute("SELECT * FROM history")
    db.commit()

    return json.dumps([dict(x) for x in history])

def add_history():
    db = get_db()
    c = db.cursor()
    
    args = ['startTime', 'endTime', 'username', 'chair']

    insert = ''
    checks = ''
    values = []

    for arg in args :
        try:
            values += [request.form[arg]]
            insert += arg + ','
            checks += '?,'
        except:
            print (arg)
            pass

    query = "INSERT INTO history (" + insert[:-1] + ") VALUES (" + checks[:-1] + ")"

    print (query)
    
    c.execute(query, values)
    db.commit()

def delete_history():
    db = get_db()
    c = db.cursor()

    c.execute("DELETE FROM history;")
    
    db.commit()

# Maps
def get_allmaps():
    db = get_db()
    c = db.cursor()

    maps = c.execute("SELECT * FROM maps")
    db.commit()

    return json.dumps([dict(x) for x in maps])

def add_map():
    db = get_db()
    c = db.cursor()
    name = request.form['name']

    try:
        pgm_file = prev_dir + "/ros_ws/maps/" + name + ".pgm"
        yaml_file = prev_dir + "/ros_ws/maps/" + name + ".yaml"
    except FileNotFoundError:
        print(name + ".pgm or " + name + ".yaml not found!")

    query = "INSERT INTO maps(name, pgm_path, yaml_path) VALUES(?, ?, ?)"
    values = [name, pgm_file, yaml_file]
    
    print(query)

    c.execute(query, values)
    db.commit()

def delete_map(name):
    db = get_db()
    c = db.cursor()

    c.execute("DELETE FROM maps WHERE name = ?", [name])
    db.commit()

def get_map(name):
    db = get_db()
    c = db.cursor()
    pgm_path = ""
    
    map = c.execute("SELECT * FROM maps WHERE name = ?", [name])
    db.commit()

    rtMap = [dict(x) for x in map]

    if(len(rtMap) != 0):
        pgm_path = rtMap[0]["pgm_path"]

    if pgm_path == None:
        return "Image not found"

    im = Image.open(pgm_path)
    png_path = pgm_path.replace("pgm", "png")
    im.save(png_path)

    return png_path
    # if(len(rtMap) != 0):
    #     return json.dumps(rtMap[0])

    #return json.dumps(None)
