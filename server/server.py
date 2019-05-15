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

@app.route("/users/<username>", methods=['POST', 'GET'])
def user_update(username):
    if request.method == 'POST':
        return update_user(username)
  
    return get_user(username)

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

@app.route("/history", methods=['POST', 'GET'])
def query_history():
    if request.method == 'POST':
        add_history()
  
    return get_allhistory()

@app.route("/maps", methods=['POST', 'GET'])
def query_maps():
    if request.method == 'POST':
        add_map()
  
    return get_allmaps()

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
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text);")
        db.commit()
        print("Table 'users' created")

    rtUser = [dict(x) for x in user]

    if(len(rtUser) != 0):
        return json.dumps(rtUser[0])

    return json.dumps(None)

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
        user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
        rows = user.fetchone()

        if rows == None:
            c.execute("INSERT INTO users(firstname, lastname, username, password, email, age, gender, chair) \
                        VALUES(?, ?, ?, ?, ?, ?, ?, ?)", (firstname, lastname, username, password, email, age, gender, None))
            db.commit()
        else:
            print("User already exists!")
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")

def update_user(username):
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
        user = c.execute("SELECT * FROM users WHERE username = ?", [username,])
        rows = user.fetchone()
        if rows != None:
            c.execute("UPDATE users SET firstname = ?, lastname = ?, password = ?, email = ?, age = ?, gender = ?, chair = ? WHERE username = ?",
                (firstname, lastname, password, email, age, gender, None, username))
            db.commit()
        else:
            print("User does not exist!")
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")

@app.route("/users/chair/<username>", methods=['POST'])
def update_user_chair_state(username):
    db = get_db()
    c = db.cursor()

    chair = request.form['chair_user']

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("UPDATE users SET chair = ? WHERE username = ?",
            (chair, username))
        db.commit()
        return '',200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")  
        return '',200  

@app.route("/remove/users/<username>", methods=['POST'])
def remove_user(username):
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("DELETE FROM users WHERE username = ?", [username,])
        db.commit()
        return '', 200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS users (firstname text, lastname text, username text, password text, email text, age integer, gender text, chair text)")
        db.commit()
        print("Table 'users' created")

###### Login #### 
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
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text UNIQUE, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")

    return json.dumps([dict(x) for x in chairs])

def get_chair(id):
    db = get_db()
    c = db.cursor()

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

    rtChair = [dict(x) for x in chair]

    if(len(rtChair) != 0):
        return json.dumps(rtChair[0])

    return json.dumps(None)

@app.route("/chair/<id>", methods=['POST'])
def update_chair_status_user(id):
    db = get_db()
    c = db.cursor()
    d = db.cursor()

    user = request.form['username']
    status = request.form['status']
    battery = request.form['status']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("UPDATE chairs SET user = ?, status = ? WHERE id = ?", (user, status, id))
        db.commit()
        return '',200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")
        return '',200

def update_chair(id):
    db = get_db()
    c = db.cursor()

    company = request.form['company']
    model = request.form['model']
    name = request.form['name']
    id = request.form['id']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("UPDATE chairs SET company = ?, model = ?, name = ? WHERE id = ?", (company, model, name, id))
        db.commit()
        return '',200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")
        return '',200

def add_chair():
    db = get_db()
    c = db.cursor()

    company = request.form['company']
    model = request.form['model']
    name = request.form['name']
    id = request.form['id']
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='chairs'") # checking if the table exists
    if c.fetchone()[0]==1:
        chair = c.execute("SELECT * FROM chairs WHERE id = ?", [id,])
        rows = chair.fetchone()

        if rows == None:
            c.execute("INSERT INTO chairs(company, model, name, id, ip, user, status, battery) \
                            VALUES(?, ?, ?, ?, ?, ?, ?, ?)", (company, model, name, id, None, None, None, None))
            db.commit()
        else:
            c.execute("UPDATE chairs SET company = ?, model = ?, name = ? WHERE id = ?", (company, model, name, id))
            db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")

@app.route("/remove/chairs/<id>", methods=['POST'])
def remove_chair(id):
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='users'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("DELETE FROM chairs WHERE id = ?", [id,])
        db.commit()
        return '', 200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS chairs (company text, model text, name text, id text, ip text, user text, status text, battery integer);")
        db.commit()
        print("Table 'chairs' created")


#### History ####
def get_allhistory():
    db = get_db()
    c = db.cursor()
    history = []

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='history'") # checking if the table exists
    if c.fetchone()[0]==1:
        history = c.execute("SELECT * FROM history")
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS history (startTime text, endTime text, username text, chairId text);")
        db.commit()
        print("Table 'history' created")

    return json.dumps([dict(x) for x in history])


def add_history():      
    db = get_db()
    c = db.cursor()

    startTime = request.form['startTime']
    endTime = request.form['endTime']
    username = request.form['username']
    chairId = request.form['chairID']

    print(startTime + " " + endTime + " " + username + " " + chairId)
    
    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='history'") # checking if the table exists
    if c.fetchone()[0]==1:
        c.execute("INSERT INTO history(startTime, endTime, username, chairId) \
                        VALUES(?, ?, ?, ?)", (startTime, endTime, username, chairId))
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS history (startTime text, endTime text, username text, chairId text);")
        db.commit()
        print("Table 'history' created")
        return redirect(url_for('login'))

@app.route("/remove/history", methods=['POST'])
def remove_history():
    db = get_db()
    c = db.cursor()

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='history'") # checking if the table exists
    if c.fetchone()[0]==1:
        # c.execute("DELETE FROM history WHERE chairId = ?", [id,])
        c.execute("DELETE FROM history")
        db.commit()
        return '', 200
    else:
        c.execute("CREATE TABLE IF NOT EXISTS history (startTime text, endTime text, username text, chairId text);")
        db.commit()
        print("Table 'history' created")



##### MAPS #####
def get_allmaps():
    db = get_db()
    c = db.cursor()
    maps = []

    c.execute("SELECT count(name) FROM sqlite_master WHERE type='table' AND name='maps'") # checking if the table exists
    if c.fetchone()[0]==1:
        maps = c.execute("SELECT * FROM maps")
        db.commit()
    else:
        c.execute("CREATE TABLE IF NOT EXISTS maps (name text, image blob);")
        db.commit()
        print("Table 'maps' created")

    return json.dumps([dict(x) for x in maps])