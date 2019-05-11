import sqlite3
from flask import Flask, g
from flask import render_template
from flask import redirect, url_for, request

app = Flask(__name__)
DATABASE = "users.db"

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
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
	# username = request.form['username']
	# password = request.form['password']

	if request.method == 'POST':
		if valid_login(username, password):
			return log_the_user_in()
		else:
			error = "Invalid username or password"


	return render_template('auth.html') # executed if method = GET


@app.route("/index")
def index():
	return render_template('index.html')

def valid_login(username, password):
	db = get_db()
	user = db.execute('select * from users where username = ?', username)
	if user is None:
		return False

	return True

def log_the_user_in():
	return redirect(url_for('index'))

def query_db(query, args=(), one=False):
    cur = get_db().execute(query, args)
    rv = cur.fetchall()
    cur.close()
    return (rv[0] if rv else None) if one else rv
