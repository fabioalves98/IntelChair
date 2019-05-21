#!/bin/bash
export FLASK_APP=server.py
#export FLASK_ENV=development
rm database.db
flask run --host=0.0.0.0