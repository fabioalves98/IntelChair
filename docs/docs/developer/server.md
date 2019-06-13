# Server

The server is the centerpiece of our system, as ensures all the communications between the web application, the wheelchair, the dashboard and the database. It is what connects everything in order to have a functional product. We decided to use [Flask](http://flask.pocoo.org/docs/1.0/) to build our server and [SQLite](https://www.sqlite.org/index.html) to manage our database. Before you test anything, make sure you install the packages at the Prerequisites section below.

## Prerequisites

* SQLite 3
* Flask with flask-cors

## How to run

- **In a new terminal run:** ./run_server.sh<br>
This is a simple script to bring the server online.

## What does it feature?

You can interact with 4 entities:

- **Users**
- **Chairs**
- **History of usage**
- **Maps**

## Users
### List all users

**Definition**

`GET /users`

### Registering a new user

**Definition**

`POST /users`

**Arguments**

`"firstname":string` -
`"lastname":string` -
`"username":string` -
`"password"':string` -
`"email":string` -
`"age":int` -
`"role":string` -
`"status":string`


If a user with the given username already exists, the server will not add or overwrite.


### Updating an user

**Definition**

`PUT /users/<username>`

**Arguments**

`"firstname":string` -
`"lastname":string` -
`"password"':string` -
`"email":string` -
`"age":int` - 
`"role":string` -
`"status":string`


### Lookup user details

**Definition**

`GET /users/<username>`

### Delete a user

**Definition**

`DELETE /users/<username>`

<br>

## Chairs
### List all chairs

**Definition**

`GET /chairs`

### Registering a new chair

**Definition**

`POST /chairs`

**Arguments**

`"company":string` -
`"model":string` -
`"name":string` -
`"id":string` - 
`"ip":string` -
`"user":string` -
`"status":string` -
`"battery":integer`


If a chair with the given name already exists, the server will not add or overwrite.


### Updating a chair

**Definition**

`PUT /chairs/<id>`

**Arguments**

`"company":string` -
`"model":string` -
`"name":string` -
`"ip":string` -
`"user":string` -
`"status":string` -
`"battery":integer`


### Lookup chair details

**Definition**

`GET /chairs/<id>`


### Delete a chair

**Definition**

`DELETE /chairs/<id>`

<br>

## History
### List history

**Definition**

`GET /history`

### Add history

**Definition**

`POST /history`

**Arguments**

`startTime:string` -
`endTime:string` -
`username:string` -
`chair:string`


### Delete history

**Definition**

`DELETE /history`

<br>

## Maps
### List all maps

**Definition**

`GET /maps`


### Add map

**Definition**

`POST /maps`

**Arguments**

`name:string` -
`pgm_path:string`

### Update map

**Definition**

`PUT /maps/<name>`

**Arguments**

`"name":string` -
`"pgm_path":string`


### Lookup specific map

**Definition**

`GET maps/<name>`


### Delete map

**Definition**

`DELETE /maps/<name>`


If the server deployment was completed successfuly you can proceed to the [wheelchair](wheelchair.md) system.