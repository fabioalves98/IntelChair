# IntelChair API

## Prerequisites

- **SQLite3:** sudo apt install sqlite3
- **Flask:** sudo pip3 install Flask
- **Flask-Cors:** sudo pip3 install -U flask-cors


## How to run

- **In a new terminal run:** ./run_server.sh

# Users
### List all users

**Definition**

`GET /users`

**Response**

- **`200 OK`** on success


[
    {
        "firstname": "Manuel",
        "lastname": "Coelho",
        "username": "manuelcoelho",
        "password": "123",
        "email": "manuelcoelho@ua.pt",
        "age": 37,
        "role": "admin",
        "status": "online"
    },
    {
        "firstname": "Francisco",
        "lastname": "Alves",
        "username": "fralves",
        "password": "456",
        "email": "fralves@ua.pt",
        "age": 28,
        "role": "user",
        "status": "offline"
    }
]

<br>

### Registering a new user

**Definition**

`POST /users`

**Arguments**

- `"firstname":string`
- `"lastname":string`
- `"username":string`
- `"password"':string`
- `"email":string`
- `"age":int`


If a user with the given username already exists, the server will not add or overwrite.

**Response**

- **`200 OK`** on success

<br>

### Updating an user

**Definition**

`PUT /users/<username>`

**Arguments**

- `"firstname":string`
- `"lastname":string`
- `"username":string`
- `"password"':string`
- `"email":string`
- `"age":int`

**Response**

- **`400 Not Found`** if the user does not exist
- **`200 OK`** on success

<br>

### Lookup user details

`GET /users/<username>`

**Response**

- **`404 Not Found`** if the user does not exist
- **`200 OK`** on success


{
    "firstname": "Manuel",
    "lastname": "Coelho",
    "username": "manuelcoelho",
    "password": "123",
    "email": "manuelcoelho@ua.pt",
    "age": 37,
    "role": "admin",
    "status": "online"
}


### Delete a user

**Definition**

`DELETE /users/<username>`

**Response**

- **`404 Not Found`** if the user does not exist
- **`200 OK`** on success

<br>

# Chairs
### List all chairs

**Definition**

`GET /chairs`

**Response**

- **`200 OK`** on success


[
    {
        "company": "Karma",
        "model": "RX123",
        "name": "IntelChair",
        "id": "123123",
        "ip": "192.168.43.122",
        "user": "manuelcoelho",
        "status": "Online",
        "battery": "70%"
    },
    {
        "company": "Karma",
        "model": "RX123",
        "name": "IntelChair2",
        "id": "725272",
        "ip": "null",
        "user": "null",
        "status": "Offline",
        "battery": "null"
    }
]

<br>

### Registering a new chair

**Definition**

`POST /chairs`

**Arguments**

- `"company":string`
- `"model":string`
- `"name":string`
- `"id":string`


If a chair with the given name already exists, the server will not add or overwrite.

**Response**

- **`200 OK`** on success

<br>

### Updating a chair

**Definition**

`PUT /chairs/<id>`

**Arguments**

- `"company":string`
- `"model":string`
- `"name":string`
- `"id":string`

**Response**

- **`400 Not Found`** if the chair does not exist
- **`200 OK`** on success

<br>

### Lookup chair details

**Definition**

`GET /chairs/<id>`

**Response**

- **`404 Not Found`** if the chair does not exist
- **`200 OK`** on success


{
    "company": "Karma",
    "model": "RX123",
    "name": "IntelChair",
    "id": "123123",
    "ip": "192.168.43.122",
    "user": "manuelcoelho",
    "status": "Online",
    "battery": "70%"
}

<br>

### Delete a chair

**Definition**

`DELETE /chairs/<id>`

**Response**

- **`404 Not Found`** if the chair does not exist
- **`200 OK`** on success

<br>

# History
### List history

**Definition**

`GET /history`

**Response**

- **`200 OK`** on success

<br>

### Add history

**Definition**

`POST /history`

**Arguments**

- `startTime:string`
- `endTime:string`
- `username:string`
- `chair:string`

**Response**

- **`200 OK`** on success

<br>

# Maps
### List maps

**Definition**

`GET /maps`

**Response**

- **`200 OK`** on success

<br>

### Add map

**Definition**

`POST /maps`

**Arguments**

- `name:string`
- `pgm_path:string`
- `yaml_path:string`

**Response**

- **`200 OK`** on success
