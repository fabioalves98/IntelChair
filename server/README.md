# IntelChair API

## Prerequisites

- **Docker**:
sudo apt-get install docker.io

- **Docker-compose**:
sudo apt-get install docker-compose

## How to run

**In a new terminal run these commands:**<br>

If the docker has been modified:<br>

- $ sudo docker-compose build<br>

Then, to run the application:<br>

- $ sudo docker-compose up

## Usage


All responses will have the form

```json
{
    "data": "Mixed type holding the content of the response",
    "message": "Description of what happened"
}
```

Subsequent response definitions will only detail the expected value of the `data field`
# Users
### List all users

**Definition**

`GET /users`

**Response**

- `200 OK` on success

```json
[
    {
        "first-name": "Manuel",
        "last-name": "Coelho",
        "username": "manuelcoelho",
        "email": "manuelcoelho@ua.pt",
        "age": 37,
        "gender": "M"
    },
    {
        "first-name": "Francisco",
        "last-name": "Alves",
        "username": "fralves",
        "email": "fralves@ua.pt",
        "age": 28,
        "gender": "M"
    }
]
```

### Registering a new user

**Definition**

`POST /users`

**Arguments**

- `"first-name":string`
- `"last-name":string`
- `"username":string`
- `"email":string`
- `"age":int`
- `"gender":char`


If a user with the given username already exists, the existing user will be overwritten.

**Response**

- `201 Created` on success

```json
{
    "first-name": "Manuel",
    "last-name": "Coelho",
    "username": "manuelcoelho",
    "email": "manuelcoelho@ua.pt",
    "age": 37,
    "gender": "M"
}
```

## Lookup user details

`GET /users/<username>`

**Response**

- `404 Not Found` if the user does not exist
- `200 OK` on success

```json
{
    "first-name": "Manuel",
    "last-name": "Coelho",
    "username": "manuelcoelho",
    "email": "manuelcoelho@ua.pt",
    "age": 37,
    "gender": "M"
}
```

## Delete a user

**Definition**

`DELETE /users/<username>`

**Response**

- `404 Not Found` if the user does not exist
- `204 No Content` on success


# Chairs
### List all chairs

**Definition**

`GET /chairs`

**Response**

- `200 OK` on success

```json
[
    {
        "company": "Karma",
        "model": "RX123",
        "name": "wheelchair1",
        "id": "a1bc2",
        "user": "none"
    },
    {
        "company": "Karma",
        "model": "RX123",
        "name": "wheelchair2",
        "id": "u74ao",
        "user": "manuelcoelho"
    }
]
```

### Registering a new chair

**Definition**

`POST /chairs`

**Arguments**

- `"company":string`
- `"model":string`
- `"name":string`
- `"id":string`
- `"user":string`


If a chair with the given name already exists, the existing chair will be overwritten.

**Response**

- `201 Created` on success

```json
{
    "company": "Karma",
    "model": "RX123",
    "name": "wheelchair1",
    "id": "a1bc2",
    "user": "none"
}
```

## Lookup chair details

`GET /chairs/<name>`

**Response**

- `404 Not Found` if the chair does not exist
- `200 OK` on success

```json
{
    "company": "Karma",
    "model": "RX123",
    "name": "wheelchair1",
    "id": "a1bc2",
    "user": "none"
}
```

## Delete a chair

**Definition**

`DELETE /chairs/<name>`

**Response**

- `404 Not Found` if the chair does not exist
- `204 No Content` on success

