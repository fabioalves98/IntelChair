# IntelChair API

## Usage

All responses will have the form

```json
{
    "data": "Mixed type holding the content of the response",
    "message": "Description of what happened"
}
```

Subsequent response definitions will only detail the expected value of the `data field`

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

- `"first-name":string` user's first name
- `"last-name":string` user's last name
- `"username":string` username on the platform
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
