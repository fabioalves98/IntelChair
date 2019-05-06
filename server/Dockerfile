FROM python:3

WORKDIR /pei-2018-2019-g07/server

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD [ "python", "./run.py" ]