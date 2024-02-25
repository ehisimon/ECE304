import requests
import time

address = '192.168.1.94'
page = '/ledchange'

req=requests.get('http://' + address + page)
print(req.status_code)
print(req.headers['Content-Type'])
print(req.text)
print(req.json())