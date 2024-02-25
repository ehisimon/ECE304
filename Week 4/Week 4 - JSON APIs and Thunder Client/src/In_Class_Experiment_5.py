import requests, time, json

address = 'http://192.168.1.94'

# Connect to get status
r1 = requests.get(address)
print ('*****Getting Circuit Status*****')
print ('JSON from Response Body =', r1.json())
print ('Headers =', r1.headers)
print ('URL =', r1.url)
print ('Content =', r1.content)
print ('Encoding =', r1.encoding)
print ('raw =', r1.raw)
print ('request =', r1.request)
print ('status_code =', r1.status_code)
print ('text =', r1.text)
print ()
input('Press Any Key to Continue')

print()
print ('*****Changing Blue LED Status*****')
r2 = (requests.get(address+'/ledchange'))
print ('JSON from Response Body = ', r2.json())
print ('Request = ', r2.request)
print ('Text = ', r2.text)
input('Press Any Key to Continue')

print()
print ('*****Changing Blue LED to OFF and Red LED to 100*****')
payload1={'blueled':0,'redled':100}
r3 = requests.get(address+'/led_set_get',params=payload1)
print ('JSON from Response Body =', r3.json())
print('Request Text =', r3.text)
print ('Request results =', r3.request)

print()
print ('*****Changing Red LED Intensity to 250*****')
payload1={'redled':250}
r4 = requests.post(address+'/led_set_post',data=json.dumps(payload1))
print ('JSON from Response Body =', r4.json())
print('Request Text =', r4.text)
print ('Request results =', r4.request)
input('Press Any Key to Continue')

print()
print ('*****Turn off both LEDs*****')
payload1={'blueled':0,'redled':0}
r5 = requests.post(address+'/led_set_post',data=json.dumps(payload1))
print ('JSON from Response Body =', r5.json())
print('Request Text =', r5.text)
print ('Request results =', r5.request)
