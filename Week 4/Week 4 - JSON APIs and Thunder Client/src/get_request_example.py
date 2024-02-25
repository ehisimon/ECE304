import requests

# Send GET request to website
r = requests.get('https://httpbin.org/get')

# Display the status code
status = r.status_code
print ("Status:",status)
print ()

# Display the header
header = r.headers
print ("Header: ", header)
print ()

# Display the content type
print ("Content Type: ",r.headers['Content-Type'])
print ()

# Use key values in request
ploads = {'things':2,'total':25}
r = requests.get('https://httpbin.org/get',params=ploads)
print ("Text: ",r.text)
print ()
print ("url: ", r.url)

