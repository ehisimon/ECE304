from flask import Flask, render_template, request
import requests
import json

ESP32_address = 'http://192.168.1.94'
network_visible = True

app = Flask(__name__)

@app.route('/')
def get_information():
    r = requests.get(ESP32_address + '/')
    data = r.json()
    i1 = data['id']
    i2 = "{:0.2f}".format(data['Temperature'])
    i3 = "{:0.2f}".format(data['Humidity'])
    i4 = "{:0.2f}".format(data['Pressure'])
    i5 = "{:0.2f}".format(data['Altitude'])

    return render_template('LED_and_Climate_Information.html', id=i1, temp=i2, humid=i3, press=i4, alt=15)


@app.route('/led', methods=["POST", "GET"])
def setLED():
    if request.method == "POST":
        if request.form.get("blueled"):
            blue_led_val = 1
        else:
            blue_led_val = 0

        if not (request.form["redled"]):
            red_led_val = 0
        else:
            red_led_val = int(request.form["redled"])

        data1 = json.dumps({'blue_led': blue_led_val, 'red_led': red_led_val})
        r = requests.post(ESP32_address + '/led', data=data1)

        return render_template('LED_Commander.html')
    else:
        return render_template('LED_Commander.html')

if __name__ == '__main__':
   if network_visible:	
     app.run(debug=True,host='0.0.0.0')
   else:
     app.run(debug=True)		
