import requests
import json
import RPi.GPIO as GPIO
import time

#set up gpio
GPIO.setmode(GPIO.BCM)
power_pin = 18
small_button_pin = 24 
large_button_pin = 23
GPIO.setup(power_pin, GPIO.OUT)
GPIO.setup(small_button_pin, GPIO.OUT)
GPIO.setup(large_button_pin, GPIO.OUT)

#set up airtable
URL_COFFEE = 'https://api.airtable.com/v0/applK0NRaebYim73c/Control_table'
ACCESS_TOKEN_COFFEE = {Access Code Here}
HEADERS_COFFEE = {'Authorization': 'Bearer ' + ACCESS_TOKEN_COFFEE}

URL_INTEGRATION = 'https://api.airtable.com/v0/appZXeS3vQKy6x41E/Drive'
ACCESS_TOKEN = {Access Code Here}
HEADERS = {'Authorization': 'Bearer ' + ACCESS_TOKEN}

def airtable_requests_coffee():
    response = requests.get(url=URL_COFFEE, headers=HEADERS_COFFEE)
    
    if response.status_code == 200:
        data = response.json()
        records = data['records']
        if records:
            button_states = {}
            for record in records:
                fields = record['fields']
                if 'Name' in fields and 'Value' in fields:
                    button_name = fields['Name']
                    button_value = fields['Value']
                    button_states[button_name] = button_value

            print("Button States:", button_states)
            return button_states, button_name, button_value
        else:
            print("No records found in response.")
            return {}
    else:
        print(f"Failed to fetch data: {response.text}")
        return {}
    
def airtable_requests_transport(id):
    response = requests.get(url=f"{URL_INTEGRATION}/{id}", headers=HEADERS)
    
    if response.status_code == 200:
        data = response.json()
        if data:
            fields = data.get('fields', {})
            design_value = fields.get('Design')

            if design_value == 'Large':
                return('Large')
            elif design_value == 'Small':
                return('Small')
            else:
                return('Unknown')
        else:
            print("No record found in response.")
            return {}
    else:
        print(f"Failed to fetch data: {response.text}")
        return {}


def button_status(response, value):
    if response == 'Power button' and value == 1:
            print ('power button pressed')
            return 'power'
    elif response == 'small button' and value == 1:
            print ('small button pressed')
            return 'small'
    elif response == 'large button' and value == 1:
            print ('large button pressed')
            return 'large'
    elif response == 'ClosedWithLID' and value == 1:
            return 'start'
    else:
            print ('no buttons pressed, ready to begin')
            return 'none'

def power_on():
        GPIO.output(power_pin, GPIO.HIGH)
        time.sleep(5)
        update_airtable_coffee('recC8aATqiAUW6DEU', 1, URL_COFFEE, HEADERS_COFFEE)
        return '1'

def coffee_buttons(power_status, size):
    if power_status == '1':
        for i in range(30, 0, -1):
            print(f"Time remaining: {i} seconds")
            time.sleep(1)

        if size == 'Large':
            GPIO.output(large_button_pin, GPIO.HIGH)
            time.sleep(10)
            GPIO.output(large_button_pin, GPIO.LOW)
            update_airtable_coffee('recG53DGW7cPE2nfA', 1, URL_COFFEE, HEADERS_COFFEE)
            print('coffee button pressed')
            return 'success'
        elif size == 'Small':    
            GPIO.output(small_button_pin, GPIO.HIGH)
            time.sleep(10)
            GPIO.output(small_button_pin, GPIO.LOW)
            update_airtable_coffee('recG53DGW7cPE2nfA', 1, URL_COFFEE, HEADERS_COFFEE)
            print('coffee button pressed')
            return 'success'
        else:
             print('size not read')

    else:
         print('error when pressing coffee button')
         return 'error'
def update_airtable_coffee(id,new_value, URL, headers):
    try:
        # data payload for updating the record
        data = {'fields': {'Value': int(new_value)}}
        response = requests.patch(f'{URL}/{id}', headers=headers, json=data)
        # Check for success
        if response.status_code == 200:
            print("Airtable value updated successfully")
        else:
            print("Error updating Airtable value:", response.status_code)
    except Exception as e:
        print("Exception occurred while updating Airtable:", str(e))

def update_airtable_int(id,new_value, URL, headers):
    try:
        data = {'fields': {'Value': int(new_value)}}
        response = requests.patch(f'{URL}/{id}', headers=headers, json=data)
        if response.status_code == 200:
            print("Airtable value updated successfully")
        else:
            print("Error updating Airtable value:", response.status_code)
    except Exception as e:
        print("Exception occurred while updating Airtable:", str(e))


def update_airtable_integration(record_id, field_name, new_value):
    customer_info = {"fields": {field_name: new_value}}
    print ('field name:', field_name)
    update_url = f"{URL_INTEGRATION}/{record_id}"
    response = requests.patch(update_url, json=customer_info, headers=HEADERS)
def update_airtable_integration_F(record_id, field_name, new_value):
    customer_info = {"fields": {field_name: new_value}}
    update_url = f"{URL_INTEGRATION}/{record_id}"
    print(f"URL: {update_url}")
    print(f"Headers: {HEADERS}")
    print(f"Payload: {customer_info}")
    response = requests.patch(update_url, json=customer_info, headers=HEADERS)
    if response.status_code != 200:
        print(f"Failed to update Airtable: {response.status_code}, {response.text}")
    else:
        print("Airtable updated successfully")

def coffee_done(input):
    print("Input value:", input) 
    if input == 'success':
            print("Coffee is successful. Starting countdown timer...")
            for i in range(50, 0, -1):
                print(f"Time remaining till coffee done: {i} seconds")
                time.sleep(1)
            update_airtable_coffee('recC8aATqiAUW6DEU', 1, URL_INTEGRATION, HEADERS)
    else:
        print('Error in coffee_done: input is not "success"')

def reset_airtable():
    update_airtable_coffee('recC8aATqiAUW6DEU', 0, URL_COFFEE, HEADERS_COFFEE) #power status
    update_airtable_coffee('recG53DGW7cPE2nfA', 0, URL_COFFEE, HEADERS_COFFEE) #coffee button
    update_airtable_coffee('recGzal1MiC7xAezn', 0, URL_COFFEE, HEADERS_COFFEE) # coffee done

try:
    while True:
        reset_airtable()
        print('Starting while loop')
        button_states, button_name, button_value = airtable_requests_coffee()
        print('Button states updated')
        print(button_states)
        #button_states = True
        print ('name:', button_name)
        print('value', button_value)
        starting_status = button_status(button_name, button_value)
        print(starting_status)

        size_value = airtable_requests_transport('recY2LOdyKyHehK5Y')
        print(size_value)
        if starting_status == 'start':  # Ensure we have data to process
            print ('ready to start')
            power_status = power_on()
            print('power status:', power_status)
            coffee_status = coffee_buttons(power_status, size_value)
            print('coffee status:', coffee_status)
            if coffee_status == 'success':
                coffee_done(coffee_status)
                GPIO.output(power_pin, GPIO.LOW)
                continue
            update_airtable_integration_F('recY2LOdyKyHehK5Y', 'Done', 1) #coffee done airtable
            update_airtable_coffee('recGzal1MiC7xAezn', 1, URL_COFFEE, HEADERS_COFFEE) #set coffee done localll=y
            update_airtable_coffee('rechYVbPiXBVTHqHx', 0, URL_COFFEE, HEADERS_COFFEE) #closed with lid reset
            print('closed lid reset')
        else:
            print("Error getting stuff from airtable")
except KeyboardInterrupt:
    GPIO.cleanup()









