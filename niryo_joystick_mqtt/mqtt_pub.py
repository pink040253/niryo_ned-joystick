import paho.mqtt.client as mqtt 
from random import randrange, uniform
import time

mqttBroker ="10.10.10.110" 

client = mqtt.Client("Temperature_Inside")
client.connect(mqttBroker) 
client.username_pw_set("niryo","robotics")

while True:
    randNumber = uniform(20.0, 21.0)
    client.publish("TEMPERATURE", randNumber)
    print("Just published " + str(randNumber) + " to topic TEMPERATURE")
    time.sleep(1)