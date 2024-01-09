import paho.mqtt.client as paho
import time
import random
import Connect;

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("test")

def on_message(client, userdata, msg):
    print(msg.topic, msg.payload)

PythonClient = paho.Client()
print("Łączenie")
PythonClient.on_connect = on_connect
PythonClient.on_message = on_message

# PythonClient.username_pw_set(Connect.Login, Connect.Password)
PythonClient.connect(Connect.MosquittoBroker, Connect.MosquittoPort, 60)

print("Połączono")

PythonClient.subscribe("Topic/subtopic1")
PythonClient.subscribe("Topic/subtopic2")
PythonClient.subscribe("Topic/subtopic3")
print("Wysyłanie wiadomości")
# PubRet = PythonClient.publish("PythonTest", "TestMessage")
while(True):
    # r = random.randint(200, 300)
    # PythonClient.publish("Voltage", str(r))
    # print("{} V".format(r))


    # r = random.randint(2, 10)
    # PythonClient.publish("Current", str(r))
    # print("{} A".format(r))

    
    # r = random.randint(500, 550)
    # PythonClient.publish("Power", str(r))
    # print("{} W".format(r))

    # time.sleep(10)
    PythonClient.loop_forever()