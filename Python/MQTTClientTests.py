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
print("Wysyłanie wiadomości")
# PubRet = PythonClient.publish("PythonTest", "TestMessage")
while(True):
    r = random.randint(0, 100)
    PythonClient.publish("testConnection", str(r))
    print("{} V".format(r))
    time.sleep(5)

    r = random.randint(2, 20)
    PythonClient.publish("Current", str(r))
    print("{} A".format(r))
    time.sleep(5)
    
    r = random.randint(500, 700)
    PythonClient.publish("Power", str(r))
    print("{} W".format(r))
    time.sleep(5)
    # PythonClient.publish("test", str(r))