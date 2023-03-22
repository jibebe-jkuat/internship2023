#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define RXD2 16
#define TXD2 17

const char* ssid = "Betty'wifi";
const char* passwd = "@Challow???";
const char* mqtt_server = "192.168.43.251";

const char *inTopic = "intopic";
const char *outTopic = "outtopic";
int led = 18;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  50
char msg[MSG_BUFFER_SIZE];


int value = 0;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //Serial.begin(115200);
  WiFi.begin(ssid, passwd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());// Function not known

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message arrived [");
  Serial.println(topic);
  
  
  // Convert the payload byte array to a string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // Call the function on the Arduino Uno and pass the message as a parameter
  Serial1.print("transmitting message to arduino");
  Serial1.println(message);
  Serial1.println();
  
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Create a random client ID
      String clientId = "FCMCU";
      clientId += String(random(0xffff), HEX);
      // Attempt to connect
      if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        // Once connected, publish an announcement...
//        client.publish("outTopic", "Start");
        // ... and resubscribe
        client.subscribe(outTopic);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5);
      }
//      manualOverride();
    }
  }


void setup() {
  // put your setup code here, to run once Serial.begin(9600);
  //  input_password.reserve(4); // maximum input characters is 33, change if needed
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200);
//  buttonState=digitalRead(button);
  pinMode(led, OUTPUT);
//  pinMode(button, INPUT);
  Serial.print("\n\nStarting program");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
  void loop() {
    
    if (!client.connected())  // Reconnect if connection is lost
    {
      reconnect();
    }
    client.loop();
//     manualOverride();
  }
