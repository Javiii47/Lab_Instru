#include <WiFi.h>

const char* ssid = "Coche-test";
const char* password = "123456789";


WiFiServer tcpServer(8080); // Listen on port 8080

void setup() {
   Serial.begin(115200);
   delay(1000);

// Connect to Wi-Fi
   //WiFi.begin(ssid, password);            //no AP
   WiFi.softAP(ssid, password);
   Serial.print("Connecting to WiFi...");
   //while (WiFi.status() != WL_CONNECTED) {
   //   delay(500);
   //   Serial.print(".");
   //}
   Serial.println(" connected!");
   Serial.println("IP address: ");
   //Serial.println(WiFi.localIP());        //no AP
   Serial.println(WiFi.softAPIP());

// Start the server
   tcpServer.begin();
   Serial.println("TCP server started on port 8080");
}

void loop() {
   WiFiClient client = tcpServer.available(); // Check for incoming clients

   if (client) {
      Serial.println("Client connected!");
      while (client.connected()) {
         if (client.available()) {
            String data = client.readStringUntil('\n');
            Serial.print("Received: ");
            Serial.println(data);
            client.println("Data received."); // Echo back
         }
      }
      client.stop();
      Serial.println("Client disconnected.");
   }
}