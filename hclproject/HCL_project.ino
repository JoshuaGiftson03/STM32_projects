#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <mutex>

#define RXD2 33
#define TXD2 17

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiManager wm;

// Double buffer configuration
const int MAX_POINTS = 40;
struct Buffer {
  float frequencies[MAX_POINTS];
  float magnitudes[MAX_POINTS];
  int count = 0;
  bool ready = false;
};
Buffer buffers[2];
std::mutex bufferMutex;
volatile int activeBuffer = 0;

String webpage = R"=====(
<html>
<head>
  <title>Cabin Shout Detection</title>
  <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
  <style>
    .alert-box {
      position: fixed;
      top: 20px;
      right: 20px;
      padding: 20px;
      border-radius: 10px;
      background: ${color};
      box-shadow: 0 4px 6px rgba(0,0,0,0.1);
      display: none;
      z-index: 1000;
    }
  </style>
</head>
<body>
  <div id="chart" style="width:100%;height:100vh;"></div>
  <div id="alert" class="alert-box"></div>

  <script>
    const alertBox = document.getElementById('alert');
    let plotData = [];
    
    // Initialize plot
    Plotly.newPlot('chart', [{
      x: [],
      y: [],
      type: 'scatter',
      mode: 'lines',
      line: {color: '#007bff'}
    }], {
      title: 'Vocal Range Spectrum (50-300Hz)',
      xaxis: {title: 'Frequency (Hz)', range: [40, 350]},
      yaxis: {title: 'Magnitude',range:[0,250]},
      margin: {t: 40}
    }, {responsive: true});

    // WebSocket handler
    const ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onmessage = e => {
      const data = JSON.parse(e.data);
      
      if(data.type === 'shout') {
        alertBox.style.display = 'block';
        alertBox.style.background = data.de > data.threshold * 1.5 ? 
          '#dc3545' : '#ffc107';
        alertBox.innerHTML = `
          <h3>⚠️ Shout Detected!</h3>
          <p>Intensity: ${data.de.toFixed(2)}</p>
          <p>Threshold: ${data.threshold.toFixed(2)}</p>
        `;
        setTimeout(() => alertBox.style.display = 'none', 2000);
      }
      else {
        Plotly.update('chart', {
          x: [data.x],
          y: [data.y]
        }, {}, [0]);
      }
    };
  </script>
</body>
</html>
)=====";

void processBuffer(String &buf) {
    static uint32_t lastUpdate = 0;
    const uint32_t UPDATE_INTERVAL = 500; // Match STM32's 500ms delay
    
    if(millis() - lastUpdate < UPDATE_INTERVAL) return;
    lastUpdate = millis();

    if(buf.startsWith("ALERT:")) {
        // Parse alert data
        int colonPos = buf.indexOf(':');
        int slashPos = buf.indexOf('/');
        
        float current = buf.substring(colonPos+1, slashPos).toFloat();
        float average = buf.substring(slashPos+1).toFloat();
        
        // Send to web interface
        String json = "{\"alert\":1,\"current\":" + String(current) + 
                     ",\"average\":" + String(average) + "}";
        webSocket.broadcastTXT(json);
        Serial.println("Alert received");
        return;
    }

  // Process normal FFT data
  if(buf.length() < 5) {
    Serial.println("[ERROR] Data too short: " + buf);
    return;
  }

  std::lock_guard<std::mutex> lock(bufferMutex);
  Buffer &current = buffers[activeBuffer];
  current.count = 0;
  
  buf.toLowerCase();
  
while(buf.length() > 0 && current.count < MAX_POINTS) {
        int spacePos = buf.indexOf(' ');
        String pair = (spacePos == -1) ? buf : buf.substring(0, spacePos);
        
        int colonPos = pair.indexOf(':');
        if(colonPos != -1) {
            float freq = pair.substring(0, colonPos).toFloat();
            
            // Only store vocal range frequencies
            if(freq >= 50 && freq <= 300) {
                current.frequencies[current.count] = freq;
                current.magnitudes[current.count] = pair.substring(colonPos+1).toFloat();
                current.count++;
            }
        }
        
        if(spacePos == -1) break;
        buf = buf.substring(spacePos+1);
    }
    

    if(current.count > 0) {
        current.ready = true;
        activeBuffer = (activeBuffer + 1) % 2;
    }
}

void serialTask(void *pvParameters) {
  String buffer;
  uint32_t lastReceived = 0;
  
  while(1) {
    while(Serial2.available()) {
      char c = Serial2.read();
      lastReceived = millis();
      
      if(c == '\n' || c == '\r') {
        buffer.trim();
        if(buffer.length() > 0) {
          processBuffer(buffer);
        }
        buffer = "";
      } else if (c != '\r') {
        buffer += c;
      }
    }

    if(buffer.length() > 0 && (millis() - lastReceived > 100)) {
      Serial.println("[WARNING] Flushing incomplete data: " + buffer);
      buffer = "";
    }
    
    delay(1);
  }
}

void sendWebSocketData() {
  static uint32_t lastSend = 0;
  if(millis() - lastSend < 50) return;

  std::lock_guard<std::mutex> lock(bufferMutex);
  for(int i = 0; i < 2; i++) {
    if(buffers[i].ready) {
      String json = "{\"x\":[";
      for(int j=0; j<buffers[i].count; j++) {
        json += String(buffers[i].frequencies[j]);
        if(j < buffers[i].count-1) json += ",";
      }
      json += "],\"y\":[";
      for(int j=0; j<buffers[i].count; j++) {
        json += String(buffers[i].magnitudes[j]);
        if(j < buffers[i].count-1) json += ",";
      }
      json += "]}";
      
      webSocket.broadcastTXT(json);
      buffers[i].ready = false;
      lastSend = millis();
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  wm.setConnectTimeout(30);
  if(!wm.autoConnect("STM32-Spectrum")) {
    Serial.println("Restarting...");
    ESP.restart();
  }

  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  
  webSocket.begin();
  server.begin();

  xTaskCreatePinnedToCore(
    serialTask,
    "SerialTask",
    10000,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
  server.handleClient();
  webSocket.loop();
  sendWebSocketData();
}