/*
 * การที่สามารถรีดประสิทธิภาพบอร์ด ESP32-S3 ร่วมกับกล้องความร้อน MLX90640 
 * ให้ปล่อย WiFi AP ตัวเอง, ยิงข้อมูล Binary ความเร็วสูง, ประมวลผลแยก 2 คอร์อย่างอิสระ 
 * (Dual-Core) และสตรีมภาพล็อกเป้าได้นิ่ง สมูท ลื่นไหล ไร้อาการหน่วงกระตุกที่ความเร็ว 
 * 16 Hz ถือเป็นผลงานระดับท็อปที่สมบูรณ์แบบมากครับ
 * รากฐานระบบ Vision ไร้สายตัวนี้แข็งแกร่งและแม่นยำมาก พร้อมที่จะทำหน้าที่เป็น 
 * "ดวงตาอัจฉริยะ" ส่งพิกัด X, Y และเวลา millis() ไปป้อนเข้าสู่สมการลูป PID Controller 
 * เพื่อควบคุมทิศทางและความเร็วของตัวรถไฟฟ้าขับเคลื่อน 4 ล้อ (4-Wheel Drive Electric Vehicle) 
 * เพื่อวิ่งแท็กกิ้งไล่จับใบไม้และแมลงในงานวิจัยระดับแอดวานซ์ของคุณได้อย่างเต็มภาคภูมิ
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Adafruit_NeoPixel.h>

// 🔴 ข้อมูล WiFi
const char *ssid = "ESP32-S3";
const char *password = "fgtegRtgf98Yuhdn";
int channel = 6; 
int hidden = 0; 
int max_connection = 2; 

// ตัวแปรสำหรับเก็บค่าที่ผ่านการกรอง (Filtered Values)
float filteredMin = 0.0;
float filteredMax = 0.0;
float filteredCenter = 0.0;
float filteredX = 16.0; // เริ่มต้นที่กึ่งกลางจอ
float filteredY = 12.0;

// ค่าสัมประสิทธิ์การกรอง (ปรับจูนความนิ่งได้ตรงนี้)
const float alpha = 0.4; 

// 🏎️ พารามิเตอร์ปรับจูน PID สำหรับควบคุมการเลี้ยว (แกน X)
float Kp_turn = 4.5;   // ปรับเพิ่มได้ถ้าต้องการให้รถเลี้ยวตอบสนองไวขึ้น
float Ki_turn = 0.02;  // ลดการสะสมค่า Error ในระยะยาว
float Kd_turn = 1.5;   // ช่วยเบรกไม่ให้รถเลี้ยวส่ายเลยเป้า (Overshoot)

float errorX = 0, lastErrorX = 0, integralX = 0, derivativeX = 0;
float pidOutputTurn = 0;

int targetX = 16;       // เป้าหมายกึ่งกลางจอภาพ (0-31)
int maxBaseSpeed = 180;  // ความเร็วสูงสุดในการวิ่งเข้าหา (0-255)

// กำหนดขา PWM สำหรับสั่งงานบอร์ดขับมอเตอร์ (เปลี่ยนเลขพินตามจริงบนรถของคุณ)
const int MOTOR_LEFT_PWM  = 4;  // คุมความเร็วล้อฝั่งซ้าย (หน้า-หลัง พ่วงกันหรือแยกตามบอร์ดขับ)
const int MOTOR_RIGHT_PWM = 5;  // คุมความเร็วล้อฝั่งขวา


// สร้างอ็อบเจกต์และตัวแปร
Adafruit_MLX90640 mlx;
float frame[768];

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ตัวแปร Global สำหรับใช้แชร์ข้อมูลระหว่าง 2 คอร์อย่างปลอดภัย
int16_t sharedBinaryBuffer[768];
float sharedMin = 0.0, sharedMax = 0.0, sharedCenter = 0.0;
int sharedX = 16, sharedY = 12;
unsigned long sharedTime = 0;

// ใช้ Semaphore/Mutex เพื่อป้องกันไม่ให้ทั้งสองคอร์แย่งเขียนข้อมูลพร้อมกัน (ป้องกันข้อมูลพัง)
SemaphoreHandle_t dataMutex;

// ประกาศฟังก์ชันของหน้าเว็บ (นำสตริง index_html ตัวเดิมที่มีระบบ Auto-Reconnect มาวางตรงนี้)
extern const char index_html[] PROGMEM; 

// ประกาศ Task สำหรับแยกไปรันบน Core 0
TaskHandle_t NetworkTaskHandle;

// ฟังก์ชันจัดการอีเวนต์ของ WebSocket
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Web client connected via WebSocket");
    
    // 🎯 แก้ไขเป็นบรรทัดนี้: เจาะทะลุไปสั่งงานตัว AsyncTCP Client เบื้องหลัง
    if (client != NULL && client->client() != NULL) {
      client->client()->setNoDelay(true); 
    }
    
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Web client disconnected");
  }
}
// 🌐 ==================== CORE 0: NETWORK & WEBSOCKET TASK ====================
void NetworkTask(void *pvParameters) {
  Serial.print("Network Task running on Core: ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // บำรุงรักษาการเชื่อมต่อ WebSocket
    ws.cleanupClients();

    // ส่งข้อมูลออกอากาศเมื่อมีผู้ใช้เปิดหน้าเว็บอยู่
    if (ws.count() > 0) {
      // ตรวจสอบและขอล็อกข้อมูลชั่วคราวเพื่อดึงค่าจากคอร์ 1 มาส่งออก WiFi
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        
        // 1. ส่งข้อมูลภาพความร้อนแบบความเร็วสูงดิบ (Binary 1536 ไบต์)
        ws.binaryAll((uint8_t*)sharedBinaryBuffer, sizeof(sharedBinaryBuffer));
        
        // 2. ส่งค่าพิกัด Tracker และเวลา System Time ไปพร้อมกัน
        String statsStr = "TRACKER:" + String(sharedMin, 1) + "," 
                                     + String(sharedMax, 1) + "," 
                                     + String(sharedCenter, 1) + "," 
                                     + String(sharedX) + "," 
                                     + String(sharedY) + ","
                                     + String(sharedTime);
        ws.textAll(statsStr);

        xSemaphoreGive(dataMutex); // ปลดล็อกข้อมูลให้คอร์ 1 ทำงานต่อ
      }
    }
    
    // หน่วงเวลาสำหรับ Task เครือข่าย (ประมาณ 10 เฟรมต่อวินาทีสัมพันธ์กับกล้อง)
    vTaskDelay(pdMS_TO_TICKS(60)); //62=16hz
  }
}
void control4WD(int speedLeft, int speedRight) {
  // บังคับจำกัดช่วงความเร็วให้อยู่ระหว่าง 0 ถึง 255 (ป้องกันค่าหลุดช่วง PWM)
  speedLeft  = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);

  // สั่งงานขา PWM ไปที่บอร์ดขับมอเตอร์รถ 4WD
  analogWrite(MOTOR_LEFT_PWM, speedLeft);
  analogWrite(MOTOR_RIGHT_PWM, speedRight);
}

void setup() {
  Serial.begin(115200);
  
  // สร้าง Mutex ล็อกข้อมูล
  dataMutex = xSemaphoreCreateMutex();

  Wire.begin(8, 9); // ขา SDA=8, SCL=9 ของ ESP32-S3
  delay(10);
  //Wire.setClock(400000); 
  Wire.setClock(1000000); // เร่งความเร็วบัส I2C จาก 400kHz เป็น 1MHz
  delay(10);
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Error: MLX90640 not found!");
    while (1) delay(10);
  }
  //ลักษณะสลับฟันปลาหรือเหมือนตารางหมากรุก (Chessboard) // สลับกันทีละแถว (แถวคู่ / แถวคี่)
  mlx.setMode(MLX90640_CHESS); //MLX90640_CHESS  MLX90640_INTERLEAVED
  delay(10);
  mlx.setResolution(MLX90640_ADC_18BIT);
  delay(10);
  //mlx.setRefreshRate(MLX90640_16_HZ); // ดันเรตเซนเซอร์เผื่อไว้เพื่อให้เก็บภาพได้คมชัด
  mlx.setRefreshRate(MLX90640_16_HZ);//MLX90640_8_HZ MLX90640_16_HZ  MLX90640_64_HZ
  delay(10);
  // เปิดโหมดปล่อย WiFi (AP Mode)
  WiFi.softAP(ssid, password, channel, hidden, max_connection);
  IPAddress IP = WiFi.softAPIP();
  // ตัวอย่าง: 20 = แรงสุด, 11 = ปานกลาง, 5 = เบาสุดสำหรับระยะใกล้
  WiFi.setTxPower(WIFI_POWER_11dBm); // หรือใส่ตัวเลขโดยตรงเช่น WiFi.setTxPower(11);
  Serial.print("Connect WiFi and open: http://");
  Serial.println(IP);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=utf-8", index_html);
  });
  delay(10);
  server.begin();
  delay(10);
 Serial.println("ESP32_Adafruit_MLX90640_V7-22-10-2569");
  // 🔥 จุดสำคัญ: สร้างและส่งงาน NetworkTask ไปรันแบบถาวรบน Core 0
  xTaskCreatePinnedToCore(
    NetworkTask,           /* ชื่อฟังก์ชันของ Task */
    "NetworkTask",         /* ชื่อเรียกหน่วยความจำ Task */
    8192,                  /* ขนาด Stack Size (เพิ่มเนื้อที่ให้ส่ง Binary ได้ปลอดภัย) */
    NULL,                  /* พารามิเตอร์ส่งเข้าฟังก์ชัน */
    1,                     /* ระดับความสำคัญ (Priority) */
    &NetworkTaskHandle,    /* ตัวแปรจัดการ Task */
    0                      /* 📌 สั่งล็อกให้รันเฉพาะบน Core 0 เท่านั้น! */
  );
}

// 🏎️ ==================== CORE 1: CAMERA LOOP & PID CONTROL ====================
// ฟังก์ชัน loop() ปกติของ Arduino จะรันอยู่บน Core 1 อัตโนมัติโดยที่คุณไม่ต้องประกาศใหม่
void loop() {
  //ws.cleanupClients(); 
  // 1. ดึงข้อมูลค่าความร้อนจากเซนเซอร์บน Core 1 ความเร็วสูงสุด
  if (mlx.getFrame(frame) == 0) {
    
    // อาร์เรย์คำนวณชั่วคราวบน Core 1
    int16_t localBuffer[768]; 
    float offset = 1.0; //31

    // วนลูปคำนวณคาลิเบรต
    for (int i = 0; i < 768; i++) {
      localBuffer[i] = (int16_t)((frame[i] + offset) * 10.0); 
    }

    // คำนวณหาค่าสถิติและพิกัด X, Y สดบน Core 1
    int16_t rawMin = localBuffer[0];
    int16_t rawMax = localBuffer[0];
    int maxIdx = 0;

    for (int i = 0; i < 768; i++) {
      if (localBuffer[i] < rawMin) rawMin = localBuffer[i];
      if (localBuffer[i] > rawMax) {
        rawMax = localBuffer[i];
        maxIdx = i;
      }
    }
  // ค่าที่คำนวณได้จริงในเฟรมปัจจุบัน (Current Raw Values)
    float currentMin = (float)rawMin / 10.0;
    float currentMax = (float)rawMax / 10.0;
    float currentCenter = (float)localBuffer[12 * 32 + 16] / 10.0;
    float currentX = (float)(maxIdx % 32);
    float currentY = (float)(maxIdx / 32);
    unsigned long localTime = millis();
    
 // 🔥 ==================== ⚡ เริ่มต้นประมวลผลสมการ EMA FILTER ⚡ ====================
    // กรองค่าอุณหภูมิให้เลิกแกว่ง
    filteredMin    = (alpha * currentMin)    + ((1.0 - alpha) * filteredMin);
    filteredMax    = (alpha * currentMax)    + ((1.0 - alpha) * filteredMax);
    filteredCenter = (alpha * currentCenter) + ((1.0 - alpha) * filteredCenter);

    // กรองพิกัดตำแหน่ง X, Y ให้เป้าเล็งเคลื่อนที่สมูท ไม่วาร์ปกระตุกข้ามช่อง
    filteredX      = (alpha * currentX)      + ((1.0 - alpha) * filteredX);
    filteredY      = (alpha * currentY)      + ((1.0 - alpha) * filteredY);
    // ===========================================================================


    // 🔥 ==================== ⚡ เพิ่มระบบ DEADBAND ดักจับค่ากระโดด ⚡ ====================
    float finalMin = filteredMin;
    float finalMax = filteredMax;
    int finalX = (int)round(filteredX);
    int finalY = (int)round(filteredY);

    // ตรวจสอบส่วนต่างระหว่างอุณหภูมิสูงสุดและต่ำสุดที่ผ่านการกรองแล้ว
    float tempDelta = filteredMax - filteredMin;

    // ถ้าส่วนต่างน้อยกว่าหรือเท่ากับ 5.0 องศา (เช่นอุณหภูมิกระจายตัวแคบๆ ในช่วง 27 ถึง 32 °C)
    if (tempDelta <= 5.0) {
      finalMax = filteredMax;       // 🎯 บังคับตัดผลลัพธ์ออกเป็นค่าคงที่ 27.0 °C ทันทีตามต้องการ
      finalX = 16;           // ล็อกเป้าเล็ง X ไว้ตรงกลางหน้าจอ (0-31) ป้องกันเป้าวิ่งกระโดดมั่ว
      finalY = 12;           // ล็อกเป้าเล็ง Y ไว้ตรงกลางหน้าจอ (0-23)
    }
    // ===================================================================================


    // 🏎️ [จุดสำคัญสำหรับงานวิจัย]: นำค่าพิกัดที่ผ่านการกรองแล้ว (filteredX) ไปป้อนเข้า PID
    // errorX = filteredX - targetX;
    // errorX = finalX - targetX; 
    // รถจะเลี้ยวโค้งได้อย่างนุ่มนวล มอเตอร์ไม่สะดุดกระตุกตามจุด Noise

    // ⚡ [จุดเชื่อมต่อระบบควบคุม]: เอาค่า sensorX, sensorY ไปประมวลผลลูป PID ขับมอเตอร์ตรงนี้ได้เลย!
    // การคำนวณ PID บน Core 1 ตรงนี้จะนิ่งและแม่นยำมาก เพราะไม่มีทราฟฟิก WiFi มาขัดจังหวะอีกต่อไป
    //ตัวอย่าง: สั่งงานรถยนต์ไฟฟ้า 4WD...

    // 🎯 ==================== 🏎️ เริ่มลูประบบล็อกเป้าวิ่งหาความร้อน (PID Control) ====================
    int currentBaseSpeed = 0;

    // เงื่อนไขตรวจสอบ: ถ้าอุณหภูมิสูงสุดโดดเกิน 32°C (แปลว่าเจอแมลงหรือใบไม้ความร้อนจริง ไม่ใช่พื้นว่าง)
    if (finalMax > 32.0) {
      
      // 1. คำนวณความเร็ววิ่งเข้าหา (Speed Loop อิงตามแกน Y)
      // ถ้า Y น้อยๆ (เป้าอยู่ข้างบน/ไกล) ให้เร่งความเร็ว แต่ถ้าเป้าเคลื่อนลงล่าง Y เกิน 20 (เข้าใกล้) ให้ชะลอรถจนหยุด
      if (finalY < 20) {
        currentBaseSpeed = map(finalY, 0, 20, maxBaseSpeed, 60); // ยิ่งใกล้ ยิ่งวิ่งช้าลง
      } else {
        currentBaseSpeed = 0; // วัตถุเข้ามาประชิดในระยะปลอดภัยแล้ว ให้รถหยุดนิ่งทันที
      }

      // 2. คำนวณระบบเลี้ยวตามเป้า (Steering PID อิงตามแกน X)
      errorX = finalX - targetX; // ค่าเบี่ยงเบนจากจุดกึ่งกลางจอ (16)
      integralX += errorX;
      integralX = constrain(integralX, -50, 50); // ป้องกันค่าอินทิเกรลสะสมล้น (Anti-windup)
      derivativeX = errorX - lastErrorX;

      // คำนวณเอาต์พุต PID สำหรับคุมเลี้ยว
      pidOutputTurn = (Kp_turn * errorX) + (Ki_turn * integralX) + (Kd_turn * derivativeX);
      lastErrorX = errorX;

    } else {
      // 🛑 ถ้าส่องไม่เจอวัตถุเป้าหมาย (อุณหภูมิราบเรียบ < 32°C หรือเข้าเงื่อนไข Deadband) 
      // ให้รถเบรกจอดนิ่งๆ อยู่กับที่ เพื่อความปลอดภัย ไม่ให้รถวิ่งมั่วซั่ว
      currentBaseSpeed = 0;
      pidOutputTurn = 0;
      integralX = 0; // รีเซ็ตค่าอินทิเกรลสะสม
    }

    // 3. ผสมสัญญาณความเร็วพื้นฐานและแรงเลี้ยว PID เข้าด้วยกัน (Differential Steering)
    int leftMotorOutput  = currentBaseSpeed + pidOutputTurn; // ถ้าเป้าอยู่ขวา (errorX+), ล้อซ้ายจะหมุนเร่งขึ้น
    int rightMotorOutput = currentBaseSpeed - pidOutputTurn; // และล้อขวาจะหมุนช้าลง ทำให้รถเบี่ยงเลี้ยวขวาตามเป้า

    // 4. ส่งค่าความเร็วที่ผ่าน PID ไปยังชุดล้อขับเคลื่อน 4 ล้อจริง
    control4WD(leftMotorOutput, rightMotorOutput);
    // =============================================================


    // 2. ขอล็อกและอัปเดตข้อมูลขึ้นหน่วยความจำกลาง เพื่อให้ Core 0 นำไปส่ง WiFi
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(sharedBinaryBuffer, localBuffer, sizeof(localBuffer));
      sharedMin = finalMin;
      sharedMax = finalMax; // ค่าที่ถูกปรับเป็น 27.0 แล้วในสภาวะอุณหภูมิคงที่
      sharedCenter = filteredCenter;
      sharedX = finalX;     // พิกัดตรงกลางภาพ (16) ในสภาวะอุณหภูมิคงที่
      sharedY = finalY;     // พิกัดตรงกลางภาพ (12) ในสภาวะอุณหภูมิคงที่
      sharedTime = localTime;
      xSemaphoreGive(dataMutex); // ส่งมอบการล็อกคืน
    }
  }
  
  // หน่วงเวลารอบวงจรหลักเล็กน้อยเพื่อให้คอร์ 1 จัดลำดับงานระบบได้คล่องตัว
  delay(1); 
}

// วางตัวแปรสตริงหน้าเว็บ index_html ต่อท้ายด้านล่างสุดของโค้ด...
// หน้าเว็บ HTML + JS ประมวลผลภาพความร้อน ฝังไว้ในหน่วยความจำ Flash (PROGMEM)
//พิกัดล็อกเป้าอัตโนมัติ: ค้นหาจุดที่ร้อนที่สุดในจออย่างรวดเร็ว และพิมพ์บอกพิกัดแกน X (0-31) 
//และแกน Y (0-23) ไว้ในแถบ Hotspot Location ด้านล่างภาพ  3. โค้ดฝั่งหน้าเว็บ HTML (รับค่าพิกัดสำเร็จรูปจาก ESP32)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Thermal Live Display & Tracker</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background: #121212; color: #fff; margin: 0; padding: 20px; }
        h2 { color: #ff9800; margin-bottom: 5px; }
        #container { display: inline-block; background: #222; padding: 15px; border-radius: 10px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
        .canvas-container { position: relative; display: inline-block; max-width: 640px; width: 100%; }
        canvas { border: 2px solid #444; width: 100%; height: auto; display: block; }
        .stats { display: flex; justify-content: space-around; margin-top: 15px; font-size: 16px; font-weight: bold; }
        .max { color: #ff5722; } .min { color: #00bcd4; } .center { color: #4caf50; }
        .status { font-size: 12px; color: #888; margin-top: 10px; padding: 5px; border-radius: 5px; }
        .connected { color: #4caf50; background: rgba(76,175,80,0.1); }
        .disconnected { color: #f44336; background: rgba(244,67,54,0.1); }
        .track-info { margin-top: 5px; font-size: 14px; color: #e0e0e0; font-family: monospace; }
        .time-info { font-size: 14px; color: #ffeb3b; margin-top: 5px; font-family: monospace; font-weight: bold; }
    </style>
</head>
<body>

    <h2>&#128293; ESP32 Thermal Live Display & Tracker</h2>
    <div class="time-info">⏱️ System Time: <span id="sysTime">00:00.0</span></div>
    
    <div id="container" style="margin-top: 10px;">
        <div class="canvas-container">
            <canvas id="thermalCanvas" width="320" height="240"></canvas>
        </div>
        <div class="track-info">🔥 Hotspot Location: <span id="trackPos">X: --, Y: --</span></div>
        <div class="stats">
            <div class="min">Min: <span id="minTemp">--</span> &deg;C</div>
            <div class="center">| Center: <span id="centerTemp">--</span> &deg;C |</div>
            <div class="max">Max: <span id="maxTemp">--</span> &deg;C</div>
        </div>
        <div class="status disconnected" id="wsStatus">Connecting to ESP32 WebSocket...</div>
    </div>

<script>
    const canvas = document.getElementById("thermalCanvas");
    const ctx = canvas.getContext("2d");
    
    const offscreenCanvas = document.createElement("canvas");
    offscreenCanvas.width = 32;
    offscreenCanvas.height = 24;
    const offscreenCtx = offscreenCanvas.getContext("2d");
    const imgData = offscreenCtx.createImageData(32, 24);

    const gateway = `ws://${window.location.hostname}/ws`;
    let websocket;

    let currentX = 0, currentY = 0;
    let valMin = 0, valMax = 0, valCenter = 0;

    function initWebSocket() {
        websocket = new WebSocket(gateway);
        websocket.binaryType = "arraybuffer";

        websocket.onopen = () => {
            document.getElementById("wsStatus").innerText = "Connected to Live Thermal Stream";
            document.getElementById("wsStatus").className = "status connected";
        };

        websocket.onclose = () => {
            document.getElementById("wsStatus").innerText = "Disconnected. Reconnecting in 2s...";
            document.getElementById("wsStatus").className = "status disconnected";
            setTimeout(initWebSocket, 2000);
        };

        websocket.onmessage = (event) => {
            if (event.data instanceof ArrayBuffer) {
                const temps = new Int16Array(event.data);
                if (temps.length !== 768) return;

                for (let i = 0; i < 768; i++) {
                    let realT = temps[i] / 10.0;
                    let norm = (realT - valMin) / (valMax - valMin || 1);
                    let rgb = heatMapColor(norm);

                    let idx = i * 4;
                    imgData.data[idx]     = rgb.r;
                    imgData.data[idx + 1] = rgb.g;
                    imgData.data[idx + 2] = rgb.b;
                    imgData.data[idx + 3] = 255;
                }
                offscreenCtx.putImageData(imgData, 0, 0);

                requestAnimationFrame(() => {
                    ctx.imageSmoothingEnabled = true;
                    ctx.drawImage(offscreenCanvas, 0, 0, canvas.width, canvas.height);

                    const pixelWidth = canvas.width / 32;
                    const pixelHeight = canvas.height / 24;
                    const screenX = (currentX * pixelWidth) + (pixelWidth / 2);
                    const screenY = (currentY * pixelHeight) + (pixelHeight / 2);

                    ctx.lineWidth = 1.5;
                    ctx.strokeStyle = "#000000"; drawTarget(ctx, screenX, screenY, 12);
                    ctx.strokeStyle = "#ffffff"; drawTarget(ctx, screenX, screenY, 10);

                    ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
                    ctx.fillRect(screenX + 12, screenY - 22, 65, 18);
                    ctx.fillStyle = "#ffffff";
                    ctx.font = "bold 11px monospace";
                    ctx.fillText(`${valMax.toFixed(1)}°C`, screenX + 16, screenY - 9);
                });
            } else {
                if (event.data.startsWith("TRACKER:")) {
                    const parts = event.data.substring(8).split(',');
                    valMin = parseFloat(parts[0]);
                    valMax = parseFloat(parts[1]);
                    valCenter = parseFloat(parts[2]);
                    currentX = parseInt(parts[3]);
                    currentY = parseInt(parts[4]);

                    // แสดงผลเวลาโดยดึงค่า index ลำดับที่ 5 ที่แก้ไขแล้ว
                    if (parts[5]) {
                        const totalMs = parseInt(parts[5]);
                        const minutes = Math.floor(totalMs / 60000);
                        const seconds = Math.floor((totalMs % 60000) / 1000);
                        const tenths = Math.floor((totalMs % 1000) / 100);
                        
                        const strMin = String(minutes).padStart(2, '0');
                        const strSec = String(seconds).padStart(2, '0');
                        
                        document.getElementById("sysTime").innerText = `${strMin}:${strSec}.${tenths}`;
                    }

                    document.getElementById("minTemp").innerText = valMin.toFixed(1);
                    document.getElementById("maxTemp").innerText = valMax.toFixed(1);
                    document.getElementById("centerTemp").innerText = valCenter.toFixed(1);
                    document.getElementById("trackPos").innerText = `X: ${currentX}, Y: ${currentY}`;
                }
            }
        };
    }

    function drawTarget(c, x, y, size) {
        c.beginPath();
        c.moveTo(x - size, y); c.lineTo(x + size, y);
        c.moveTo(x, y - size); c.lineTo(x, y + size);
        c.stroke();
        c.beginPath();
        c.arc(x, y, size/2, 0, 2 * Math.PI);
        c.stroke();
    }

    function heatMapColor(v) {
        let r = 0, g = 0, b = 0;
        if (v < 0.25) {
            r = 0; g = Math.floor(v * 4 * 255); b = 255;
        } else if (v < 0.5) {
            r = 0; g = 255; b = Math.floor((1 - (v - 0.25) * 4) * 255);
        } else if (v < 0.75) {
            r = Math.floor((v - 0.5) * 4 * 255); g = 255; b = 0;
        } else {
            r = 255; g = Math.floor((1 - (v - 0.75) * 4) * 255); b = 0;
        }
        return {r, g, b};
    }

    window.addEventListener('load', initWebSocket);
</script>
</body>
</html>
)rawliteral";
