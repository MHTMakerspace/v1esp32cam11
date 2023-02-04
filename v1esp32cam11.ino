// Include the necessary libraries

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#define CORE_DEBUG_LEVEL ESP_LOG_INFO
#define BOOTLOG "/bootlog.txt"
#include "esp_log.h"
#define TAG  "CAM" 

#define MAX_DAYS 4

#include "secrets.h"
#define VERSION "0.01"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>

#include <uptime.h>
#include <uptime_formatter.h>
#include <DateTime.h>
#include <DateTimeTZ.h>
#include <ESPDateTime.h>
#include <TimeElapsed.h>

#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "Base64.h"
#include <UrlEncode.h>  // https://github.com/plageoj/urlencode

//Aysnc HTTP WebServer with OTA
 
#include <ESPAsyncWebServer.h>
// For OTA updating
#include <Update.h>
#define U_PART U_SPIFFS
 
#include <iostream>
#include <string>
#include <queue>
#include <stack>
#include <ostream>
#include <istream>
#include <sstream>


// Local camera defines for GPIO and 
#include "camera.h"


#define XQUOTE(x) #x
#define QUOTE(x) XQUOTE(x)
#define WDT_TIMEOUT 900
#define WIFI_TIMEOUT_MS 11000 // 11 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 45000 // Wait 45 seconds after a failed connection attempt

#define DEBUG 1
#define MS_PER_FRAME 500

#define PIN_RED 33
#define PIN_PIR 3
#define PIN_PIR_ACTIVE LOW

#define STACK_UPLOAD_QUEUE 2534
#define BIGBUFFER 262144

void keepWiFiAlive(void * parameter);
void SetupCamera();
void SetupWiFi();
void SetupMicroSD();
void SetupPIR();
void SetupUploadTask();
boolean toggleRedLED();
void SetupHTTPServer();
boolean ProcessPIR(boolean pir, camera_fb_t *frame);
boolean createDir(fs::FS &fs, const char * path);
void appendFile(fs::FS &fs, const char * path, const char * message);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
boolean PurgeSD(char * directory);
boolean PurgeOldDir(int days,int lookback);
void LogBoot();
boolean PIR();

String zenurlencode(String str);

char lastSavedFrame[80] = "/";        // Copy of the filename for last file we saved
char * SaveFrame(camera_fb_t *frame); // Write a frame to storage and release the malloc
camera_fb_t * Shoot(); // Grab one frame, return framebuffer reference to allocated struct
boolean ForceSnapshot=false;  // force a snapshot to be saved and uploaded

// Global variables
camera_fb_t * fb; // Pointer is a global
time_t now=DateTime.getTime();
// Unique device ID
char host[16];

AsyncWebServer server(80);

std::queue<std::string> Q;
std::queue<std::string> QE;

void setup() {
  esp_log_level_set("LOGGING", ESP_LOG_VERBOSE);
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("wifi", ESP_LOG_WARN);
  esp_log_level_set(TAG,ESP_LOG_DEBUG); // My custom events
  if(DEBUG) esp_log_level_set("*", ESP_LOG_VERBOSE);

  // set IO pin to output and 'off'
  //pinMode(0, OUTPUT);
  ///digitalWrite(PIN_RED, LOW); // Inverted logic, turn the LED on

  snprintf(host, 16, "ESP%012llX", ESP.getEfuseMac());
  
  Serial.begin(115200);
  Serial.printf("\r\n %s in setup()\r\n",host);
  delay(10);

  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  SetupMicroSD();
  appendFile(SD_MMC,BOOTLOG,"\r\nStarting up...\r\n");
 
  delay(10);
  SetupCamera();
  
  pinMode(PIN_RED, OUTPUT);
  toggleRedLED();

  SetupWiFi();

  LogBoot(); // Log to SD card
  toggleRedLED();



  if(DEBUG) Serial.println("Grabbing one frame...");
  fb=Shoot();  //Grab a new frame
  if(fb) {
    if(DEBUG) Serial.println("Captured a frame buffer");
    char * path=SaveFrame(fb);
    if(path) {
      strcpy(lastSavedFrame,path);
       if(DEBUG) Serial.printf("Saved frame as %s\r\n",lastSavedFrame);
       std::string file(lastSavedFrame);

      if(DEBUG) Serial.println("Uploading...");
      if(DoPrimitiveUpload(file))  
           if(DEBUG) Serial.println("Uploaded!");
      else
          if(DEBUG) Serial.println("Failed to Upload.");
    }
  else  
    if(DEBUG) Serial.println("Failed to fetch and save a frame.");
}
  else  
    if(DEBUG) Serial.println("Failed to fetch a frame.");




  PurgeOldDir(MAX_DAYS,3);
  if(DEBUG) Serial.println("Creating upload task.");
  SetupUploadTask();
  toggleRedLED();
  SetupHTTPServer();

  
 log_i(TAG,"Setting hardware watchdog timeout to %d", WDT_TIMEOUT);
 esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
 esp_task_wdt_add(NULL); //add current thread to WDT watch

Serial.print("Leaving setup at ");
Serial.println(DateTime.toISOString().c_str());
}


///////////////////////////////////////////
//
//  loop
void loop() {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int sleep = MS_PER_FRAME;
  
  // put your main code here, to run repeatedly:
  esp_task_wdt_reset(); // Reset watchdog timer
  toggleRedLED();
 
  boolean didcap=ProcessPIR(PIR(),fb);  // Processes based on PIR, returns true if a frame was written

  if(fb) esp_camera_fb_return(fb);  //  free the memory allocated by esp_camera_fb_get

  fb=Shoot();  //Grab a new frame
  
  if(Q.size()) {
    // We have fresh elements needing uploading
    log_i(TAG,"Queue is %d elements.",Q.size());
  }

  toggleRedLED();
  if(!didcap) sleep=MS_PER_FRAME*2;
  vTaskDelayUntil( &xLastWakeTime, sleep / portTICK_RATE_MS);
}

boolean ProcessPIR(boolean pir, camera_fb_t *frame) {
static boolean lastpir=false;
boolean commitframe=false;

if(ForceSnapshot) {
    // We need a snapshot, regardless of PIR.
    ForceSnapshot=false;
    commitframe=true;
    if(DEBUG) Serial.println("Processing ForceSnapshot");
}

if(pir) {
      // PIR is true
      if(!lastpir) {
         // Just transitioned from 0->1
         if(DEBUG) Serial.print("P");
         // Call our "new motion webhook"
         lastpir=true;
      }
    commitframe=true; // We want to save frames while PIR is active
  }
  else {
    // PIR is false
    if(lastpir) {
        // Just transitioned from 1->0
        if(DEBUG) Serial.print("p");
        // Call our "end motion webhook"
        lastpir=false;
        commitframe=true; // Capture one last frame
    }
    else {
      if(DEBUG > 2) Serial.print(".");
    }
  }
  if(commitframe) {
      if(DEBUG) Serial.print("+");
      char * path=SaveFrame(frame);
      if(path) {
        strcpy(lastSavedFrame,path);
        std::string s(path);
        Q.push(s);
      }
  }
return(commitframe);
}


/**
 * Task: monitor the WiFi connection and keep it alive!
 * 
 * When a WiFi connection is established, this task will check it every 10 seconds 
 * to make sure it's still alive.
 * 
 * If not, a reconnect is attempted. If this fails to finish within the timeout,
 * the ESP32 will wait for it to recover and try again.
 */
void keepWiFiAlive(void * parameter){
    for(;;){
        if(WiFi.status() == WL_CONNECTED){
          if (!DateTime.isTimeValid()) {
            log_i(TAG,"Failed to get time from DateTime server, retry.");
            DateTime.begin();
            }
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            continue;
        }
        if(DEBUG) Serial.print("[WIFI DOWN]");
        log_i(TAG,"Connecting to %s",WIFI_NETWORK );
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

        unsigned long startAttemptTime = millis();

        // Keep looping while we're not connected and haven't reached the timeout
        while (WiFi.status() != WL_CONNECTED && 
                millis() - startAttemptTime < WIFI_TIMEOUT_MS)  {
                  vTaskDelay(50 / portTICK_PERIOD_MS);
                  }

        // When we couldn't make a WiFi connection (or the timeout expired)
      // sleep for a while and then retry.
        if(WiFi.status() != WL_CONNECTED){
            log_e("WiFi FAILED");
            vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
        continue;
        }
     if(DEBUG) Serial.println("[WIFI RESTORED]");
     log_v(TAG,"Connected to %s with IP address %s ",ssid,WiFi.localIP().toString().c_str());
     vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    // When you're done, call vTaskDelete. Don't forget this!
    vTaskDelete(NULL);
}




///////////////////////////////////////////
//
// deleteFile
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

///////////////////////////////////////////
//
//  DoUpload
//


boolean DoUpload(const std::string input) {
  static char* psdRamBuffer = (char*)ps_malloc(BIGBUFFER);
  const char * imagefile=input.c_str();
  fs::FS &fs = SD_MMC;
  File sourcefile =  fs.open(imagefile);
  if(!sourcefile) {
    if(DEBUG) Serial.printf("File %s is unreadable, deleting.",imagefile);
    deleteFile(SD_MMC,imagefile);
    return true;  // File is unreadable,
    }
   if(!sourcefile.size()) {
      if(DEBUG) Serial.printf("File %s is empty, deleting.",imagefile);
      sourcefile.close();
      deleteFile(SD_MMC,imagefile);
      return(true);
  }

char* bigbuffer = psdRamBuffer;
char * postdata=psdRamBuffer;
unsigned long freebuffer=BIGBUFFER-1;

if(DEBUG) Serial.printf("DoUpload is building POST for %s\r\n",imagefile);
std::string fullpath(imagefile);
std::string dateFolder=fullpath.substr(1,10);
std::string fileName=fullpath.substr(11);
 
int used=snprintf(bigbuffer,freebuffer,"apikey=%s&camera=%s&filename=%s&datefolder=%s&data=",SHAREDAPIKEY, MDNS_NAME,
  fileName.c_str(),dateFolder.c_str());
freebuffer-=used;bigbuffer+=used;

#define READBUF 256
static char buf[READBUF];
char output[4+(4*(READBUF/3))];

size_t len = sourcefile.size();
size_t flen = len;
size_t fread=0;
String encoded;

if(DEBUG) Serial.printf("DoUpload is reading %s, bufferused=%d",fileName.c_str(),used);

int encodedtotal=0;
while (sourcefile.available() && len) {
  if(DEBUG) Serial.print(".");
  size_t toRead = len;
  if(toRead > READBUF){
    toRead = READBUF;
  }
  fread+=sourcefile.readBytes(buf, toRead);
  len -= toRead;
  base64_encode(output,buf, toRead); 
  encoded=urlEncode(String(output));
  int used=snprintf(bigbuffer,freebuffer,"%s",encoded.c_str());
  freebuffer-=used;bigbuffer+=used;encodedtotal+=used;
  }
*bigbuffer=NULL; bigbuffer++; *bigbuffer=NULL; 
if(DEBUG) Serial.printf("\r\nDisk size %d, read %d, encoded as %d\r\n",flen,fread,encodedtotal);
sourcefile.close();
 

WiFiClientSecure *client = new WiFiClientSecure;
//client->setInsecure();
{ 
    // Force scoping so HTTPclient is destroyed before WifiClientSecure client
    HTTPClient http;    
    int  obj =   http.begin(*client, UPLOADFRAME_WEBAPP);
    if(obj) {
      if(DEBUG) Serial.println("Connected HTTPS to upload target.");
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      if(DEBUG) Serial.printf("DoUpload POST buffer is %d bytes\r\n",strlen(postdata));
      
      int httpResponseCode = http.POST(postdata );
      http.end();

      
      log_i("POST received response of %d",httpResponseCode);
      if(DEBUG) Serial.printf("POST received response of %d\r\n",httpResponseCode);
      if(200==httpResponseCode) return(true);
    }
    else {
      if(DEBUG) Serial.println("Failed to start upload");
      log_i("Failed to start upload");
      return(false);
    }
}
  if(DEBUG) Serial.println("Leaving DoUpload with return code of false");
  return(false);
}

/////////////////////////////////
//
// DoPrimitiveUpload
//
boolean DoPrimitiveUpload(const std::string input) {
 boolean outcome=false;
 WiFiClientSecure client;
 //client.setInsecure();
  
  if (!client.connect("pinebarrens.msg.net", 443))  {         
    Serial.println("Connect to " + String(UPLOADFRAME_WEBDOMAIN) + " failed.");
    return(outcome);
  }
  else {
    Serial.println("DoPrimitiveUpload Connection successful");
    
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();  
    if(!fb) {
      Serial.println("Camera capture failed");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      return(outcome);
    }
  
    char *input = (char *)fb->buf;
    char output[base64_enc_len(3)];
    String imageFile = "";
    for (int i=0;i<fb->len;i++) {
      base64_encode(output, (input++), 3);
      if (i%3==0) imageFile += zenurlencode(String(output));
    }
    char poster[128];
    std::string fullpath(lastSavedFrame);
    std::string dateFolder=fullpath.substr(1,10);
    std::string fileName=fullpath.substr(11);
 
    int posterlen=snprintf(poster,128,"apikey=%s&camera=%s&filename=%s&datefolder=%s&data=",SHAREDAPIKEY, MDNS_NAME, fileName.c_str(),dateFolder.c_str());
    //String Data = String(poster);
    
    esp_camera_fb_return(fb);
    
    Serial.println("Send a captured image to Google Drive.");
    
    client.printf("POST %s HTTP/1.1",UPLOADFRAME_WEBAPP);
    client.printf("Host: %s\n" , UPLOADFRAME_WEBDOMAIN);
    client.printf("Content-Length: %u\n" +  posterlen +imageFile.length()  );
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println();
    
    client.printf("%s",poster);
    
    int Index;
    for (Index = 0; Index < imageFile.length(); Index = Index+1000) {
      client.print(imageFile.substring(Index, Index+1000));
    }
    
    Serial.println("Waiting for response.");
    long int StartTime=millis();
    static int waitingTime = 30000; //Wait 30 seconds to google response.
    while (!client.available()) {
      Serial.print(".");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if ((StartTime+waitingTime) < millis()) {
        Serial.println();
        Serial.println("No response.");
        outcome=false;
        //If you have no response, maybe need a greater value of waitingTime
        break;
      }
    }
    Serial.println();   
     outcome=true;
     while (client.available()) {
      Serial.print(char(client.read()));
     
    }  
  }
  client.stop();
  return(outcome);
}

//////////////////////////////////
//
//DoManualUpload
//
boolean DoManualUpload(const std::string input) {
  static char* psdRamBuffer = (char*)ps_malloc(BIGBUFFER);
  const char * imagefile=input.c_str();
  static int waitingTime = 30000; //Wait 30 seconds to google response.
  fs::FS &fs = SD_MMC;
  File sourcefile =  fs.open(imagefile);
  if(!sourcefile) {
    if(DEBUG) Serial.printf("File %s is unreadable, deleting.",imagefile);
    deleteFile(SD_MMC,imagefile);
    return true;  // File is unreadable,
    }
   if(!sourcefile.size()) {
      if(DEBUG) Serial.printf("File %s is empty, deleting.",imagefile);
      sourcefile.close();
      deleteFile(SD_MMC,imagefile);
      return(true);
  }

char* bigbuffer = psdRamBuffer;
char * postdata=psdRamBuffer;
unsigned long freebuffer=BIGBUFFER-1;

if(DEBUG) Serial.printf("DoManualUpload is building POST for %s\r\n",imagefile);
std::string fullpath(imagefile);
std::string dateFolder=fullpath.substr(1,10);
std::string fileName=fullpath.substr(11);
//std::ostringstream oss;
//oss << "apikey=" << SHAREDAPIKEY << "&camera=" << MDNS_NAME << "&filename=" << fileName << "&datefolder=" << dateFolder << "data=";

int used=snprintf(bigbuffer,freebuffer,"apikey=%s&camera=%s&filename=%s&datefolder=%s&data=",SHAREDAPIKEY, MDNS_NAME,
  fileName.c_str(),dateFolder.c_str());
freebuffer-=used;bigbuffer+=used;

#define READBUF 256
static char buf[READBUF];
char output[4+(4*(READBUF/3))];

size_t len = sourcefile.size();
size_t flen = len;
size_t fread=0;
String encoded;

if(DEBUG) Serial.printf("DoManualUpload is reading %s, bufferused=%d",fileName.c_str(),used);

int encodedtotal=0;
while (sourcefile.available() && len) {
  if(DEBUG) Serial.print(".");
  size_t toRead = len;
  if(toRead > READBUF){
    toRead = READBUF;
  }
  fread+=sourcefile.readBytes(buf, toRead);
  len -= toRead;
  base64_encode(output,buf, toRead);
  //oss << urlEncode(String(output));
  encoded=urlEncode(String(output));
  int used=snprintf(bigbuffer,freebuffer,"%s",encoded.c_str());
  freebuffer-=used;bigbuffer+=used;encodedtotal+=used;
  }
*bigbuffer=NULL; bigbuffer++; *bigbuffer=NULL; 
if(DEBUG) Serial.printf("\r\nDisk size %d, read %d, encoded as %d\r\n",flen,fread,encodedtotal);
sourcefile.close();

//std::string postdata = oss.str();

WiFiClientSecure client;
//client.setInsecure();
{ 
    if (client.connect(UPLOADFRAME_WEBDOMAIN, 443)) {
    Serial.println("DoManualUpload Connection successful");

    int bufsize=strlen(postdata);
    if(DEBUG) Serial.printf("DoManualUpload POST buffer is %d bytes\r\n",bufsize);
  
    client.printf("POST %s HTTP/1.1\n",UPLOADFRAME_WEBAPP);
    client.printf("Host: %s\n", UPLOADFRAME_WEBDOMAIN);
    client.printf("Content-Length: %d\n" , bufsize);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println();
    if(DEBUG) Serial.println("DoManualUpload sending full POST buffer");
    client.print(postdata);
    if(DEBUG) Serial.println("DoManualUpload sent full POST buffer");
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    
    Serial.println("DoManualUploadWaiting for response.");
    long int StartTime=millis();
    while (!client.available()) {
      Serial.print(".");
      vTaskDelay(100 / portTICK_PERIOD_MS); 
      if ((StartTime+waitingTime) < millis()) {
        Serial.println();
        Serial.println("DoManualUpload No response.");
        //If you have no response, maybe need a greater value of waitingTime
        break;
     }
    }
    Serial.println();   
    while (client.available()) {
      Serial.print(char(client.read()));
    }  
    } else {        
    Serial.println("DoManualUpload Connect failed.");
     client.stop();
     if(DEBUG) Serial.println("Leaving DoManualUpload with return code of false");
     return(false);
    }
  client.stop();
}

if(DEBUG) Serial.println("Leaving DoManualUpload with return code of true");
return(true);
}



///////////////////////////////////////////
//
// UploadQueue
//
void UploadQueue(void * parameter) {
  for(;;){
    if(WiFi.status() != WL_CONNECTED) {
      if(DEBUG && (Q.size()+QE.size()) ) Serial.printf("q(%d,%d) ",Q.size(),QE.size());
    }
    else {
      if(DEBUG && (Q.size()+QE.size()) ) Serial.printf("Q(%d,%d) ",Q.size(),QE.size());
      if(Q.empty())
          vTaskDelay(1000 / portTICK_PERIOD_MS);
       else {
        // We have work to do
         const auto &result = Q.front(); // reference to element
        if(DEBUG) Serial.printf("UploadQueue main queue will upload %s\r\n",result.c_str());

        //boolean success=DoManualUpload(result);
        boolean success=DoPrimitiveUpload(result);
        if(DEBUG) Serial.printf("Upload returned %s", success ? "true" : "false");
        
        if(!success) QE.push(result);
          Q.pop();  // delete front -- refence becomes invalid
        if(DEBUG) Serial.printf("UploadQueue main queue is now %d\r\n",Q.size() ); 
        }

    if(Q.empty() && !QE.empty()) {
        // We have error queue work to do
        const auto &result = QE.front(); // reference to element
        if(DEBUG) Serial.printf("UploadQueue retry queue will upload %s\r\n",result.c_str());

        //boolean success=DoManualUpload(result);
        boolean success=DoPrimitiveUpload(result);
        if(DEBUG) Serial.printf("Retry Queue Upload returned %s", success ? "true" : "false");
        
        if(success )
          QE.pop ();  // delete front -- refence becomes invalid
        else {
          log_i(TAG,"Failed to upload %s on second try, abandoning it.",result.c_str());
            if(DEBUG) Serial.printf("Abandoning %s after retry.\r\n",result.c_str());
            QE.pop ();
        }
      if(DEBUG) Serial.printf("UploadQueue error queue is now %d\r\n",QE.size() );   
      }

  if(Q.empty() && QE.empty() )
   vTaskDelay(1000 / portTICK_PERIOD_MS);
  else
    vTaskDelay(100 / portTICK_PERIOD_MS);  
  }
 }
}

///////////////////////////////////////////
//
// WebSendImage
//
void WebSendImage(AsyncWebServerRequest *request) {
  AsyncWebServerRequest* rawRequest = request;
  static uint32_t lastTimestamp;

  if(!fb) {
    log_w(TAG,"HTTP WebSendImage failed due to lack of a frame in fb");
    rawRequest->send(503);
    return;
  }
  if(lastTimestamp == fb->timestamp.tv_sec) {
     log_w(TAG,"HTTP WebSendImage returning 304");
     rawRequest->send(304);
     return;
  }
  //  Return the latest framebuffer contents
  AsyncWebServerResponse *response = rawRequest->beginResponse_P(200, "image/jpeg", fb->buf,fb->len);
  lastTimestamp=fb->timestamp.tv_sec;
    
  response->addHeader("Server","ESP Async Web Server");
  response->addHeader("Expires","now");
  response->addHeader("Refresh","5; /image.jpg");


  rawRequest->send(response);
  log_d(TAG,"HTTP WebSendImage Exiting OK");
}

///////////////////////////////////////////
//
// WebSendIndex
//
void WebSendIndex(AsyncWebServerRequest *request) {
  AsyncWebServerRequest* rawRequest = request;
AsyncResponseStream *response = rawRequest->beginResponseStream("text/html");

response->addHeader("Server","ESP Async Web Server");
response->printf("<!DOCTYPE html><html><head><title>WebCam v%s</title>",QUOTE(VERSION));
 response->print("<meta http-equiv=\"refresh\" content=\"25\" /><meta http-equiv=\"pragma\" content=\"no-cache\" />");

  response->printf("</HEAD><BODY><UL><LI><A HREF=\"/ForceSnapshot\">ForceSnapshot</A></LI>");
  response->printf("<LI><A HREF=\"/image.jpg\">Latest frame buffer</A></LI>");
  response->printf("</UL>\r\n");

// Last image we captured, if any.
response->printf("<BR><HR><IMG SRC=%s ALT=%s >",lastSavedFrame,lastSavedFrame); 
rawRequest->send(response);

log_d(TAG,"HTTP WebSendIndex Exiting OK");
}

///////////////////////////////////////////
//
// WebForceSnapshot
//
void WebForceSnapshot(AsyncWebServerRequest *request) {
  AsyncWebServerRequest* rawRequest = request;
AsyncResponseStream *response = rawRequest->beginResponseStream("text/html");

response->addHeader("Server","ESP Async Web Server");
response->addHeader("Refresh","3;/image.jpg");
response->printf("<!DOCTYPE html><html><head><title>WebCam v%s</title>",QUOTE(VERSION));
response->print("<meta http-equiv=\"refresh\" content=\"3;/image.jpg\" /><meta http-equiv=\"pragma\" content=\"no-cache\" />");
 response->print("</head><BODY><H1><CENTER>Snapshot requested</CENTER</H1></BODY>");
 
rawRequest->send(response);

log_d(TAG,"HTTP WebForceSnapshot Exiting OK");
if(DEBUG) Serial.println("WebForceSnapshot() calling ForceSnapshot");
ForceSnapshot=true;
}

///////////////////////////////////////////
//
// handleUpdate
//
void handleUpdate(AsyncWebServerRequest *request) {
  char* html = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  request->send(200, "text/html", html);
}

///////////////////////////////////////////
//
// handleDoUpdate
void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
   size_t content_len;

   if(!request->authenticate(AUTH_USERNAME, AUTH_PASSWORD))
      return request->requestAuthentication();
  if (!index){
    Serial.println("Update");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");  
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      Serial.println("Update complete");
      Serial.flush();
      ESP.restart();
    }
  }
}

///////////////////////////////////////////
//
// SetupHTTPServer
//
void SetupHTTPServer() {
  //authProvider.requireAuthentication(AUTH_USERNAME, AUTH_PASSWORD);


  if(DEBUG) Serial.println("Setting up HTTP server...");
  
    server.on("/image.jpg", HTTP_GET, [](AsyncWebServerRequest *request){WebSendImage(request);});
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){WebSendIndex(request);});
    server.on("/ForceSnapshot", HTTP_GET, [](AsyncWebServerRequest *request){WebForceSnapshot(request);});

    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
    server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {
                    if(!request->authenticate(AUTH_USERNAME, AUTH_PASSWORD))
                        return request->requestAuthentication();
                    handleDoUpdate(request, filename, index, data, len, final);}
          );
  
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});

  if(DEBUG) Serial.println("Starting HTTP server...");
  server.begin();
  if(DEBUG) {
    if(DEBUG >2) Serial.println("Started HTTP server!");
    Serial.printf("\r\nhttp://%s:80/\r\n",WiFi.localIP().toString().c_str() );
    delay(300);
  }
} 

///////////////////////////////////////////
//
// SetupWiFi
//
void SetupWiFi() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.print("WiFi");
  
  WiFi.mode(WIFI_STA);

  log_i(TAG,"Connecting to %s",WIFI_NETWORK );
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);  

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  log_i(TAG,"Connected to %s with IP address %s ",WIFI_NETWORK,WiFi.localIP().toString().c_str());
  if(DEBUG) Serial.printf("\r\nWiFi OK,Connected to %s with IP address %s\r\n",WIFI_NETWORK,WiFi.localIP().toString().c_str());
  

  if (MDNS.begin(QUOTE(MDNS_NAME)))
    log_i(TAG,"MDNS responder started as %s", MDNS_NAME);
  else 
     log_w(TAG,"MDNS.begin failed!");

  log_i(TAG,"Setting clock with DateTime... ");
  DateTime.setServer(NTPSERVER);
  DateTime.setTimeZone(TIMEZONE);
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    log_w(TAG,"DateTime Failed to get time from server %s.", NTPSERVER);
  } else {
    log_i(TAG,"Date Now is %s, timestamp %ld", DateTime.toISOString().c_str(), DateTime.now());
  }
    
  log_i(TAG,"Creating keepWiFiAlive task");
  
  xTaskCreatePinnedToCore(
    keepWiFiAlive,
    "keepWiFiAlive",  // Task name
    1000,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL,             // Task handle
    xPortGetCoreID()
    );
}

void SetupUploadTask() {
  auto result = xTaskCreatePinnedToCore(
    UploadQueue,
    "UploadQueue",  // Task name
    STACK_UPLOAD_QUEUE,             // Stack size (bytes)
    NULL,             // Parameter
    2,                // Task priority
    NULL,             // Task handle
    xPortGetCoreID()
    );
 if(pdPASS == result) {
  log_i("Successfully created UploadQueue task");
  if(DEBUG) Serial.println("Successfully created UploadQueue task");
  return;
 }
   log_e("Failed to create  UploadQueue task");
  if(DEBUG) Serial.println("Failed to create UploadQueue task");
  delay(2000);
}

/////////////////////////////////
//
// Shoot
//
camera_fb_t * Shoot() {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();  
    if(!fb) {
      log_e(TAG,"Camera capture failed");
      if(DEBUG) Serial.println("Failed to capture frame from camera");
    }

  log_d(TAG,"Captured %d byte image at %d x %d", fb->len,fb->width,fb->height);
  if(DEBUG>1) Serial.printf("Captured %d byte image at %d x %d\r\n", fb->len,fb->width,fb->height);
    
  return(fb);
  
    //char *input = (char *)fb->buf;
    // fb->timestamp is system boot clock timestamp of the first DMA buffer of the frame
    // fb->len is size of buffer in bytes
    esp_camera_fb_return(fb);  //  free the memory allocated by esp_camera_fb_get
}

/////////////////////////////////
//
// SetupCamera
//
void SetupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  config.jpeg_quality = 24; //  From 10 (best) to 63 (worst). Defaults to 10
  config.fb_count = 1;

if (psramFound()) {
    if(DEBUG) Serial.printf("We have psram, %d kb, with %d kb free.\r\n",ESP.getPsramSize()/1024, ESP.getFreePsram()/1024 );
    log_d("Total PSRAM: %d", ESP.getPsramSize());
    log_d("Free PSRAM: %d", ESP.getFreePsram());
    config.frame_size = FRAMESIZE_XGA;
  }
    
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
      delay(900);
    log_e("Camera init failed with error 0x%x", err);
    Serial.println("Could not start camera, retrying...");
    // The GIOP32 drives Q2 which powers the CSI voltage regulators.
    // To switch off the CSI voltage,   set the GOIP32 to High
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM,HIGH); 
    delay(1000);
    digitalWrite(PWDN_GPIO_NUM,LOW);
    delay(1000);
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      log_e("Retry camera init failed with error 0x%x", err);
      Serial.println("Retry could not start camera, rebooting...");
      delay(1000);
      ESP.restart();
      }
  }
}

////////////////////////////////////////
// SaveFrame
//
char * SaveFrame(camera_fb_t *frame) {
  static char directory[12];
  static char name[42];
  static unsigned int dirday;

  fs::FS &fs = SD_MMC;
  
  if(!frame) {
      if(DEBUG) Serial.println("SaveFrame(NULL) failure");
      return NULL;
  }
  if(DEBUG>1) Serial.printf("SaveFrame(%d byte image dim %d x %d);\r\n", fb->len,fb->width,fb->height);
  DateTimeParts p = DateTime.getParts();
  unsigned int bytes=0;
  unsigned int today;

  today=p.getYear() + p.getMonth() + p.getMonthDay();
  if(dirday != today) {
    // We just flipped to a new day, so we need a new directory
    sprintf(directory,"/%4.4d-%2.2d-%2.2d",p.getYear(),1+ p.getMonth(),p.getMonthDay());

    if(DEBUG) Serial.printf("SaveFrame() will need to createDir(%s)\r\n",directory);
    File root = fs.open(directory);
    if(root && root.isDirectory()){
      if(DEBUG>2) Serial.printf("Directory %s exists\r\n",directory);
    }
    else {
      if(!createDir(SD_MMC,directory)) {
          today=0;
           if(DEBUG) Serial.printf("SaveFrame() failed to createDir(%s)!!\r\n",directory);
      }
    }
    dirday=today;
  }

  sprintf(name,"%s/%2.2d-%2.2d-%2.2d_%d.jpg",directory,p.getHours(),p.getMinutes(),p.getSeconds(),(esp_timer_get_time()  / 10000));
  File file = fs.open(name, FILE_WRITE);
  if(!file){
        log_e("Failed to open file %s for writing.\r\n",name);
        if(DEBUG) Serial.printf("SaveFrame() failed to open %s for writing!\r\n",name);
        return(NULL);
        }
        
    toggleRedLED();
    unsigned long start= millis();
    bytes=file.write(frame->buf, frame->len);
    if(DEBUG) Serial.printf("Wrote %d bytes in %u millis\r\n" , frame->len,  millis()-start);
    file.close();
    toggleRedLED();
  
  if(bytes == frame->len)  
    log_i(TAG,"Wrote %d bytes to %s",frame->len,name);
  else
    log_w(TAG,"Wrote %d bytes (but expected %d) to %s",bytes, frame->len,name);
    
  esp_camera_fb_return(frame);  //  free the memory allocated by esp_camera_fb_get
  if(DEBUG) Serial.printf("SaveFrame() wrote %d to %s\r\n",bytes,name);
  return(name);
}

void SetupMicroSD() 
{
  log_i(TAG,"Starting SD Card");
  delay(1000);
  //SD_MMC.begin("/sdcard", true) disables the pin 4 issue which flashes the LED
  if(!SD_MMC.begin("/sdcard",true)) {
    log_w(TAG,"SD Card Mount Failed");
    Serial.println("Could not initialize SD storage");
    return;
    }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    log_w(TAG,"No SD Card attached");
    Serial.println("SD storage not found");
    return;
  }

  if(cardType == CARD_MMC)
    log_i(TAG,"Card is MMC");
  else if(cardType == CARD_SD)
    log_i(TAG,"Card is SDSC");
  else if(cardType == CARD_SDHC)
    log_i(TAG,"Card is SDHC");
  else 
    log_i(TAG,"Card is UNKNOWN");

  uint64_t cardSize = SD_MMC.cardSize();
  unsigned int cardSizeInMB = cardSize/(1024 * 1024);
  uint64_t bytesAvailable = SD_MMC.totalBytes(); 
  unsigned int bytesAvailableInMB = bytesAvailable/(1024 * 1024);
  uint64_t bytesUsed = SD_MMC.usedBytes();
  unsigned int bytesUsedInMB = bytesUsed/(1024 * 1024);
    
   log_i(TAG,"Card size: %u, with %u MB available and %u MB used. ", cardSizeInMB, bytesAvailableInMB, bytesUsedInMB );
   Serial.printf("Card size: %u, with %u available and %u  used. ", cardSize, bytesAvailable, bytesUsed);

if(DEBUG) listDir(SD_MMC,"/",1);

fs::FS &fs = SD_MMC;
File file = fs.open(BOOTLOG, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open BOOTLOG to append");
        log_w(TAG,"Failed to open %s for appending",BOOTLOG);
        file = fs.open(BOOTLOG, FILE_WRITE);
        if(!file) {
          Serial.println("Failed to open BOOTLOG for writing\r\n");
          return;
        }
    }
 
    if(file.print("\r\nSD start...\r\n")) 
      Serial.println("Wrote to BOOTLOG");
    else
      Serial.println("Failed to write to BOOTLOG");
  file.close();

}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            //if(levels){
            //    listDir(fs, file.path(), levels -1);
            //}
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}


void appendFile(fs::FS &fs, const char * path, const char * message) {
    log_d(TAG,"Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        log_w(TAG,"Failed to open file %s for appending",path);
        if(DEBUG) Serial.printf("Failed to open %s for append.\r\n",path);
        
        File file = fs.open(path, FILE_WRITE);
        if(!file){
          Serial.printf("Failed to open file %s for writing\r\n",path);
          return;
        }
    }
 
    if(file.print(message)) 
        log_d(TAG,"Message appended");
      else  
        log_i(TAG,"Append failed");

      file.close();
}

boolean createDir(fs::FS &fs, const char * path) {
    if(DEBUG) Serial.printf("\r\nCreating Dir: %s\r\n", path);

  File root = fs.open(path);
    if(root) {
      if(root.isDirectory()) {
        if(DEBUG) Serial.printf("Directory %s exists\r\n",path);
        root.close();
        return(false);
        }
      if(DEBUG) Serial.printf("Someting named %s exists, but apparently not a directory???\r\n",path);
      root.close();
      return(false);
    }
    
    if(fs.mkdir(path)){
        log_d(TAG,"Dir created");
        return(true);
    } else {
        log_e("mkdir failed");
        if(DEBUG) Serial.println("mkdir failed");
        return(false);
    }
}

void LogBoot() {
char timestamp[80];

  for(int i=0;i<15 && ! DateTime.isTimeValid(); i++) {
    if(DEBUG) Serial.print("T");
    delay(100);
  }
  
  if(DateTime.isTimeValid()) {
    sprintf(timestamp,"%s\r\n",DateTime.toISOString().c_str());
    appendFile(SD_MMC,BOOTLOG,timestamp);
    if(DEBUG) Serial.println("ime logged to BOOTLOG");
    return;
  }
  if(DEBUG) Serial.println("ime still not set, skipping BOOTLOG.");
}

//////////////////////
// SetupPIR
void SetupPIR() {
  pinMode(PIN_PIR, INPUT);
}

//////////////////////
//
// PIR
//      Read the PIR sensor
boolean PIR() {
  if( PIN_PIR_ACTIVE == digitalRead(PIN_PIR) )
      return(true);
  return(false);
}


 

/////////////////////////////////////
// PurgeSD
//
boolean PurgeSD(char * directory) {
 char fullpath[60];
 fs::FS &fs = SD_MMC;

 if(!directory) return(false);
 
 File base = fs.open(directory);
  if(!base){
    if(DEBUG) Serial.printf("PurgeSD(%s) Failed to open directory!\r\n",directory);
    return(false);
    }

 if(!base.isDirectory())  {
    if(DEBUG) Serial.printf("PurgeSD(%s) argument is not a directory!\r\n",directory);
    return(false);
 }

  int files=0;
  int directories=0;
  File file = base.openNextFile();
  while(file){
    sprintf(fullpath,"%s/%s",directory,file.name() );
    if(file.isDirectory()) {
        if(fs.rmdir(file.name())) 
            directories++;
    }
     else  {
        if(fs.remove(file.name())) 
          files++;
     }
    file = base.openNextFile();
  }
if(DEBUG) Serial.printf("PurgeSD(%s) removed %d files and %d directories\r\n",directory,files,directories);
if(fs.rmdir(directory))
  return(true);
return(false);
}


boolean PurgeOldDir(int days,int lookback) {
  fs::FS &fs = SD_MMC;
  time_t now;
  struct tm * timeinfo;
  char directory[12];
  unsigned int bytes=0;

  if (!DateTime.isTimeValid()) {
    log_i(TAG,"Failed to get time from DateTime server, retry.");
    if(DEBUG) Serial.println("Failed to PurgeOldDir as clock is not set!");
    return (false);
  }

  if(DEBUG>1) Serial.printf("Starting PurgeOldDir(%d,%d);\r\n",days,lookback);
  
  if(lookback)
       PurgeOldDir(1+days,--lookback);
  
  time(&now);
  now=now - (days * 86400);
  timeinfo = localtime(&now); 
  sprintf(directory,"/%4.4d-%2.2d-%2.2d",timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday);
  if(DEBUG>1) Serial.printf("Will purge %d days old directory %s\r\n",days,directory);
 
  return PurgeSD(directory);
}

boolean toggleRedLED() {
  static boolean state;

if(state) {
  state=false;
  digitalWrite(PIN_RED, HIGH);
}
else {
  state=true;
  digitalWrite(PIN_RED, LOW); 
  }
return(state);
}

#if (defined(__AVR__))
#include <avr\pgmspace.h>
#else
#include <pgmspace.h>
#endif

const char PROGMEM b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

/* 'Private' declarations */
inline void a3_to_a4(unsigned char * a4, unsigned char * a3);
inline void a4_to_a3(unsigned char * a3, unsigned char * a4);
inline unsigned char b64_lookup(char c);

int base64_encode(char *output, char *input, int inputLen) {
  int i = 0, j = 0;
  int encLen = 0;
  unsigned char a3[3];
  unsigned char a4[4];

  while(inputLen--) {
    a3[i++] = *(input++);
    if(i == 3) {
      a3_to_a4(a4, a3);

      for(i = 0; i < 4; i++) {
        output[encLen++] = pgm_read_byte(&b64_alphabet[a4[i]]);
      }

      i = 0;
    }
  }

  if(i) {
    for(j = i; j < 3; j++) {
      a3[j] = '\0';
    }

    a3_to_a4(a4, a3);

    for(j = 0; j < i + 1; j++) {
      output[encLen++] = pgm_read_byte(&b64_alphabet[a4[j]]);
    }

    while((i++ < 3)) {
      output[encLen++] = '=';
    }
  }
  output[encLen] = '\0';
  return encLen;
}

int base64_decode(char * output, char * input, int inputLen) {
  int i = 0, j = 0;
  int decLen = 0;
  unsigned char a3[3];
  unsigned char a4[4];


  while (inputLen--) {
    if(*input == '=') {
      break;
    }

    a4[i++] = *(input++);
    if (i == 4) {
      for (i = 0; i <4; i++) {
        a4[i] = b64_lookup(a4[i]);
      }

      a4_to_a3(a3,a4);

      for (i = 0; i < 3; i++) {
        output[decLen++] = a3[i];
      }
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 4; j++) {
      a4[j] = '\0';
    }

    for (j = 0; j <4; j++) {
      a4[j] = b64_lookup(a4[j]);
    }

    a4_to_a3(a3,a4);

    for (j = 0; j < i - 1; j++) {
      output[decLen++] = a3[j];
    }
  }
  output[decLen] = '\0';
  return decLen;
}

int base64_enc_len(int plainLen) {
  int n = plainLen;
  return (n + 2 - ((n + 2) % 3)) / 3 * 4;
}

int base64_dec_len(char * input, int inputLen) {
  int i = 0;
  int numEq = 0;
  for(i = inputLen - 1; input[i] == '='; i--) {
    numEq++;
  }

  return ((6 * inputLen) / 8) - numEq;
}

inline void a3_to_a4(unsigned char * a4, unsigned char * a3) {
  a4[0] = (a3[0] & 0xfc) >> 2;
  a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
  a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
  a4[3] = (a3[2] & 0x3f);
}

inline void a4_to_a3(unsigned char * a3, unsigned char * a4) {
  a3[0] = (a4[0] << 2) + ((a4[1] & 0x30) >> 4);
  a3[1] = ((a4[1] & 0xf) << 4) + ((a4[2] & 0x3c) >> 2);
  a3[2] = ((a4[2] & 0x3) << 6) + a4[3];
}

inline unsigned char b64_lookup(char c) {
  if(c >='A' && c <='Z') return c - 'A';
  if(c >='a' && c <='z') return c - 71;
  if(c >='0' && c <='9') return c + 4;
  if(c == '+') return 62;
  if(c == '/') return 63;
  return -1;
}

//https://github.com/zenmanenergy/ESP8266-Arduino-Examples/
String zenurlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}

///EOF///
