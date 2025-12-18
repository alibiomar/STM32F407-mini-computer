#include <ESP8266WiFi.h>
#include <ESP_Mail_Client.h>
#include <EEPROM.h>  // NEW: For storing WiFi credentials

// WiFi Configuration - Default values
String ssid = "YOUR_WIFI_SSID";
String password = "YOUR_WIFI_PASSWORD";

// EEPROM addresses for storing WiFi credentials
#define EEPROM_SIZE 512
#define SSID_ADDR 0
#define SSID_LEN_ADDR 100
#define PASS_ADDR 110
#define PASS_LEN_ADDR 210
#define MAGIC_ADDR 220
#define MAGIC_VALUE 0xAA  // Magic byte to check if EEPROM has valid data

// Gmail SMTP Configuration
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
#define AUTHOR_EMAIL "your-email@gmail.com"
#define AUTHOR_PASSWORD "your-app-password"

// SMTP Session
SMTPSession smtp;

// Data storage
String collectedText = "";
const int MAX_TEXT_LENGTH = 2000;

// Status tracking
bool wifiConnected = false;
unsigned long lastIPSendTime = 0;
const unsigned long IP_SEND_INTERVAL = 30000;
bool initialIPSent = false;

// NEW: WiFi credential buffers
String newSSID = "";
String newPassword = "";
bool wifiUpdatePending = false;

// RX buffer for interrupt-based reception
#define RX_BUFFER_SIZE 256
volatile char rxCircularBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;
volatile bool rxOverflow = false;

// Callback for SMTP status
void smtpCallback(SMTP_Status status);

// NEW: WiFi credential management functions
void saveWiFiCredentials(String ssid, String pass);
bool loadWiFiCredentials();
void connectToWiFi(String ssid, String pass);

void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(512);
  Serial.setTimeout(100);
  
  delay(500);
  
  // NEW: Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // NEW: Try to load saved WiFi credentials
  if (loadWiFiCredentials()) {
    sendMessage("DISPLAY:Loading saved WiFi");
  }
  
  // Connect to WiFi
  connectToWiFi(ssid, password);
  
  delay(1000);
  
  // Signal ready state
  sendMessage("ACK:ESP_READY");
  delay(200);
  
  // Send initial IP address
  sendIPAddress();
  initialIPSent = true;
  
  smtp.callback(smtpCallback);
  lastIPSendTime = millis();
}

void loop() {
  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      wifiConnected = false;
      sendMessage("DISPLAY:WiFi disconnected");
      sendMessage("WIFI:Disconnected");
    }
  } else {
    if (!wifiConnected) {
      wifiConnected = true;
      sendMessage("DISPLAY:WiFi reconnected");
      sendMessage("WIFI:Connected");
      // Send IP on reconnection
      if (millis() - lastIPSendTime > 5000) {
        sendIPAddress();
      }
    }
  }
  
  // NEW: Check if WiFi update is pending
  if (wifiUpdatePending) {
    wifiUpdatePending = false;
    
    // Disconnect from current WiFi
    WiFi.disconnect();
    delay(1000);
    
    sendMessage("WIFI:Connecting...");
    
    // Connect with new credentials
    connectToWiFi(newSSID, newPassword);
    
    if (WiFi.status() == WL_CONNECTED) {
      // Save credentials to EEPROM
      saveWiFiCredentials(newSSID, newPassword);
      ssid = newSSID;
      password = newPassword;
      
      sendMessage("WIFI:Connected!");
      delay(500);
      sendMessage("WIFI:Success");
      
      // Send new IP
      delay(1000);
      sendIPAddress();
    } else {
      sendMessage("WIFI:Failed to connect");
      delay(500);
      sendMessage("WIFI:Error - Check credentials");
      
      // Try to reconnect to old WiFi
      delay(1000);
      connectToWiFi(ssid, password);
    }
  }
  
  // Process incoming data from STM32
  processSTM32Data();
  
  // Small delay to prevent watchdog issues
  delay(10);
  yield();
}

// ============================================================================
// NEW: WiFi Credential Management Functions
// ============================================================================
void saveWiFiCredentials(String ssid, String pass) {
  // Write magic byte
  EEPROM.write(MAGIC_ADDR, MAGIC_VALUE);
  
  // Write SSID length
  uint8_t ssidLen = ssid.length();
  EEPROM.write(SSID_LEN_ADDR, ssidLen);
  
  // Write SSID
  for (uint8_t i = 0; i < ssidLen && i < 99; i++) {
    EEPROM.write(SSID_ADDR + i, ssid[i]);
  }
  
  // Write Password length
  uint8_t passLen = pass.length();
  EEPROM.write(PASS_LEN_ADDR, passLen);
  
  // Write Password
  for (uint8_t i = 0; i < passLen && i < 99; i++) {
    EEPROM.write(PASS_ADDR + i, pass[i]);
  }
  
  EEPROM.commit();
  
  sendMessage("ACK:WIFI_SAVED");
}

bool loadWiFiCredentials() {
  // Check magic byte
  if (EEPROM.read(MAGIC_ADDR) != MAGIC_VALUE) {
    return false;
  }
  
  // Read SSID length
  uint8_t ssidLen = EEPROM.read(SSID_LEN_ADDR);
  if (ssidLen == 0 || ssidLen > 99) {
    return false;
  }
  
  // Read SSID
  char ssidBuf[100];
  for (uint8_t i = 0; i < ssidLen; i++) {
    ssidBuf[i] = EEPROM.read(SSID_ADDR + i);
  }
  ssidBuf[ssidLen] = '\0';
  
  // Read Password length
  uint8_t passLen = EEPROM.read(PASS_LEN_ADDR);
  if (passLen == 0 || passLen > 99) {
    return false;
  }
  
  // Read Password
  char passBuf[100];
  for (uint8_t i = 0; i < passLen; i++) {
    passBuf[i] = EEPROM.read(PASS_ADDR + i);
  }
  passBuf[passLen] = '\0';
  
  // Update global variables
  ssid = String(ssidBuf);
  password = String(passBuf);
  
  return true;
}

void connectToWiFi(String ssid, String pass) {
  WiFi.begin(ssid.c_str(), pass.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    sendMessage("DISPLAY:WiFi Connected");
    sendMessage("WIFI:Connected");
  } else {
    wifiConnected = false;
    sendMessage("DISPLAY:WiFi connection failed");
    sendMessage("WIFI:Connection failed");
  }
}

// ============================================================================
// Optimized message sending with line ending
// ============================================================================
void sendMessage(const char* message) {
  Serial.println(message);
  Serial.flush();
  delay(5);
}

void sendMessage(String message) {
  Serial.println(message);
  Serial.flush();
  delay(5);
}

// ============================================================================
// Process data from STM32 with proper buffering
// ============================================================================
void processSTM32Data() {
  static String rxBuffer = "";
  static unsigned long lastRxTime = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    lastRxTime = millis();
    
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        rxBuffer.trim();
        
        // Process command
        processCommand(rxBuffer);
        
        // Clear buffer
        rxBuffer = "";
      }
    } else if (c != '\r') {
      rxBuffer += c;
      
      // Prevent buffer overflow
      if (rxBuffer.length() > 200) {
        sendMessage("ACK:BUFFER_OVERFLOW");
        sendMessage("DISPLAY:Buffer overflow");
        rxBuffer = "";
      }
    }
  }
  
  // Timeout: clear partial command after 2 seconds of inactivity
  if (rxBuffer.length() > 0 && (millis() - lastRxTime) > 2000) {
    rxBuffer = "";
  }
}

// ============================================================================
// Command processing
// ============================================================================
void processCommand(String cmd) {
  cmd.trim();
  
  // KEY command: Single character input
  if (cmd.startsWith("KEY:")) {
    if (cmd.length() >= 5) {
      char keyChar = cmd.charAt(4);
      
      if (keyChar == '\n' || keyChar == '\r') {
        if (collectedText.length() < MAX_TEXT_LENGTH) {
          collectedText += '\n';
          sendMessage("DISPLAY_UPDATE_TEXT");
        }
      } 
      else if (keyChar == '\b' || keyChar == 127) {
        if (collectedText.length() > 0) {
          collectedText.remove(collectedText.length() - 1);
          sendMessage("DISPLAY_UPDATE_TEXT");
        }
      } 
      else {
        if (collectedText.length() < MAX_TEXT_LENGTH) {
          collectedText += keyChar;
          sendMessage("DISPLAY_UPDATE_TEXT");
        } else {
          sendMessage("ACK:BUFFER_FULL");
          sendMessage("DISPLAY:Buffer full");
        }
      }
      sendMessage("ACK:KEY_OK");
    }
  }
  
  // NEW: WiFi SSID command
  else if (cmd.startsWith("WIFI_SSID:")) {
    newSSID = cmd.substring(10);
    newSSID.trim();
    
    if (newSSID.length() > 0 && newSSID.length() <= 32) {
      sendMessage("ACK:SSID_RECEIVED");
      sendMessage("WIFI:SSID received");
    } else {
      sendMessage("ACK:SSID_INVALID");
      sendMessage("WIFI:Error - Invalid SSID");
    }
  }
  
  // NEW: WiFi Password command
  else if (cmd.startsWith("WIFI_PASS:")) {
    newPassword = cmd.substring(10);
    newPassword.trim();
    
    // Allow empty password for open networks
    if (newPassword.length() <= 64) {
      sendMessage("ACK:PASS_RECEIVED");
      if (newPassword.length() == 0) {
        sendMessage("WIFI:Open network (no password)");
      } else {
        sendMessage("WIFI:Password received");
      }
    } else {
      sendMessage("ACK:PASS_INVALID");
      sendMessage("WIFI:Error - Password too long");
    }
  }
  
  // NEW: WiFi Connect command
  else if (cmd.startsWith("WIFI_CONNECT")) {
    if (newSSID.length() > 0) {
      wifiUpdatePending = true;
      sendMessage("ACK:WIFI_CONNECTING");
      sendMessage("WIFI:Updating WiFi...");
    } else {
      sendMessage("ACK:WIFI_INCOMPLETE");
      sendMessage("WIFI:Error - Missing credentials");
    }
  }
  
  // IP request commands
  else if (cmd.startsWith("GET_IP") || 
           cmd.startsWith("TEST:REQUEST_IP") || 
           cmd.indexOf("REQUEST_IP") >= 0) {
    sendIPAddress();
  }
  
  // Email sending command
  else if (cmd.startsWith("SEND_EMAIL:")) {
    String recipient = cmd.substring(11);
    recipient.trim();
    
    if (recipient.length() > 0) {
      handleSendEmail(recipient);
    } else {
      sendMessage("EMAIL:Error - No recipient");
      sendMessage("DISPLAY:No recipient");
    }
  }
  
  // Clear text buffer command
  else if (cmd.startsWith("CLEAR_TEXT")) {
    collectedText = "";
    sendMessage("ACK:TEXT_CLEARED");
    sendMessage("DISPLAY:Text cleared");
  }
  
  // Get text length command
  else if (cmd.startsWith("GET_TEXT_LEN")) {
    String response = "ACK:TEXT_LEN:" + String(collectedText.length());
    sendMessage(response);
  }
  
  // Unknown command
  else {
    String response = "ACK:UNKNOWN_CMD:" + cmd;
    sendMessage(response);
  }
}

// ============================================================================
// Email handling with detailed status updates
// ============================================================================
void handleSendEmail(String recipient) {
  String subject = "Text from STM32 Keyboard";
  
  recipient.trim();
  recipient.toLowerCase();
  
  // Basic email validation
  int atPos = recipient.indexOf('@');
  int dotPos = recipient.lastIndexOf('.');
  
  if (atPos < 1 || dotPos < atPos + 2 || dotPos >= recipient.length() - 1) {
    sendMessage("EMAIL:Invalid email format");
    sendMessage("DISPLAY:Invalid email");
    delay(200);
    return;
  }
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    sendMessage("EMAIL:WiFi not connected");
    sendMessage("DISPLAY:WiFi not connected");
    delay(200);
    return;
  }
  
  // Check if there's text to send
  if (collectedText.length() == 0) {
    sendMessage("EMAIL:No text to send");
    sendMessage("DISPLAY:No text to send");
    delay(200);
    return;
  }
  
  // Send progressive status updates
  sendMessage("EMAIL:Preparing...");
  delay(300);
  
  sendMessage("EMAIL:Connecting to SMTP...");
  delay(300);
  
  // Send the email
  bool success = sendEmailViaSMTP(recipient, subject, collectedText);
  
  delay(500);
  
  // Report results with clear status codes
  if (success) {
    sendMessage("EMAIL:Sent successfully!");
    delay(300);
    
    // Clear text after successful send
    collectedText = "";
    
    sendMessage("EMAIL:success");
    delay(200);
    
    sendMessage("ACK:EMAIL_SENT");
  } else {
    sendMessage("EMAIL:Send failed!");
    delay(300);
    
    sendMessage("EMAIL:fail");
    delay(200);
    
    sendMessage("ACK:EMAIL_FAILED");
  }
}

// ============================================================================
// SMTP email sending
// ============================================================================
bool sendEmailViaSMTP(String recipient, String subject, String message) {
  if (message.length() == 0) {
    message = "(Empty message)";
  }
  
  // Configure SMTP session
  Session_Config config;
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";
  
  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 1;
  config.time.day_light_offset = 0;
  
  sendMessage("EMAIL:Authenticating...");
  
  // Connect to SMTP server
  if (!smtp.connect(&config)) {
    sendMessage("EMAIL:Connection failed");
    delay(200);
    return false;
  }
  
  // Check login
  if (!smtp.isLoggedIn()) {
    sendMessage("EMAIL:Login failed");
    smtp.closeSession();
    delay(200);
    return false;
  }
  
  sendMessage("EMAIL:Sending message...");
  
  // Create email message
  SMTP_Message smtp_message;
  
  smtp_message.sender.name = F("STM32 Keyboard");
  smtp_message.sender.email = AUTHOR_EMAIL;
  smtp_message.subject = subject;
  smtp_message.addRecipient(F("Recipient"), recipient);
  
  // Build message body with statistics
  String textMsg = "Keyboard Text Data:\n\n";
  textMsg += "=================================\n";
  textMsg += message;
  textMsg += "\n=================================\n\n";
  
  // Add statistics
  textMsg += "Total Characters: " + String(message.length()) + "\n";
  
  // Count words
  int wordCount = 0;
  String temp = message;
  temp.trim();
  if (temp.length() > 0) {
    wordCount = 1;
    for (unsigned int i = 0; i < temp.length(); i++) {
      if (temp.charAt(i) == ' ' || temp.charAt(i) == '\n' || temp.charAt(i) == '\t') {
        if (i + 1 < temp.length() && temp.charAt(i + 1) != ' ') {
          wordCount++;
        }
      }
    }
  }
  textMsg += "Total Words: " + String(wordCount) + "\n";
  
  // Count lines
  int lineCount = 1;
  for (unsigned int i = 0; i < message.length(); i++) {
    if (message.charAt(i) == '\n') {
      lineCount++;
    }
  }
  textMsg += "Total Lines: " + String(lineCount) + "\n";
  textMsg += "\nSent from ESP8266 STM32 Keyboard Display System";
  
  smtp_message.text.content = textMsg.c_str();
  smtp_message.text.charSet = "us-ascii";
  smtp_message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  smtp_message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_normal;
  
  // Send email
  if (!MailClient.sendMail(&smtp, &smtp_message, true)) {
    String errorMsg = smtp.errorReason();
    String errResponse = "EMAIL:Error - " + errorMsg;
    sendMessage(errResponse);
    smtp.closeSession();
    delay(200);
    return false;
  }
  
  smtp.closeSession();
  
  return true;
}

// ============================================================================
// SMTP callback
// ============================================================================
void smtpCallback(SMTP_Status status) {
  if (status.success()) {
    sendMessage("EMAIL:Message sent success");
  }
}

// ============================================================================
// Send IP address to STM32
// ============================================================================
void sendIPAddress() {
  if (WiFi.status() == WL_CONNECTED) {
    String ipMsg = "IP:" + WiFi.localIP().toString();
    sendMessage(ipMsg);
    lastIPSendTime = millis();
    
    // Debug acknowledgment
    String ackMsg = "ACK:IP_SENT:" + WiFi.localIP().toString();
    sendMessage(ackMsg);
  } else {
    sendMessage("IP:Not connected");
  }
}