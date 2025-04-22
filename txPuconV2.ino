#include <ModbusMaster.h>
#include "LoRa_E220.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <esp_task_wdt.h>

#define ID   1
#define ENABLE_RSSI true
#define FREQUENCY_915
#define TIMEPERIOD  45000
#define NETOK       13
#define ONOFF       14
#define WDT_TIMEOUT 30      // WDT Timeout in seconds
#define RX   32 
#define TX   33
#define ACKWAITTIME 5000
#define RETRYNUM    3

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define DESTINATION_ADDL 2
#define CHANNEL  65

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum st
{
  ON,
  OFF
};

int counter = 0;
int sendCounter;
ResponseStatus result;
bool waitResponse;
bool netOk;

unsigned long timer;
unsigned long arTimer;

struct msgNivel
{
  char type[6];
  byte id;
  byte counter;
  uint16_t nivel;
};

struct msgAck
{
  char type[6];
  byte id;
  byte origId;
  byte counter;
  byte onOff;
  byte modo;
};

unsigned int nivel;
st state;

ModbusMaster node;
// ---------- esp32 pins --------------
LoRa_E220 e220ttl(&Serial2, 27, 25, 26); //  RX AUX M0 M1

void setup() 
{
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1, RX, TX);    // Puerto lectura Modbus RTU

  if(!display.begin(i2c_Address, true))
  {
    Serial.println("Error en inicializacion de display OLED");
  }
 //display.setContrast (0); // dim display
 
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Satori SpA ");
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Tank Level Meter V1");
  display.setCursor(0, 50);
  display.print("Init OK...");
 
  display.display();

  e220ttl.begin();
  node.begin(1, Serial1);

  initLoRa();

  pinMode(NETOK, OUTPUT);
  pinMode(ONOFF, OUTPUT);

  digitalWrite(NETOK, HIGH);   // Logica inversa
  digitalWrite(ONOFF, HIGH);

  waitResponse = false;
  Serial.println("init  txPucon V2 ok");

  state = OFF; 
  netOk = false;
   
  timer =  millis();

  esp_task_wdt_init(WDT_TIMEOUT, true); // Initialize ESP32 Task WDT
  esp_task_wdt_add(NULL);               // Subscribe to the Task WDT
}

void loop() 
{
  if((millis() - timer) > TIMEPERIOD)
  {
    timer = millis();
    if(checkNivel())
    {
      showNivel();
      sendNivel();
      waitResponse = true;
      arTimer = millis();
    }
  }

  if(waitResponse)
  {
    if(e220ttl.available()>1) 
    {
        ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(msgAck));
        struct msgAck m2 = *(msgAck*) rc.data;
        if(m2.id == ID)
        {
          if(m2.origId == ID)
          {
            counter++;
            Serial.println("Ack recibido");
            waitResponse = false;
            digitalWrite(NETOK, LOW);    // Encender led NetOk
            netOk = true;
            
            if(m2.onOff == 1)
            {
               digitalWrite(ONOFF, LOW);
               state = ON;
            }
            else
            {
              digitalWrite(ONOFF, HIGH);
              state = OFF;
            }
          }      
        }
    }
     
    else 
    {
       if((millis() - arTimer) > ACKWAITTIME)
       {
         if(sendCounter >= RETRYNUM)   //Intentar reenvio hasta 5 veces
         {
           Serial.println("No ack recibido desde HUB");
           digitalWrite(NETOK, HIGH);    // Apagar led NetOk
           netOk = false;
           waitResponse = false;
         }        
         else
         {
           Serial.println("Reenviando datos");
           struct msgNivel m1 = {"NIVEL", ID, counter, nivel};
           e220ttl.sendFixedMessage(0, DESTINATION_ADDL, CHANNEL, &m1,sizeof(msgNivel));
           sendCounter++;
           arTimer = millis();
         } 
       }
    }
  }
  
  esp_task_wdt_reset();
}

bool checkNivel()
{
  uint8_t j, result;
  uint16_t dataVal;
  bool sendOk = false;

  result = node.readHoldingRegisters(4,1);  //Leer 1 registros de 16 bits desde address 4
  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    dataVal = node.getResponseBuffer(j);
    Serial.print("Nivel agua: ");
    Serial.print(dataVal);
    Serial.println(" mm");
    nivel = dataVal;
    sendOk = true;
  }

  else
  {
    Serial.println("Error en lectura Modbus");
    display.setTextSize(2);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Error Modbus RTU");
    display.display();
    sendOk = false;
  }

  return sendOk;
}

void sendNivel()
{
   struct msgNivel m1 = {"NIVEL", ID, counter, nivel};
   result = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, CHANNEL, &m1,sizeof(msgNivel));
   sendCounter = 0;
}

void showNivel()
{
   display.setTextSize(2);
   display.clearDisplay();
   display.setCursor(0, 0);
   display.print("Nivel: ");
   display.setCursor(25, 25);
   display.print(nivel);
   display.print(" mm");
   display.display(); 
}

void initLoRa()
{
  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*) c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
   
  //printParameters(configuration);
  configuration.ADDL = ID;
  configuration.ADDH = 0;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
  configuration.CHAN = CHANNEL; // Communication channel
   
  // Set configuration changed and set to not hold the configuration
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  printParameters(configuration);
  c.close();
}

void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


  Serial.println("----------------------------------------");
}
