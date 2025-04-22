// If you want use RSSI uncomment
#define ENABLE_RSSI true
#define FREQUENCY_915
#define TINY_GSM_MODEM_SIM7600

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "Arduino.h"
#include "LoRa_E220.h"
#include "ZMPT101B.h"
#include <ArduinoJson.h>
#include "time.h"
#include <esp_task_wdt.h>

#define ID       2
#define CHANNEL  65
#define DESTINATION_ADDL 3
#define RXLoRa  18
#define TXLoRa  19
#define VAC     34
#define PVERDE  12
#define PAMARI  13
#define MEASTIME  10000       // Check voltaje AC cada 10 s
#define SENDTIME  300000      // Envio de datos por MQTT cada 5 min
#define MAXTIME   300000      // Maximo tiempo de timer de tx y rx sin actividad 

#define WDT_TIMEOUT 60     // set watchdog a 60 seg
#define UART_BAUD   115200
#define MODEM_TX      17
#define MODEM_RX      16
#define MODEM_PWRKEY  4
#define MQTT_MAXBUFFER_SIZE  2048
#define FIXEDSIZE  768
#define UNITSIZE   50
#define ARRAYSIZE  20

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial2
//#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient  mqtt(client);

boolean reboot;
bool netOK;
bool gprsOK;
bool rtcOK;
bool modemOK;

uint32_t lastReconnectAttempt = 0;

const char* ntpServer = "pool.ntp.org";
const int TZ = -16;    // Timezone Santiago, Chile

const char apn[]      = "m2m.entel.cl";
const char gprsUser[] = "entelpcs";
const char gprsPass[] = "entelpcs";

const String modID = "TvUPAaomkX9F6viWOUzDYs4XB2PoPqqZ";

// MQTT details
const char* broker = "164.92.88.189";
const char* mqtt_pass = "hangel2024";
const char* mqtt_user = "hangel"; 
const char* topicPiedraNegra = "satori/PiedraNegra";

int retries = 0;

// ---------- esp32 pins --------------
LoRa_E220 e220ttl(&Serial1, 27, 25, 26); //  RX AUX M0 M1
ZMPT101B voltageSensor(VAC, 50.0);

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

struct msgElect
{
  char type[6];
  byte id;
  int16_t voltA;
  int16_t voltB;
  int16_t voltC;
  int16_t ampA;
  int16_t ampB;
  int16_t ampC;
  int16_t actPowA;
  int16_t actPowB;
  int16_t actPowC;
  int16_t reactPowA;
  int16_t reactPowB;
  int16_t reactPowC;
  int16_t powFA;
  int16_t powFB;
  int16_t powFC;
  int16_t freqz;
  uint16_t totPH;
  uint16_t totPL;
  uint16_t totQH;
  uint16_t totQL;
};

struct msgOnOff
{
  char type[6];
  byte id;
  byte onOff;
  byte modo;
};

unsigned long timer;
unsigned long sendTimer;
unsigned long txOnTimer;
unsigned long rxOnTimer;
bool acOK;
const float minVoltage = 100.0;
bool bombaOn;
byte modo;
uint16_t nivel;
int txRSSI;   // rssi en dBm
int rxRSSI;   // rssi en dBm
// Parámetros eléctricos bomba
float voltA;
float voltB;
float voltC;
float ampA;
float ampB;
float ampC;
int16_t actPowA;
int16_t actPowB;
int16_t actPowC;
int16_t reactPowA;
int16_t reactPowB;
int16_t reactPowC;
float powFA;
float powFB;
float powFC;
float freqz;
double totP;
double totQ;

float latitude;
float longitude;

struct Event
{
  unsigned long ts;
  byte cod;        
};

Event eventsFIFO[ARRAYSIZE];
int eventsCounter;

bool LoRaTxOn;
bool LoRaRxOn;

void setup() 
{
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1, RXLoRa, TXLoRa);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  delay(500);

  // Startup all pins and UART
  e220ttl.begin();

  initLoRa();

  Serial.println("Repetidor Pucon V2");
  acOK = true;
  bombaOn = false;
  LoRaTxOn = LoRaRxOn = false;

  modemPowerOn();

  Serial.println("SIM7600 serial initialize");
  delay(1000);

    // Restart takes quite some time
  // To skip it, call init() instead of restart()

  Serial.println("Initializing modem...");

  modemOK = modem.init(); 
  while ((!modemOK) and (retries < 5)) 
  {
      Serial.println("Failed to restart modem, delaying 10s and retrying");
      retries++;
      delay(10000);
      modemOK = modem.init();
  }

  if(modemOK)
  {
    String ret;
    ret = modem.setNetworkMode(2);
    DBG("setNetworkMode:", ret);
  
    String name = modem.getModemName();
    DBG("Modem Name:", name);
  
    String modemInfo = modem.getModemInfo();
    DBG("Modem Info:", modemInfo);
  
    netOK = false;
    rtcOK = false;
  
    SerialMon.print("Waiting for network...");
    if(!modem.waitForNetwork()) {
        SerialMon.println(" fail");
    }
  
    else
      SerialMon.println(" success");
  
    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
        netOK = true;
    }
  
    if(netOK)
    {
      // GPRS connection parameters are usually set after network registration
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          SerialMon.println(" fail");
      }
      SerialMon.println(" success");
    
      if (modem.isGprsConnected()) {
          SerialMon.println("GPRS connected");
          initRTC();
          rtcOK = true;
      }
    }
  
    else
    {
      Serial.println("Error conectando a red celular");
    }
  }


  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setBufferSize(MQTT_MAXBUFFER_SIZE);

  enableGPS();

  pinMode(PVERDE, OUTPUT);
  pinMode(PAMARI, OUTPUT);

  digitalWrite(PVERDE, LOW);   // Luces apagadas, lógica directa
  digitalWrite(PAMARI, LOW);

  //Init timers
  txOnTimer = millis();
  rxOnTimer = millis();
  timer = millis();
  sendTimer = millis();
  
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void loop() 
{
	// If something available
  if (e220ttl.available()>1) 
  {
     char type[6];
  	 ResponseContainer rs = e220ttl.receiveInitialMessage(sizeof(type));
     String typeStr = rs.data;
     
     if (rs.status.code!=1)
     {
       Serial.println(rs.status.getResponseDescription());
     }
     else
     {
        if(typeStr == "NIVEL")
        {
          struct MessageNivelPartial 
          {
            byte id;
            byte counter;
            uint16_t nivel;
          };

          // Reenviar datos a receptor final (ID 3)
          ResponseStructContainer rsc = e220ttl.receiveMessageRSSI( sizeof(MessageNivelPartial));
          struct MessageNivelPartial msg = *(MessageNivelPartial*) rsc.data;

          nivel = msg.nivel;
          txRSSI = -(256-rsc.rssi);
          
          Serial.print("NIVEL / ");
          Serial.print(msg.id);
          Serial.print(" / ");
          Serial.print(msg.counter);
          Serial.print(" / ");
          Serial.println(nivel);
          Serial.print("RSSI Tx(dBm): "); Serial.println(txRSSI, DEC);

          if(msg.id == 1)
          {
            Serial.println("Reenviando datos a receptor final");
            struct msgNivel m2 = { "NIVEL", msg.id, msg.counter, msg.nivel};
            e220ttl.sendFixedMessage(0, 3,CHANNEL, &m2,sizeof(msgNivel));
            LoRaTxOn = true;
            txOnTimer = millis();
          }

          rsc.close();
        }
        
        else if(typeStr == "ACKNG")
        {
          struct MessageAckPartial 
          {
            byte id;
            byte origId;
            byte counter;
            byte onOff;
            byte modo;
          };

          // Reenviar datos a receptor final (ID 3)
          ResponseStructContainer rsc = e220ttl.receiveMessageRSSI( sizeof(MessageAckPartial));
          struct MessageAckPartial m = *(MessageAckPartial*) rsc.data;

          rxRSSI = -(256-rsc.rssi);
          modo = m.modo;

          Serial.print("ACKNG / ");
          Serial.print(m.id);
          Serial.print(" / ");
          Serial.print(m.origId);
          Serial.print(" / ");
          Serial.print(m.counter);
          Serial.print(" / ");
          Serial.print(m.onOff);
          Serial.print(" / ");
          Serial.println(modo);
          Serial.print("RSSI Rx(dBm): "); Serial.println(rxRSSI, DEC);

          if((m.id == 3) and (m.origId == 1))
          {
            // Reenviar ack a transmisor original (ID 1)
            Serial.println("Reenviando ack a transmisor original");
            struct msgAck m2 = { "ACKNG", ID, m.origId, m.counter, m.onOff};
            e220ttl.sendFixedMessage(0, 1,CHANNEL, &m2,sizeof(msgAck));
            LoRaRxOn = true;
            rxOnTimer = millis();

            if(m.onOff == 1)
            {
              if(!bombaOn)
              {
                Serial.println("Bomba encendida");
                // Enviar evento de bomba encendida 
                Event curEvent;
                curEvent.ts = getEpochTime();
                curEvent.cod = 1;
                insertNewEvent(curEvent);
              }
              
              digitalWrite(PAMARI, HIGH);
              bombaOn = true;
            }
            else if(m.onOff == 0)
            {
              if(bombaOn)
              {
                Serial.println("Bomba apagada");
                // Enviar evento de bomba apagada
                Event curEvent;
                curEvent.ts = getEpochTime();
                curEvent.cod = 0;
                insertNewEvent(curEvent);
              }
              
              digitalWrite(PAMARI, LOW);
              bombaOn = false;
            }

            Serial.println("-------------------------------------");
          }

          rsc.close();
        }
        
        else if(typeStr == "ELECT")
        {
          struct MessageElectPartial
          {
            byte id;
            int16_t voltA;
            int16_t voltB;
            int16_t voltC;
            int16_t ampA;
            int16_t ampB;
            int16_t ampC;
            int16_t actPowA;
            int16_t actPowB;
            int16_t actPowC;
            int16_t reactPowA;
            int16_t reactPowB;
            int16_t reactPowC;
            int16_t powFA;
            int16_t powFB;
            int16_t powFC;
            int16_t freqz;
            uint16_t totPH;
            uint16_t totPL;
            uint16_t totQH;
            uint16_t totQL;
          };

           // Leer datos
          ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(MessageElectPartial));
          struct MessageElectPartial m = *(MessageElectPartial*) rsc.data;
          
          if(m.id == 3)
          {  
            Serial.println("Parámetros eléctricos de bomba recibidos");
            Serial.println("----------------------------------------");
              // Recibir datos de power meter
            voltA = (float)m.voltA/10.0;
            voltB = (float)m.voltB/10.0;
            voltC = (float)m.voltC/10.0;
            ampA = (float)m.ampA/100.0;
            ampB = (float)m.ampB/100.0;
            ampC = (float)m.ampC/100.0;
            actPowA = m.actPowA;
            actPowB = m.actPowB;
            actPowC = m.actPowC;
            reactPowA = m.reactPowA;
            reactPowB = m.reactPowB;
            reactPowC = m.reactPowC;
            powFA = (float)m.powFA/1000.0;
            powFB = (float)m.powFB/1000.0;
            powFC = (float)m.powFC/1000.0;
            freqz = (float)m.freqz/100.0;
          
            totP = (double)((m.totPH << 16) + m.totPL)/100.0;     // En kWH
            totQ = (double)((m.totQH << 16) + m.totQL)/100.0;     // en kVAH

            //showData();
            LoRaRxOn = true;
            rxOnTimer = millis();
          }
        }

        else if(typeStr == "ONOFF")
        {
          struct MessageOnOffPartial
          {
             byte id;
             byte onOff;
             byte modo;
          };
                     // Leer datos
          ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(MessageOnOffPartial));
          struct MessageOnOffPartial m = *(MessageOnOffPartial*) rsc.data;
          
          if(m.id == 3)
          {  
            Serial.println("Mensaje On/Off de bomba recibido");
            LoRaRxOn = true;
            rxOnTimer = millis();

            modo = m.modo;

            if(m.onOff == 1)
            {
              if(!bombaOn)
              {
                Serial.println("Bomba encendida");
                // Enviar evento de bomba encendida 
                Event curEvent;
                curEvent.ts = getEpochTime();
                curEvent.cod = 1;
                insertNewEvent(curEvent);
              }
              
              digitalWrite(PAMARI, HIGH);
              bombaOn = true;
            }
            else if(m.onOff == 0)
            {
              if(bombaOn)
              {
                Serial.println("Bomba apagada");
                // Enviar evento de bomba apagada
                Event curEvent;
                curEvent.ts = getEpochTime();
                curEvent.cod = 0;
                insertNewEvent(curEvent);
              }
              
              digitalWrite(PAMARI, LOW);
              bombaOn = false;
            }
          }

          Serial.println("---------------------------------");
        }
     }
   }

   if((millis()-timer) > MEASTIME)
   {
     timer = millis();
     checkVAC();

     if((millis()-txOnTimer) > MAXTIME)
     {
        Serial.println("Enlace con transmisor LoRa perdido");
        LoRaTxOn = false;
     }

     if((millis()-rxOnTimer) > MAXTIME)
     {
        Serial.println("Enlace con receptor LoRa perdido");
        LoRaRxOn = false;
     }

     if((LoRaTxOn == false) or (LoRaRxOn == false))
     {
        digitalWrite(PVERDE, LOW); 
     }
     else
     {
        digitalWrite(PVERDE, HIGH);
     }
   }

   if((millis()-sendTimer) > SENDTIME)
   {
     sendTimer = millis();
     if(acOK)
     {
        Serial.println("Enviando datos por MQTT");
        // Enviar datos!!!!!!!!!!!!!
        sendMsg();
     }
     else
        Serial.println("Energía AC desconectada. No hay envío de datos!!!"); 
   }

   esp_task_wdt_reset();    // Reset watchdog
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
  //configuration.SPED.airDataRate = AIR_DATA_RATE_010_24; 
  //configuration.OPTION.transmissionPower = POWER_22;
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
void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD: "));  Serial.print(moduleInformation.COMMAND, HEX);Serial.print(" ");Serial.print(moduleInformation.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(moduleInformation.LENGHT, DEC);

  Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}

void checkVAC()
{
   float vrms = voltageSensor.getRmsVoltage();
   //Serial.println(vrms);
   
   if(vrms < minVoltage)
   {
      if(acOK == true)
      {
        //Enviar alarma de corte de energía por red Celular!!!!! 
        Serial.println("Corte de energía!!!!!!!!!!!!!!");
        sendMsgAlarma(false);
      }
      
      acOK = false;
   }
   else
   {
      if(acOK == false)
      {
        //Enviar aviso de reposición de energía por red Celular!!!!!
        Serial.println("Reposición de energía");
        sendMsgAlarma(true);
      }

      acOK = true;
   }
}

void modemPowerOn()
{
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
}

void initRTC()
{
  int   year3    = 0;
  int   month3   = 0;
  int   day3     = 0;
  int   hour3    = 0;
  int   min3     = 0;
  int   sec3     = 0;
  float timezone = 0;
  
  DBG("Asking modem to sync with NTP");
  modem.NTPServerSync(ntpServer, TZ);

  for (int8_t i = 5; i; i--) 
  {
    DBG("Requesting current network time");
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,&timezone)) 
    {
      DBG("Year:", year3, "\tMonth:", month3, "\tDay:", day3);
      DBG("Hour:", hour3, "\tMinute:", min3, "\tSecond:", sec3);
      DBG("Timezone:", timezone);
      break;
    } 
    else 
    {
      DBG("Couldn't get network time, retrying in 15s.");
      delay(15000L);
    }
  }
  
  DBG("Retrieving time again as a string");
  String timeStr = modem.getGSMDateTime(DATE_FULL);
  DBG("Current Network Time:", timeStr);  

  struct tm time = {0};
  time_t epoch = 0;
  int hh, mm, ss, yy, mon, day, tz;
  sscanf(timeStr.c_str(), "%d/%d/%d,%d:%d:%d-%d", &yy, &mon, &day, &hh, &mm, &ss, &tz);

  time.tm_hour = hh;
  time.tm_min = mm;
  time.tm_sec = ss;
  time.tm_year = 2000 + yy - 1900;
  
  time.tm_mon = mon - 1;
  time.tm_mday = day;
  time.tm_isdst = -1;

  epoch = mktime(&time);
  struct timeval tv;
  tv.tv_sec = epoch+1;
  tv.tv_usec = 0;
  
  settimeofday(&tv, NULL);

  setenv("TZ", "CLT,M4.1.0/0,M9.1.0/0", 1);
  tzset();
  printLocalTime();
}

void printLocalTime()
{
  String cDT = getCurrentDateTime();
  Serial.println(cDT); 
}

unsigned long getEpochTime() 
{
  if(modemOK)
  {
    time_t now;
    struct tm timeinfo;
    for(int i = 0; i < 10; i++)
    {
      if(getLocalTime(&timeinfo))
        break;
      delay(100);
    }
    time(&now);
    return now-3600*(TZ/4);
  }
  else
    return 0;
}

String getCurrentDateTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return String("");
  }

  String curDay;
  byte d = timeinfo.tm_mday;
  if(d < 10)
    curDay = "0" + String(d);
  else
    curDay = String(d);
    
  String curMonth;

  byte m = timeinfo.tm_mon + 1;

  if(m < 10)
    curMonth = "0" + String(m);
  else
    curMonth = String(m);     
  
  String curYear = String(timeinfo.tm_year +1900);
  
  String curHour;

  byte hh = timeinfo.tm_hour;
 
  if(hh < 10)
    curHour = "0" + String(hh);
  else
    curHour = String(hh); 

  String curMin;
  
  byte mm = timeinfo.tm_min;
 
  if(mm < 10)
    curMin = "0" + String(mm);
  else
    curMin = String(mm); 

  String curSec;
   
  byte ss = timeinfo.tm_sec;
 
  if(ss < 10)
    curSec = "0" + String(ss);
  else
    curSec = String(ss);
       
  String curDateTime = curDay + "/" + curMonth + "/" + curYear + " " + curHour + ":" + curMin + ":" + curSec;
  return curDateTime; 
}

bool checkNetwork()
{
    // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) 
  {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(30000L, true)) 
      {
          SerialMon.println(" fail");
          //delay(10000);
          return false;
      }
      if (modem.isNetworkConnected()) 
      {
          SerialMon.println("Network re-connected");
      }
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected()) 
      {
          SerialMon.println("GPRS disconnected!");
          SerialMon.print(F("Connecting to "));
          SerialMon.print(apn);
          if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
          {
              SerialMon.println(" fail");
              //delay(10000);
              return false;
          }
          if (modem.isGprsConnected()) 
          {
              SerialMon.println("GPRS reconnected");
              if(!rtcOK)
              {
                initRTC();
                rtcOK = true;
              }      
          }
      }
  }

  return true;
}

bool connectMqtt()
{
  bool connectOk = false;
  byte nTries = 0;
  // Conectar a broker MQTT
  while (!mqtt.connected() and nTries < 10) 
  {
    Serial.println("Connecting to MQTT...");
 
    if (mqtt.connect("ESP32Client", mqtt_user, mqtt_pass )) 
    {
      Serial.println("connected");
      connectOk = true;
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(mqtt.state());
      nTries++;
      delay(2000);
    }
  }

  return connectOk;
}

void disconnectMqtt()
{
  mqtt.disconnect();

  Serial.println("Disconnecting from MQTT broker");
  Serial.println("******************************************");  
}

double round2(double value) 
{
  return (int)(value * 100 + 0.5) / 100.0;
}

void sendMsg()
{
  bool sendOk = false;
  int rssi = 0;
  
  if(modemOK)
  {
    int csq = modem.getSignalQuality();
    rssi = (56*(csq - 2) - 3052)/28;

    if(!getGPS())
       Serial.println("Error getting GPS data");
    else
    {
       Serial.print("latitude:"); Serial.print(latitude,5);
       Serial.print(", longitude:"); Serial.println(longitude,5);
    }
    
    if(checkNetwork())
    {
      // Conectar a broker MQTT
      bool procOk = connectMqtt();
    
      if(procOk)
      {
        String IMEI = modem.getIMEI();
        String ICCID = modem.getSimCCID();
        DynamicJsonDocument doc(FIXEDSIZE+(eventsCounter * UNITSIZE));

        String bombaStr = "";
        if(bombaOn)
           bombaStr += "ON";
        else
           bombaStr += "OFF";    

        String modoStr = "";
        if(modo == 1)
           modoStr = "MAN";
        else
           modoStr = "AUT";

        String LoRaTxStr = "";
        if(LoRaTxOn)
           LoRaTxStr = "ON";
        else
           LoRaTxStr = "OFF";

        String LoRaRxStr = "";
        if(LoRaRxOn)
           LoRaRxStr = "ON";
        else
           LoRaRxStr = "OFF";

        doc["tipo"] = "Normal";
        doc["id"] = modID;
        doc["ts"] = getEpochTime();
        doc["rssi"] = rssi;
        doc["IMEI"] = IMEI;
        doc["ICCID"] = ICCID;

        JsonObject GPS = doc["GPS"].to<JsonObject>();
        GPS["lat"] = latitude;
        GPS["lon"] = longitude;

        JsonObject data = doc["data"].to<JsonObject>();
        data["nivel"] = nivel;
        data["LoRaTx"] = LoRaTxStr;
        data["LoRaRx"] = LoRaRxStr;
        data["txRSSI"] = txRSSI;
        data["rxRSSI"] = rxRSSI;
        data["bOnOff"] = bombaStr;
        data["modo"] = modoStr;
        
        JsonObject data_pElect = data["pElect"].to<JsonObject>();
        data_pElect["voltA"] = round2(voltA);
        data_pElect["voltB"] = round2(voltB);
        data_pElect["voltC"] = round2(voltC);
        data_pElect["ampA"] = round2(ampA);
        data_pElect["ampB"] = round2(ampB);
        data_pElect["ampC"] = round2(ampC);
        data_pElect["actPowA"] = actPowA;
        data_pElect["actPowB"] = actPowB;
        data_pElect["actPowC"] = actPowC;
        data_pElect["reactPowA"] = reactPowA;
        data_pElect["reactPowB"] = reactPowB;
        data_pElect["reactPowC"] = reactPowC;
        data_pElect["powFA"] = round2(powFA);
        data_pElect["powFB"] = round2(powFB);
        data_pElect["powFC"] = round2(powFC);
        data_pElect["freqz"] = round2(freqz);
        data_pElect["totP"] = round2(totP);
        data_pElect["totQ"] = round2(totQ);

        JsonArray EVENTS = doc.createNestedArray("events");
    
        for(int i = 0; i < eventsCounter; i++)
        {
           JsonObject obj = EVENTS.createNestedObject();
           obj["ts"] = eventsFIFO[i].ts;
           obj["cod"] = eventsFIFO[i].cod;
        }

        clearEventsArray();
        clearPElect();
        
        String buf;
        doc.shrinkToFit();  // optional
        size_t n = serializeJson(doc, buf);
      
        Serial.print(n);
        Serial.print("/");
        Serial.println(buf);
        
        byte nTries = 0;
      
        while((!sendOk) and (nTries < 3))
        { 
          if (mqtt.publish(topicPiedraNegra, buf.c_str(),n)) 
          {
             Serial.println("Success sending message");
             sendOk = true;
          } 
          else 
          {
             Serial.println("Error sending message");
             nTries++;
             delay(1000);
          }
        }
      
        delay(500);
        disconnectMqtt();
      }
    }
  }
}

void sendMsgAlarma(bool st)
{
  bool sendOk = false;
  int rssi = 0;
  
  if(modemOK)
  {
    int csq = modem.getSignalQuality();
    rssi = (56*(csq - 2) - 3052)/28;

    if(!getGPS())
       Serial.println("Error getting GPS data");
    else
    {
       Serial.print("latitude:"); Serial.print(latitude,5);
       Serial.print(", longitude:"); Serial.println(longitude,5);
    }
    
    if(checkNetwork())
    {
      // Conectar a broker MQTT
      bool procOk = connectMqtt();
    
      if(procOk)
      {
        String alarmaStr = "";
        if(!st)
          alarmaStr = "Corte energia";
        else
          alarmaStr = "Reposicion energia";
        
        String IMEI = modem.getIMEI();
        String ICCID = modem.getSimCCID();
        DynamicJsonDocument doc(386);    

        doc["tipo"] = "Alarma";
        doc["id"] = modID;
        doc["ts"] = getEpochTime();
        doc["rssi"] = rssi;
        doc["IMEI"] = IMEI;
        doc["ICCID"] = ICCID;

        JsonObject GPS = doc["GPS"].to<JsonObject>();
        GPS["lat"] = latitude;
        GPS["lon"] = longitude;

        JsonObject data = doc["data"].to<JsonObject>();
        doc["data"]["alarma"] = alarmaStr;
        
        String buf;
        doc.shrinkToFit();  // optional
        size_t n = serializeJson(doc, buf);
      
        Serial.print(n);
        Serial.print("/");
        Serial.println(buf);
        
        byte nTries = 0;
      
        while((!sendOk) and (nTries < 3))
        { 
          if (mqtt.publish(topicPiedraNegra, buf.c_str(),n)) 
          {
             Serial.println("Success sending message");
             sendOk = true;
          } 
          else 
          {
             Serial.println("Error sending message");
             nTries++;
             delay(1000);
          }
        }
      
        delay(500);
        disconnectMqtt();
      }
    }
  }  
}

void showData()
{
  Serial.println("-------------------------");
  Serial.print("voltA: "); Serial.println(voltA);
  Serial.print("voltB: "); Serial.println(voltB);
  Serial.print("voltC: "); Serial.println(voltC);
  Serial.print("ampA: "); Serial.println(ampA);
  Serial.print("ampB: "); Serial.println(ampB);
  Serial.print("ampC: "); Serial.println(ampC);
  Serial.print("actPowA: "); Serial.println(actPowA);
  Serial.print("actPowB: "); Serial.println(actPowB);
  Serial.print("actPowC: "); Serial.println(actPowC);
  Serial.print("reactPowA: "); Serial.println(reactPowA);
  Serial.print("reactPowB: "); Serial.println(reactPowB);
  Serial.print("reactPowC: "); Serial.println(reactPowC);
  Serial.print("powFA: "); Serial.println(powFA);
  Serial.print("powFB: "); Serial.println(powFB);
  Serial.print("powFC: "); Serial.println(powFC);
  Serial.print("freqz: "); Serial.println(freqz);
  Serial.print("totP: "); Serial.println(totP);
  Serial.print("totQ: "); Serial.println(totQ);
  Serial.println("-------------------------");  
}

void enableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) 
    {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();
}

void disableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
    modem.disableGPS();
}

bool getGPS()
{
  bool gpsOk;
//  gpsOk = modem.getGPS(&latitude, &longitude);
//  if(gpsOk)
//  {
//
//    Serial.print("latitude:"); Serial.print(latitude,5);
//    Serial.print(", longitude:"); Serial.println(longitude,5);
//      
//  }
//  else
//  {
//    Serial.println("Error getting GPS data");
//    //latitude = 0.0;
//    //longitude = 0.0;
//  }

  byte tries = 0;
  while(!gpsOk and tries < 5)
  {
    gpsOk = modem.getGPS(&latitude, &longitude);
    if(!gpsOk)
    {
      delay(1500);
      tries++;
    }
  }
  
  return gpsOk;
}

void insertNewEvent(Event& ev)
{
  if(eventsCounter < ARRAYSIZE)
  {
    eventsFIFO[eventsCounter] = ev;
    eventsCounter++;
  }
}

void clearEventsArray()
{
    //Borrar array de eventos
  for(int i = 0; i < ARRAYSIZE; i++)
  {
    eventsFIFO[i].ts = 0;
    eventsFIFO[i].cod = 0;
  }

  eventsCounter = 0;
}

void clearPElect()
{
  voltA = 0.0;
  voltB = 0.0;
  voltC = 0.0;
  ampA =  0.0;
  ampB =  0.0;
  ampC =  0.0;
  actPowA = 0;
  actPowB = 0;
  actPowC = 0;
  reactPowA = 0;
  reactPowB = 0;
  reactPowC = 0;
  powFA = 0.0;
  powFB = 0.0;
  powFC = 0.0;
  freqz = 0.0; 
}
