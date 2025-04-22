#include "LoRa_E220.h"
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>
#include <ModbusMaster.h>

#define TIMERULES   0

#define ID          3
#define FREQUENCY_915
#define ENABLE_RSSI true
#define NETOK       13
#define ONOFF       14
#define CONTACT     15
#define START       32   
#define STOP        33
#define SELECTOR    23
#define RXModbus    18   // Agregar conexión física en placa
#define TXModbus    19   // Agregar conexión física en placa
#define NUMREADS    3
#define DESTINATION_ADDL 2
#define CHANNEL     65
#define TXID        1

#define WDT_TIMEOUT 30      // WDT Timeout in seconds
#define SELECTTIME  1000
#define NETGUARDTIME 300000    // Limite tiempo sin recepcion de packets desde Tx  (5 min)
#define PMTIME     50000
#define MINLEVEL   2000
#define MAXLEVEL   2750
#define STARTTIME   18
#define ENDTIME     23 

#define VOLTA        0x00
#define VOLTB        0x01
#define VOLTC        0x02
#define AMPA         0x03
#define AMPB         0x04
#define AMPC         0x05
#define ACTPOWA      0x08
#define ACTPOWB      0x09
#define ACTPOWC      0x0A
#define REACTPOWA    0x0C
#define REACTPOWB    0x0D
#define REACTPOWC    0x0E
#define POWFA        0x14
#define POWFB        0x15
#define POWFC        0x16
#define FREQ         0x1A
#define TOTP         0x1D     // Registro Doble
#define TOTQ         0x3B     // Registro Doble

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

LoRa_E220 e220ttl(&Serial2, 27, 25, 26); //  RX AUX M0 M1   9600 bps

RTC_DS1307 rtc;
DateTime now;
//DateTime startTime = DateTime(2025,1,1, STARTTIME, 0, 0);
//DateTime endTime = DateTime(2025,1,1, ENDTIME, 0, 0);

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

enum st
{
  ON,
  OFF
};

enum md
{
  AUTO,
  MANUAL
};

st state;
md modo;

unsigned long selectTimer;
unsigned long netCheckTimer;
unsigned long pmTimer;

bool netOk;
bool rtcOk;

unsigned int nivel;
int errorCounter;

char daysOfWeek[7][12] = {
  "Domingo",
  "Lunes",
  "Martes",
  "Miercoles",
  "Jueves",
  "Viernes",
  "Sabado"
};

ModbusMaster node;

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
uint32_t totP;
uint32_t totQ;

void setup() 
{
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1, RXModbus, TXModbus);    // Puerto lectura Modbus RTU

  node.begin(1, Serial1);

  pinMode(NETOK, OUTPUT);
  pinMode(ONOFF, OUTPUT);
  pinMode(CONTACT, OUTPUT);
  
  pinMode(START, INPUT_PULLUP);
  pinMode(STOP, INPUT_PULLUP);
  pinMode(SELECTOR, INPUT_PULLUP);

  modo = AUTO;
  state = OFF;

  netOk = false;
  rtcOk = false;
  
  digitalWrite(NETOK, HIGH);     // Lógica inversa
  digitalWrite(ONOFF, HIGH);
  digitalWrite(CONTACT, HIGH);
  
  e220ttl.begin();

  initLoRa();

  // SETUP RTC MODULE
  if (! rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
  }
  else
  {
    rtcOk = true;
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("Error en inicializacion de display OLED");
  }
 //display.setContrast (0); // dim display
 
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Satori SpA ");
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Water Pump Control V2");
  display.setCursor(0, 50);
  display.print("Init OK...");
  display.display();

  Serial.println("Inicializacion rxPucon V2 OK");

  selectTimer = millis();
  netCheckTimer = millis();
  pmTimer = millis();

  nivel = 2500;

  esp_task_wdt_init(WDT_TIMEOUT, true); // Initialize ESP32 Task WDT
  esp_task_wdt_add(NULL);               // Subscribe to the Task WDT
}

void loop() 
{
  checkNetwork();
  
  if((millis() - selectTimer) > SELECTTIME)
  {
    checkSelector();
    now = rtc.now();
    selectTimer = millis();
  }

  if((millis() - pmTimer) > PMTIME)   // Leer valores desde Power Meter
  {
    pmTimer = millis();
    readAllValues();
    //showValues();
    // Enviar valores por enlace LoRa a repetidor
    uint16_t totPH = (totP & 0xffff0000) >> 16;
    uint16_t totPL = totP & 0xffff;
    uint16_t totQH = (totQ & 0xffff0000) >> 16;
    uint16_t totQL = totQ & 0xffff;
    
    //Enviar datos de power meter a repetidor
    struct msgElect m = {"ELECT", ID, voltA, voltB, voltC, ampA, ampB, ampC, actPowA, actPowB, actPowC, reactPowA, reactPowB, reactPowC, powFA, powFB, powFC, freqz, totPH, totPL, totQH, totQL};
    e220ttl.sendFixedMessage(0, DESTINATION_ADDL,CHANNEL, &m,sizeof(msgElect));
  }
  
  if(modo == MANUAL)
    checkButtons();

  if((millis() - netCheckTimer) > NETGUARDTIME)     // Chequear si ya no se reciben mensajes desde el transmisor (max. 5 min)
  {
     netOk = false;
     digitalWrite(NETOK, HIGH); 
     
     if(modo == AUTO)
     {
        if(state == ON)
            setOff();
     }
     
     //netCheckTimer = millis();
  }
  
  esp_task_wdt_reset();
}

void setOn()
{
  digitalWrite(ONOFF, LOW);
  digitalWrite(CONTACT, LOW);
  state = ON;

  byte mod;
  if(modo == AUTO)
     mod = 0;
  else
     mod = 1;
  
  struct msgOnOff m = {"ONOFF", ID, 1, mod};
  e220ttl.sendFixedMessage(0, DESTINATION_ADDL,CHANNEL, &m,sizeof(msgOnOff));  
}

void setOff()
{
  digitalWrite(ONOFF, HIGH);
  digitalWrite(CONTACT, HIGH);
  state = OFF;

  byte mod;
  if(modo == AUTO)
      mod = 0;
  else
      mod = 1;
  
  struct msgOnOff m = {"ONOFF", ID, 0, mod};
  e220ttl.sendFixedMessage(0, DESTINATION_ADDL,CHANNEL, &m,sizeof(msgOnOff));   
}

void checkNetwork()
{
  if (e220ttl.available()>1) 
  {
    Serial.println("Message received!");
    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(msgNivel));
    
    if (rc.status.code!=1)
    {
        Serial.println(rc.status.getResponseDescription());
    }
    
    else  
    {
      // Print the data received
      struct msgNivel m1 = *(msgNivel*) rc.data;
      //Serial.println(rc.status.getResponseDescription());
      //Serial.println(rc.data);
      //Serial.print("RSSI: "); Serial.println(rc.rssi, DEC);

      if(m1.id == TXID)   // Chequear que mensaje viene desde transmisor ID = 1
      {
        // Enviar ack a transmisor
        Serial.println("Enviando ack");
  
        nivel = m1.nivel;
  
        byte onOff;
        if(state == ON)
           onOff = 1;
        else
           onOff = 0;

        byte mod;
        if(modo == AUTO)
            mod = 0;
        else
            mod = 1;
   
        struct msgAck m2 = {"ACKNG", ID, m1.id, m1.counter, onOff, mod};
        e220ttl.sendFixedMessage(0, DESTINATION_ADDL,CHANNEL, &m2,sizeof(msgAck));
  
        showNivel(nivel);
        showDatetime(now);
      }    

      if(modo == AUTO)
      {
        // Actuar en base a valor recibido si modo es AUTO
        analizarValue(nivel);
      }
  
      netOk = true;
      digitalWrite(NETOK, LOW);    // Led netOk encendido
      netCheckTimer = millis();
    }

    rc.close();
  }
}

void checkButtons()
{ 
  if(digitalRead(START) == LOW)     // Boton start (verde) directo NO
  {
    delay(50);
    if(digitalRead(START) == LOW)
    {
      if(state == OFF)
      {
        // Prender bomba
        Serial.println("Activando bomba");
        setOn();
      }
    }
  }
  
  else if(digitalRead(STOP) == HIGH)   // Boton stop (rojo) inverso NC
  {
    delay(50);
    if(digitalRead(STOP) == HIGH)
    {
      if(state == ON)
      {
        // Apagar bomba
        Serial.println("Desactivando bomba");
        setOff();
      }
    }
  }
}

void checkSelector()
{
  if(digitalRead(SELECTOR) == HIGH)
  {
    delay(50);
    if(digitalRead(SELECTOR) == HIGH)
    {
      if(modo == MANUAL)
      {
        Serial.println("Cambio a modo AUTO");
        if(state == ON)
            setOff();
      }
      modo = AUTO;
    }
  }
  
  else if(digitalRead(SELECTOR) == LOW)
  {
    delay(50);
    if(digitalRead(SELECTOR) == LOW)
    {
      if(modo == AUTO)
      {
        Serial.println("Cambio a modo MANUAL");
        if(state == ON)
            setOff();
      }    
      modo = MANUAL;
    }
  }
}

void analizarValue(unsigned int val)
{
//  if(TIMERULES)
//  {
//    if((now >= startTime.unixtime()) and (now <= endTime.unixtime()))
//    {
//      Serial.println("Horario punta");
//    }
//    else
//    {
//      if(val <= MINLEVEL)
//      {
//        setOn();
//      }
//      
//      else if(val >= MAXLEVEL)
//      {
//        setOff();
//      }     
//    }
//  }
  if(val <= MINLEVEL)
  {
    if(state == OFF)
    {
        // Prender bomba
        Serial.println("Activando bomba");
        setOn();
    }
  }
  
  else if(val >= MAXLEVEL)
  {
    if(state == ON)
    {
        // Apagar bomba
        Serial.println("Desactivando bomba");
        setOff();
    }
  }
}

void showNivel(unsigned int n)
{
   display.setTextSize(2);
   display.clearDisplay();
   display.setCursor(0, 0);
   display.print("Nivel: ");
   display.setCursor(25, 25);
   display.print(n);
   display.print(" mm");
   display.display();
}

void showDatetime(DateTime dt)
{
  Serial.print("RTC Date Time: ");
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(" (");
  Serial.print(daysOfWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  String str;

  str = String(now.day()) + "/" + String(now.month()) + "/" + String(now.year())+ " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()); 

  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(str);
  display.display();
}

int leerRegistro(uint16_t reg)
{
  uint8_t result;
  int16_t val = 0;

  result = node.readHoldingRegisters(reg,1);  //Leer 1 registros de 16 bits desde address reg
  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
     val = node.getResponseBuffer(0); 
  }

  else
  {
    Serial.println("Error en lectura Modbus");
  }

  return val;
}

long leerRegistroDoble(uint16_t reg)
{
  uint8_t result;
  int32_t val = 0;

  result = node.readHoldingRegisters(reg,2);  //Leer 2 registros de 16 bits desde address reg
  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
     val = (node.getResponseBuffer(0) << 16) + node.getResponseBuffer(1); 
  }

  else
  {
    Serial.println("Error en lectura Modbus");
  }

  return val; 
}

void readAllValues()
{
   voltA = leerRegistro(VOLTA);
   voltB = leerRegistro(VOLTB);
   voltC = leerRegistro(VOLTC);
   ampA =  leerRegistro(AMPA);
   ampB =  leerRegistro(AMPB);
   ampC =  leerRegistro(AMPC);
   actPowA = leerRegistro(ACTPOWA);
   actPowB = leerRegistro(ACTPOWB);
   actPowC = leerRegistro(ACTPOWC);
   reactPowA = leerRegistro(REACTPOWA);
   reactPowB = leerRegistro(REACTPOWB);
   reactPowC = leerRegistro(REACTPOWC);
   powFA = leerRegistro(POWFA);
   powFB = leerRegistro(POWFB);
   powFC = leerRegistro(POWFC);
   freqz = leerRegistro(FREQ);

   totP = leerRegistroDoble(TOTP);     // En WH
   totQ = leerRegistroDoble(TOTQ);     // En VAH
}


void showValues()
{
  Serial.println("***************************************");
  Serial.print("voltA: ");
  Serial.println((float)voltA/10.0);
  Serial.print("voltB: ");
  Serial.println((float)voltB/10.0);
  Serial.print("voltC: ");
  Serial.println((float)voltC/10.0);
  Serial.print("ampA: ");
  Serial.println((float)ampA/100.0);
  Serial.print("ampB: ");
  Serial.println((float)ampB/100.0);
  Serial.print("ampC: ");
  Serial.println((float)ampC/100.0);  
  Serial.print("actPowA: ");
  Serial.println(actPowA);
  Serial.print("actPowB: ");
  Serial.println(actPowB);
  Serial.print("actPowC: ");
  Serial.println(actPowC);  
  Serial.print("reactPowA: ");
  Serial.println(reactPowA);
  Serial.print("reactPowB: ");
  Serial.println(reactPowB);
  Serial.print("reactPowC: ");
  Serial.println(reactPowC); 
  Serial.print("powFA: ");
  Serial.println((float)powFA/1000.0);
  Serial.print("powFB: ");
  Serial.println((float)powFB/1000.0);
  Serial.print("powFC: ");
  Serial.println((float)powFC/1000.0);
  Serial.print("freqz: ");
  Serial.println((float)freqz/100.0);
  Serial.print("TotalActPower(WH): ");
  Serial.println(totP);
  Serial.print("TotalReactPower(VAH): ");
  Serial.println(totQ);
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
