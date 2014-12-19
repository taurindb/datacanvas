/***************************************************************** 
Upload data to localdata servers from Seeeduino 

Development environment specifics:
    IDE: Arduino 1.5.6-r2
    Hardware Platform: Seeeduino
*****************************************************************/
// Process.h gives us access to the Process class, which can be
// used to construct Shell commands and read the response.
#include <avr/pgmspace.h>
#include <Process.h>

#include <DHT.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>

/* user configuration */
#define PRIVATE_KEY         "ci3mv36vi0001ep0uyo5kcjbx"
#define LATLNG              "[37.7902370,-122.2300810]"    // http://mygeoposition.com/


/**/
#define pin_uv              A2      // UV  sensor
#define pin_sound           A0      // Sound sensor
#define pin_dust            7       // Dust sensor
#define pin_air_quality     A1      // Air quality sensor
#define pin_dht             4       // Humidity and temperature sensor
#define DHTTYPE             DHT22   // DHT 22  (AM2302)

#define FP(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))

/* sample function */
float iReadTemperature(void);
float iReadHumidity(void);
float iReadDensityDust(void);
unsigned long iReadLux(void);
float iReadUVRawVol(void);
int iReadSoundRawVol(void);

/* dust variables */
unsigned long dust_starttime;
unsigned long duration_starttime;
unsigned long duration;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = -1;
int sensorValue=0;

/* temperature and humidity sensor */
DHT dht(pin_dht, DHTTYPE);

/* sound sensor */
uint16_t snd_raw_max = 0;
uint32_t snd_sum_100ms = 0;
uint16_t cntr_snd = 0;
uint16_t snd_last_avg = 0;

/* Air Quality */
enum{AQ_INIT, AQ_WAIT_INIT, AQ_WORK};
enum{AQ_HIGH_POLLUTION, AQ_POLLUTION, AQ_LOW_POLLUTION, AQ_FRESH, AQ_WARMUP};
unsigned long air_quality_sensor_init_starttime;
unsigned long air_quality_sensor_init_time = 200000;
unsigned long air_quality_sensor_evaluate_starttime;
int air_quality_sensor_state = AQ_WAIT_INIT;
int aq_first_vol, aq_last_vol, aq_std_vol, aq_std_vol_sum;
static int cntr_aq_avg = 0;
int aq_result = AQ_WARMUP;

/* Digital Light sensor */
int32_t lux_last_valid = 0;

/* Global varibles */
unsigned long push_starttime;
unsigned long push_interval = 10000;  //ms

/////////////////
// CURL Request //
/////////////////
const char curlStart[] PROGMEM = "curl -X POST -k -H \"Content-Type: application/json\" -d '{";
const char curlClose[] PROGMEM = " }'  https://localdata-sensors.herokuapp.com/api/sources/<KEY>/entries"; 
const char latlng[] PROGMEM = LATLNG; 
const char userKey[] PROGMEM = PRIVATE_KEY;

//https://localdata-sensors.herokuapp.com/api/sources/ci3mv36vi0001ep0uyo5kcjbx/entries?startIndex=0&count=1000000

// How many data fields are in your stream?
const int NUM_FIELDS = 10;
// What are the names of your fields?
const char field0[] PROGMEM = "timestamp";
const char field1[] PROGMEM = "location";
const char field2[] PROGMEM = "airquality";
const char field2_1[] PROGMEM = "airquality_raw";
const char field3[] PROGMEM = "dust";
const char field4[] PROGMEM = "humidity";
const char field5[] PROGMEM = "light";
const char field6[] PROGMEM = "sound";
const char field7[] PROGMEM = "temperature";
const char field8[] PROGMEM = "uv";
const char* fieldName[NUM_FIELDS] = {field0, field1, field2, field2_1, field3, field4, field5, field6, field7, field8};
// We'll use this array later to store our field data
String fieldData[NUM_FIELDS];

Process postProcess; // Used to send command to Shell, and view response

void setup() 
{
  Bridge.begin();
  Serial.begin(115200);
  delay(2000); // Wait for usb serial port to initialize

  /* Wire Begin */
  Wire.begin();

  Serial.println(F("******DataCanvasSensorNode******\r\n"));
  
   /* Initialize temperature and humidity sensor */
  Serial.println(F("Initialize sensors...\r\n"));
  dht.begin();

  /* Initialize Dust sensor */
  digitalWrite(pin_dust, HIGH);
  pinMode(pin_dust,INPUT);
  attachInterrupt(get_interrupt_num(pin_dust), dust_interrupt, CHANGE);

  /* Digital Light Sensor */
  TSL2561.init();

  /* Sound Sensor */
  pinMode(pin_sound, INPUT);

  /* UV Sensor */
  pinMode(pin_uv, INPUT);

  /* Air Quality */
  pinMode(pin_air_quality, INPUT);
  air_quality_sensor_init_starttime = millis();

  /* other */
  push_starttime = 0;
  
  /* check the internet connection is established */
  checkInternet();
  
  /* Sync clock with NTP */
  setClock();
  Serial.println(F("Done."));
  Serial.println(F("=========== Ready to Stream ==========="));
}

void loop()
{
  //fast loops the sound sampling  
  delay(100); //wait the power source to calm down after the wifi module transfered internet packets
              //or the pulse in vcc will affect the measuring of sound sensor
  uint32_t curtime = millis();
  cntr_snd = 0;
  snd_sum_100ms = 0;
  while(millis() - curtime < 100)
  {
    int smax = 0;
    for (int i =0;i<10;i++)  //get the max value in 10 samples
    {
      int s = analogRead(pin_sound);
      if (s > smax) smax = s;
      delayMicroseconds(10);
    }
    snd_sum_100ms += smax;
    cntr_snd++;
  }
  snd_last_avg = snd_sum_100ms / cntr_snd;
  //Serial.print(F("sound:"));
  //Serial.println(snd_last_avg);
  if (snd_last_avg > snd_raw_max) snd_raw_max = snd_last_avg;
  

  //fast loops the state machine for air quality driver
  if (air_quality_sensor_state != AQ_WORK) air_quality_state_machine();
  
  //wait to evaluate air quality sensor's outputs - interval 2s
  if (air_quality_sensor_state == AQ_WORK && (millis() - air_quality_sensor_evaluate_starttime) > 2000)
  {
    air_quality_sensor_evaluation();
    cntr_snd = 0;
    snd_raw_max = 0;
  }
  
  //wait to post data - slow
  if((millis() - push_starttime) > push_interval) //maybe increase posting interval to 2x per minute?
  {
    push_starttime = millis();
    fieldData[0] = String(timeInEpoch())+F("000");         //Timestamp is multipied by 1000 for UNIX time 
    fieldData[1] = String(FP(latlng));                     // [lat, lng]
    switch (aq_result)
    {
      case AQ_WARMUP:
        fieldData[2] = String(F("\"WarmUp\"")); break;
      case AQ_FRESH:
        fieldData[2] = String(F("\"Fresh\"")); break;
      case AQ_LOW_POLLUTION:
        fieldData[2] = String(F("\"LowPollution\"")); break;
      case AQ_POLLUTION:
        fieldData[2] = String(F("\"Pollution\"")); break;
      case AQ_HIGH_POLLUTION:
        fieldData[2] = String(F("\"HighPollution\"")); break;
    }
    fieldData[3] = String(analogRead(pin_air_quality));  // ~0ms
    fieldData[4] = String(iReadDensityDust());          // ~0ms, pcs/0.01cf or pcs/283ml
    fieldData[5] = String(iReadHumidity());            // >250ms, %
    fieldData[6] = String(iReadLux());                // >100ms , lux
    fieldData[7] = String(iReadSoundRawVol());       // ~0ms, mV;
    fieldData[8] = String(iReadTemperature());      // >250ms, F
    fieldData[9] = String(iReadUVRawVol());        // > 128ms, mV

    // Post Data
    Serial.println(F("\nPosting Data!"));
    postData(); // the postData() function does all the work,see below.
    cntr_snd = 0;
    snd_raw_max = 0;
  }
  
  //read response from the post process
  while (postProcess.available() > 0)
  {
    char c = postProcess.read();
    Serial.print(c);
  }
  Serial.flush();
}

int get_interrupt_num(int pin)
{
  switch(pin)
  {
    case 3:{ return 0;}
    case 2:{ return 1;}
    case 0:{ return 2;}
    case 1:{ return 3;}
    case 7:{ return 4;}
    default:{Serial.println(F("invalid pin assigned for dust sensor")); return 0;} 
  }
}

/* Interrupt service */
void dust_interrupt()
{
  unsigned long cur_micros = micros();
  if (digitalRead(pin_dust) == 0)  //fall
  {
    duration_starttime = cur_micros;
  }else
  {
    duration = cur_micros - duration_starttime;
    if(duration > 10000 && duration < 90000) lowpulseoccupancy += duration;
    
    unsigned long cur_millis = millis();

    if ((cur_millis - dust_starttime) > sampletime_ms)
    {
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      lowpulseoccupancy = 0;
      dust_starttime = cur_millis;
    }
  }
}

void air_quality_state_machine()
{
  switch (air_quality_sensor_state)
  {
    case AQ_WAIT_INIT:
      {
        if (millis() - air_quality_sensor_init_starttime > air_quality_sensor_init_time)
        {
          air_quality_sensor_state = AQ_INIT;
        }
        break;
      }
    case AQ_INIT:
      {
        int v = analogRead(pin_air_quality);
        if (v < 798 && v > 10)  //the init voltage is ok
        {
          //Serial.print("init:");
          //Serial.println(v);

          aq_first_vol = v;
          aq_last_vol = v;
          aq_std_vol = v;
          aq_std_vol_sum = 0;
          air_quality_sensor_state = AQ_WORK;
        }
        else
        {
          air_quality_sensor_init_starttime = millis();
          air_quality_sensor_state = AQ_WAIT_INIT;
        }
        break;
      }
    case AQ_WORK:
      {
        break;
      }
    default:
      break;
  }
}

void air_quality_sensor_window_avg()
{
  if (++cntr_aq_avg >= 150)  //sum for 5 minutes
  {
    cntr_aq_avg = 0;
    aq_std_vol = aq_std_vol_sum / 150;
    aq_std_vol_sum = 0;
  } else
  {
    aq_std_vol_sum += aq_first_vol;
  }
}

void air_quality_sensor_evaluation()
{
  aq_last_vol = aq_first_vol;
  aq_first_vol = analogRead(pin_air_quality);

  //Serial.println(aq_std_vol);
  //Serial.println(aq_first_vol);

  if (aq_first_vol - aq_last_vol > 200 || aq_first_vol > 350)
  {
    aq_result = AQ_HIGH_POLLUTION;
  } else if ((aq_first_vol - aq_last_vol > 200 && aq_first_vol < 350) || aq_first_vol - aq_std_vol > 75)
  {
    aq_result = AQ_POLLUTION;
  } else if ((aq_first_vol - aq_last_vol > 100 && aq_first_vol < 350) || aq_first_vol - aq_std_vol > 25)
  {
    aq_result = AQ_LOW_POLLUTION;
  } else
  {
    aq_result = AQ_FRESH;
  }
  air_quality_sensor_window_avg();
}

//*****************************************************************************
//! \brief Read temperature
//! cost time: > 250ms
//! \return  temperature
//! \refer to http://www.seeedstudio.com/wiki/File:Humidity_Temperature_Sensor_pro.zip
//*****************************************************************************
float iReadTemperature(void) {
    float temper;
    temper = dht.readTemperature(true); //true: get F, false: get oC
    return temper;
}

//*****************************************************************************
//! \brief Read Humidity
//! cost time: > 250ms
//! \return  Humidity
//*****************************************************************************
float iReadHumidity(void) {
    float humidity;
    humidity = dht.readHumidity();
    return humidity;
}

//*****************************************************************************
//! \brief Read Dust Density
//! cost time: ~0ms , unit: pcs/0.01cf or pcs/283ml
//! \return  temperature
//*****************************************************************************
float iReadDensityDust(void) {
    return concentration;
}

//*****************************************************************************
//! \brief Read Lux value of visible light
//! \return  Luminance
//*****************************************************************************
unsigned long iReadLux(void) {
    //cost time: > 100ms
    int32_t lux0 = TSL2561.readVisibleLux();
    if(lux0 != -1)
    {
      lux_last_valid = lux0;
    }
    return lux_last_valid;
}
//*****************************************************************************
//! \brief Read the raw voltage signal of UV sensor
//! \return  voltage mV
//*****************************************************************************
float iReadUVRawVol(void) {
    unsigned long sum=0;
    for(int i=0; i<128; i++)
    {
      sum += analogRead(pin_uv);
      delay(1);
    }
    sum >>= 7;
    return sum*(4980.0f/1023.0f);
}

//*****************************************************************************
//! \brief Read the raw voltage signal of sound sensor
//! \return  voltage mV
//*****************************************************************************
int iReadSoundRawVol() {
  uint16_t snd = snd_raw_max* (int)(4980.0f / 1023.0f);
  snd_raw_max = 0;
  return snd;
  //return snd_last_avg * (int)(4980.0f / 1023.0f);
}

void postData()
{
  String curlCmd; // Where we'll put our curl command

  // Construct the curl command:
  curlCmd = FP(curlStart);
  int i = 0;
  for (i=0; i<(NUM_FIELDS-1); i++) {
    curlCmd += String("\"") + FP((PGM_P)fieldName[i]) + "\": " + fieldData[i] + ", "; // Add our data fields to the command
  }
  curlCmd += String("\"") + FP((PGM_P)fieldName[NUM_FIELDS-1]) + "\": " + fieldData[NUM_FIELDS-1];
  curlCmd += FP(curlClose); // Add the server URL, including public key
  curlCmd.replace(String(F("<KEY>")), String(FP(userKey)) );
  
  // Send the curl command:
  Serial.print(F("Sending command: "));
  Serial.println(curlCmd); // Print command for debug
  postProcess.runShellCommandAsynchronously(curlCmd); // Send command through Shell
  
  // Read out the response:
  //Serial.print(F("Response: "));
  // Use the process to check for any response
  
}

/////////////////////////////////////////////////////////////////////
// Make sure the internet connection is established
void checkInternet()
{
  Process p;
  
  while(true)
  {
    p.runShellCommand(F("ping -q -w 1 -c 1 0.openwrt.pool.ntp.org > /dev/null && echo ok || echo error"));
    if(p.find("ok")) break;
    else
    {
      Serial.println(F("Internet Connection is not established"));
    }
  }
}

/////////////////////////////////////////////////////////////////////
// Synchronize clock using NTP
void setClock() {  
  Process p;
  Serial.println(F("Setting clock."));
  // Sync clock with NTP
  while(true)
  {
    uint32_t t = millis();
    p.runShellCommandAsynchronously(F("ntpd -nqp 0.openwrt.pool.ntp.org"));
    // Block until clock sync is completed
    while(p.running() && millis() - t < 10000);
    if(p.running()) { p.close();continue;}
    else break;
  }
  //light up the led
  digitalWrite(13, HIGH);
}

/////////////////////////////////////////////////////////////////////
// Return timestamp of Linino
unsigned long timeInEpoch() {
  Process time;                   // process to run on Linuino
  char epochCharArray[12] = "";   // char array to be used for atol

  // Get UNIX timestamp
  time.begin(F("date"));
  time.addParameter(F("+%s"));
  time.run();
  
  // When execution is completed, store in charArray
  while (time.available() > 0) {
    //millisAtEpoch = millis();
    time.readString().toCharArray(epochCharArray, 12);
  }
  
  // Return long with timestamp
  return atol(epochCharArray);
}
