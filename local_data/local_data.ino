 /***************************************************************** 
Upload data to localdata servers from Seeeduino 

Development environment specifics:
    IDE: Arduino 1.5.6-r2
    Hardware Platform: Seeeduino
*****************************************************************/
// Process.h gives us access to the Process class, which can be
// used to construct Shell commands and read the response.
#include <Process.h>

#include <DHT.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>

#define pin_uv              A2      // UV  sensor
#define pin_sound           A0      // Sound sensor
#define pin_dust            7       // Dust sensor
#define pin_air_quality     A1      // Air quality sensor
#define pin_dht             4       // Humidity and temperature sensor
#define DHTTYPE             DHT22   // DHT 22  (AM2302)

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
int snd_raw_max = 0;
uint32_t snd_sum_100ms = 0;
uint16_t cntr_snd = 0;
uint16_t snd_last_avg = 0;

/* Air Quality */
enum{AQ_INIT, AQ_WAIT_INIT, AQ_WORK};
enum{AQ_HIGH_POLLUTION, AQ_POLLUTION, AQ_LOW_POLLUTION, AQ_FRESH, AQ_WARMUP};
unsigned long air_quality_sensor_init_starttime;
int air_quality_sensor_state = AQ_WAIT_INIT;
unsigned long air_quality_sensor_init_time = 200000;
int aq_first_vol, aq_last_vol, aq_std_vol, aq_std_vol_sum;
static int cntr_aq_avg = 0;
int aq_result = AQ_WARMUP;

/* Digital Light sensor */
int32_t lux_last_valid = 0;

/* Global varibles */
boolean valid_dust = false;
unsigned long push_starttime;
unsigned long push_interval = 10000;  //ms

/////////////////
// CURL Request //
/////////////////
const String curlStart = "curl -X POST -k -H \"Content-Type: application/json\" -d '{";
const String curlClose = " }'  https://localdata-sensors.herokuapp.com/api/sources/"; //TODO: Change key to user variable
String latlng = "[37.7902370,-122.2300810]"; // http://mygeoposition.com/
const String userKey = "ci3mv36vi0001ep0uyo5kcjbx";
const String entries = "/entries";

//https://localdata-sensors.herokuapp.com/api/sources/ci3mv36vi0001ep0uyo5kcjbx/entries?startIndex=0&count=1000000

// How many data fields are in your stream?
const int NUM_FIELDS = 9;
// What are the names of your fields?
String fieldName[NUM_FIELDS] = {"timestamp", "location", "airquality", "dust", "humidity", "light", "sound", "temperature", "uv"};
// We'll use this array later to store our field data
String fieldData[NUM_FIELDS];

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
  attachInterrupt(0, dust_interrupt, CHANGE);

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
  //init_timer1(200);  //us, get the value based on Shannon's law, sound freq: 0~3400Hz
  
  // Sync clock with NTP
  setClock();
  Serial.println(F("Done."));
  Serial.println(F("=========== Ready to Stream ==========="));
}

void loop()
{   
  int snd_raw = analogRead(pin_sound);
  if (snd_raw > snd_raw_max) snd_raw_max = snd_raw;
  if((++cntr_snd)%5 == 0)
  {
    snd_sum_100ms += snd_raw_max;
    snd_raw_max = 0;
  }
  
  if (cntr_snd >= 5000)
  {
    cntr_snd = 0;
    uint16_t snd_this_avg = snd_sum_100ms/1000;
    snd_sum_100ms = 0;
    if(snd_last_avg == 0)
      snd_last_avg = snd_this_avg;
    else
      snd_last_avg = (snd_last_avg + snd_this_avg) / 2;
  }
  
  if((millis() - push_starttime) > push_interval) //maybe increase posting interval to 2x per minute?
  {
    push_starttime = millis();
    
    fieldData[0] = String(timeInEpoch())+F("000");         //Timestamp is multipied by 1000 for UNIX time 
    fieldData[1] = String(latlng);                       // [lat, lng]
    fieldData[2] = String(analogRead(pin_air_quality));  // ~0ms
    fieldData[3] = String(iReadDensityDust());          // ~0ms, pcs/0.01cf or pcs/283ml
    fieldData[4] = String(iReadHumidity());            // >250ms, %
    fieldData[5] = String(iReadLux());                // >100ms , lux
    fieldData[6] = String(iReadSoundRawVol());       // ~0ms, mV;
    fieldData[7] = String(iReadTemperature());      // >250ms, F
    fieldData[8] = String(iReadUVRawVol());        // > 128ms, mV

    // Post Data
    Serial.println(F("Posting Data!"));
    postData(); // the postData() function does all the work,see below.
    
    cntr_snd = 0;
    snd_raw_max = 0;
  }
}

/* Interrupt service */
void dust_interrupt()
{
  if (digitalRead(pin_dust) == 0)  //fall
  {
    duration_starttime = millis();
  }else
  {
    duration = millis() - duration_starttime;
    if (duration > 10 && duration < 200)
    {
      lowpulseoccupancy+=duration;
    }
    if ((millis()-dust_starttime) > sampletime_ms)
    {
      valid_dust = true;
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      lowpulseoccupancy = 0;
      dust_starttime = millis();
    }
  }
}

/* Timer1 Service */
/*static int cntr_aq_sample = 0;
ISR(TIMER1_OVF_vect)
{
  
  int snd_raw = analogRead(pin_sound);return;
  if (snd_raw > snd_raw_max) snd_raw_max = snd_raw;
  if((++cntr_snd)%5 == 0)
  {
    snd_sum_100ms += snd_raw_max;
    snd_raw_max = 0;
  }
  
  if (cntr_snd >= 5000)
  {
    cntr_snd = 0;
    uint16_t snd_this_avg = snd_sum_100ms/1000;
    snd_sum_100ms = 0;
    if(snd_last_avg == 0)
      snd_last_avg = snd_this_avg;
    else
      snd_last_avg = (snd_last_avg + snd_this_avg) / 2;
    //Serial.print(snd_this_avg);
    //Serial.print(",");
    //Serial.println(snd_last_avg);
  }
}

//Air quality sensor is OK, dust sensor is either .62 or 0....
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

#define RESOLUTION 65536    // Timer1 is 16 bit
void init_timer1(long us)
{
  TCCR1A = 0;                 // clear control register A
  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer

  long cycles;
  long microseconds = us;   //setup microseconds here
  unsigned char clockSelectBits;
  cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if (cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum

  ICR1 = cycles;
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock

  TIMSK1 = _BV(TOIE1);
  TCNT1 = 0;
  sei();                      //enable global interrupt
}
*/
//*****************************************************************************
//
//! \brief Read temperature
//!
//! cost time: > 250ms
//!
//! \param[in]
//! \param[in]
//!
//! \return  temperature
//! \refer to http://www.seeedstudio.com/wiki/File:Humidity_Temperature_Sensor_pro.zip
//*****************************************************************************
float iReadTemperature(void) {
    float temper;
    temper = dht.readTemperature(true); //true: get F, false: get oC
    return temper;
}

//*****************************************************************************
//
//! \brief Read Humidity
//!
//! cost time: > 250ms
//!
//! \param[in]
//! \param[in]
//!
//! \return  Humidity
//!
//*****************************************************************************
float iReadHumidity(void) {
    float humidity;
    humidity = dht.readHumidity();
    return humidity;
}

//*****************************************************************************
//
//! \brief Read Dust Density
//!
//! cost time: ~0ms , unit: pcs/0.01cf or pcs/283ml
//!
//! \param[in]
//! \param[in]
//!
//! \return  temperature
//!
//*****************************************************************************
float iReadDensityDust(void) {
    return concentration;
}

//*****************************************************************************
//
//! \brief Read Lux value of visible light
//!
//! \param[in]
//! \param[in]
//!
//! \return  Luminance
//!
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
//
//! \brief Read the raw voltage signal of UV sensor
//!
//! \param[in]
//! \param[in]
//!
//! \return  voltage mV
//!
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
//
//! \brief Read the raw voltage signal of sound sensor
//!
//! \param[in]
//! \param[in]
//!
//! \return  voltage mV
//!
//*****************************************************************************
int iReadSoundRawVol() {
  return snd_last_avg * (int)(4980.0f / 1023.0f);
}

void postData()
{
  Process p; // Used to send command to Shell, and view response
  String curlCmd; // Where we'll put our curl command

  // Construct the curl command:
  curlCmd = (curlStart);
  int i = 0;
  for (i=0; i<(NUM_FIELDS-1); i++) {
    curlCmd += "\""+ fieldName[i] + "\": " + fieldData[i] + ", "; // Add our data fields to the command
  }
  curlCmd += "\""+ fieldName[8] + "\": " + fieldData[8];
  curlCmd += curlClose; // Add the server URL, including public key
  curlCmd += userKey;
  curlCmd += entries;
  
  // Send the curl command:
  Serial.print(F("Sending command: "));
  Serial.println(curlCmd); // Print command for debug
  p.runShellCommandAsynchronously(curlCmd); // Send command through Shell
  
  // Read out the response:
  Serial.print(F("Response: "));
  //Serial.println(phant.parseInt());  
  // Use the phant process to read in any response from Linux:
  while (p.available() > 0)
  {
    char c = p.read();
    Serial.print(c);
  }
  Serial.flush();
}

/////////////////////////////////////////////////////////////////////
// Synchronize clock using NTP
void setClock() {  
  Process p;
  
  Serial.println(F("Setting clock."));
  
  // Sync clock with NTP
  p.runShellCommand(F("ntpd -nqp 0.openwrt.pool.ntp.org"));
  
  // Block until clock sync is completed
  while(p.running());
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
