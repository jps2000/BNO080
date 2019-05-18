/*********************************************************************

 Code for BNO080 and nfr52 feather
 IMU data are  sentto Adafruit Bluefruit LE app
 Copyright JP Schramel Jan 3 2019
 MIT license, check LICENSE for more information
 Text to be included in any redistribution
 ***********************************************
 
 Based upon Adafruit bleuart example:
 This is an example for our nRF52 based Bluefruit LE modules
 Pick one up today in the adafruit shop!
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution.
 ******************************************************************
 Features
 # Sends data from IMU BNO080 to Adafruit BLE connect app. 
 # Reports battery level
 # BNO Calibrate by sending "c" via  bleuart
 # BNO Tare by sending "t" via bleuart

Connect BNO080 breakout  to feather nrf52:
SDA --> SDA  (4k7 pullup)
SCL--> SCL   (4k7 pullup)
N_RST --> pin 27 (4k7 pullup)
3V3
GND
 
*********************************************************************/
/*************************************
                INCLUDES
 *************************************/
#include <bluefruit.h>
#include <Wire.h> 

/*************************************
                DEFINITIONS
 *************************************/
#define serial_out                      // serial out 

//#define quaternions                   // bleuart out (uart)
//#define euler                                        (uart)
#define plotter                                        (plotter)

#define NAME                        "NRF52_BNO080"             // devicename for advertising
#define BNO_ADDRESS                 (0x4A)                     // device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD
#define QP(n)                       (1.0f / (1 << n))          // 1 << n ==  2^-n
#define radtodeg                    (180.0f / PI)

#define VBAT_PIN                    (A7)
#define VBAT_MV_PER_LSB             (0.7324F)     // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER                (0.7128F)     // 0.806M  and 2M voltage divider factor

#define Led                         (17)          // Led internal
#define N_RST                       (27)          // N_RST pin of BNO080

const int serial_interval         = 500;          // serial interval in ms
const int ble_interval            = 50;           // BLE interval in ms

uint8_t cargo[23];                

uint8_t readbuf                   = 0x00;         // buffer to hold uart read data

float Q0,Q1,Q2,Q3, H_est;
uint8_t stat_;                                    // Status (0-3)
float yaw,pitch,roll;

const uint8_t quat_report        = 0x05;          // defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),                                                 // without magnetometer : game rotation vector (0x08), AR/VR Game (0x29)
const int reporting_frequency    = 100;           // reporting frequency in Hz  // note that serial output strongly reduces data rate

const uint8_t B0_rate            = 1000000 / reporting_frequency;              //calculate LSB (byte 0)
const uint8_t B1_rate            = B0_rate >> 8;                               //calculate byte 1                              

unsigned long timer_timeout      = 0;
unsigned long timer_data         = 0;
unsigned long timer_data1        = 0;


float battery_level              = 4100;          // battery level in mV 
float battery_percent            = 100;           // battery level in %
int vbat_raw                     = 0;             // ADC reading of Vbat
 
// define BLE Services
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

//****************************************************************************************************
void setup()
{
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("nrf52 _ BNO080 IMU 9DOF");
  Serial.println("---------------------------");
  
  // pin definitions

  digitalWrite(N_RST, LOW);                // Reset pulse active low  
  pinMode(N_RST, OUTPUT);
  delay(100);                              // not clear how long this must be. reset probably not needed at all
  digitalWrite(N_RST, HIGH);                     

  digitalWrite(Led,LOW);                   // initial state of LED = off  
  pinMode(Led, OUTPUT);
  
  Bluefruit.begin();
  
  Wire.begin();                            // start I2C communication pins are defined in variants  
  //Wire.setClock(400000);                 // select 400 kbps (100, 250) not needed because 400kHz is default 
 
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);
  
  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // Set power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 3, 4 (5,6,7,8, nrf52840 only)
  Bluefruit.setTxPower(4);
  
  Bluefruit.setName(NAME);
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("JPS2000");
  bledis.setModel("Lab 0");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  vbat_raw = analogRead(VBAT_PIN); 
  battery_level= vbat_raw * VBAT_MV_PER_LSB / VBAT_DIVIDER;         //convert to mV
  battery_percent = map(battery_level,3400,4200,0,100);             // linear approximation mV to %
  blebas.write(battery_percent);

  // Set up and start advertising
  startAdv();
  Serial.print("Battery ");
  Serial.print(vbat_raw * VBAT_MV_PER_LSB / VBAT_DIVIDER );
  Serial.println(" mV");

  timer_timeout = millis();         // set timeout timer
  
  Bluefruit.printInfo();            // print SoftDevice Config and BLE Settings

// BNO settings
  Wire.beginTransmission(BNO_ADDRESS);
  while (Wire.endTransmission() != 0);         //wait until device is responding (32 kHz XTO running)
  Serial.println("BNO found");
    
  delay(200);                            //needed to accept feature command; minimum not tested
  set_feature_cmd_QUAT();                // set the required feature report data rate  are generated  at preset report interva 
 
  ME_cal(1,1,1,0);                       // switch autocal on @ booting (otherwise gyro is not on)
}

//******************************************************************************************
void loop()
{ 
   get_QUAT(); 
  
// Forward sensor data to serial 
 #ifdef serial_out
 if (millis() - timer_data1 > serial_interval)
  {  
        Serial.print ("S "); Serial.print (stat_);
        Serial.print ("; E "); Serial.print (H_est + 0.05f,1);                   // including rounding
        Serial.print ("; q0 "); Serial.print (Q0 + 0.00005f,4);                  
        Serial.print ("; q1 "); Serial.print (Q1 + 0.0005f,3);
        Serial.print ("; q2 "); Serial.print (Q2 + 0.0005f,3);
        Serial.print ("; q3 "); Serial.println (Q3 + 0.0005f,3);
        Serial.println();
        timer_data1 = millis();
  }
 #endif 
 //Forward sensor data to BLE

 if ((millis() - timer_data) > ble_interval)                      // send data  in defined interval 
  {
 #ifdef quaternions
   bleuart.print (Q0);                  
   bleuart.print (" "); bleuart.print (Q1);
   bleuart.print (" "); bleuart.print (Q2);
   bleuart.print (" "); bleuart.println (Q3);
 #endif 

 #ifdef euler
   bleuart.print ("S "); bleuart.print (stat_);  
   bleuart.print (" H "); bleuart.print (yaw,1);                 
   bleuart.print (" P "); bleuart.print (pitch,1);
   bleuart.print (" R "); bleuart.println (roll,1); 
 #endif

 #ifdef plotter
  // Having  everything in one string leads to much faster data transfer --> check with nrf logger and toolbox
   bleuart.print((String)"-200,360," + String(yaw,0) +"," + String(pitch,0) +"," + String(roll,0) + "\r\n");  //println does require an additional transfer

  // bleuart.print(-200); bleuart.print(" ");  bleuart.print(360); bleuart.print(" ");  // set limits for plot to override autoscale in app
  // bleuart.print(roll,0);bleuart.print(" ");
  // bleuart.print(pitch,0);bleuart.print(" ");
  // bleuart.println(yaw,0);  
 #endif
             
  timer_data = millis();
  } 
  
  if (stat_ == 3)  digitalWrite(Led,HIGH);                   // status Led              
  else digitalWrite(Led, LOW);      

// get data from BLEUART
  while ( bleuart.available() )
  {
  readbuf = bleuart.read();                       // as read clears also the buffer it can be read only once
  }
 
  if(readbuf == 't')                        // button pressed  stores actual orientation as q(1,0,0,0)
   {
    TARE(); 
    readbuf = 0;                            // clear readbuffer
    for(int i=0; i<2;i++){digitalWrite (Led,LOW); delay(200);digitalWrite (Led,HIGH);delay(200);} // acqknowledge blink 2x                                             
   }    

  if(readbuf == 'c')
   {
    save_DCD();                             // store cal in flash
    delay(200);
    ME_cal(0,0,1,0);                        //autocal acc + gyro stop; magnetometer  cal continues
    readbuf = 0;                            // clear readbuffer
    for(int i=0; i<2;i++){digitalWrite (Led,LOW); delay(200);digitalWrite (Led,HIGH);delay(200);}     // acqknowledge blink 2x 
   }

}   // loop ends here

//************ BLE code *****************************************************************

// callback invoked when central connects

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

}

/*
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
 
  Serial.println();
  Serial.println("Disconnected");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms (244 x 0.625)
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // (fastmode, slow mode) in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

//************** BNO080 code *************************************************************************

void get_QUAT()
{
int16_t q0,q1,q2,q3,h_est;                        // quaternions q0 = qw 1 = i; 2 = j; 3 = k;
float a,b,c,d,norm;
                                                              
  if (quat_report == 0x08 || quat_report == 0x29)
    {
    Wire.requestFrom(BNO_ADDRESS,21);  //21
    int i=0; 
    while (Wire.available())
      {
      cargo[i] = Wire.read();
      i++;
      }
    }
   
  else
    { 
    Wire.requestFrom(BNO_ADDRESS,23); //23
    int i=0; 
      while (Wire.available())
      {
      cargo[i] = Wire.read();
      i++;
      }
    }
    if((cargo[9] == quat_report) )
     {                     
        q1 = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        q2 = (((int16_t)cargo[16] << 8) | cargo[15] );
        q3 = (((int16_t)cargo[18] << 8) | cargo[17] );
        q0 = (((int16_t)cargo[20] << 8) | cargo[19] ); 

       // patch for  rarely occurring wrong data from BNO of unknown reasons. --> Check if q(0,1,2,3) is not unity vector
       
        a = q0 * QP(14); b = q1 * QP(14); c = q2 * QP(14); d = q3 * QP(14);       // apply Q point (quats are already unity vector)
        norm = sqrtf(a * a + b * b + c * c + d * d);
        if(abs(norm - 1.0f) > 0.0015) return;                                      // skip faulty quats; margin is empirically determined
        
        Q0 = a; Q1 = b; Q2 = c; Q3 = d;                                            // update quaternions
        stat_ = cargo[11] & 0x03;                                                  // bits 1:0 contain the status (0,1,2,3) 
     
        if (quat_report == 0x05 || quat_report == 0x09 || quat_report == 0x28 )    // heading accurracy only in some reports available
          {  
          h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
          H_est = h_est * QP(12);                                                  // apply Q point 
          H_est *= radtodeg;                                                       // convert to degrees                
          }
        
        // calculate euler angles                
        yaw   =  atan2f(Q1 * Q2 + Q0 * Q3, Q0 * Q0 + Q1 * Q1 - 0.5f);
        float argument = 2.0f * (Q1 * Q3 - Q0 * Q2);                               // may be > 1 due to rounding errors --> undefined
        if(abs(argument) > 1.0f) argument = 1.0f; 
        pitch =  asinf(argument); 
        roll  =  atan2f(Q0 * Q1 + Q2 * Q3, Q0 * Q0 + Q3 * Q3 - 0.5f);
  
        yaw += PI * 0.5f; // correction of the y axis direction
        if(yaw < 0) yaw += 2.0f * PI;
        if(yaw > 2.0f *PI) yaw -= 2.0f * PI;  
  
        yaw   *= radtodeg;
        yaw    = 360.0f - yaw;
        pitch *= radtodeg;
        roll  *= radtodeg;
 
       }    
     
}

//************************************************************************
//                COMMANDS
//************************************************************************

// This code activates quaternion output  at defined rate

void set_feature_cmd_QUAT()                                // quat_report determines the kind of quaternions (see data sheets)
  {
  uint8_t quat_setup[21] = {21,0,2,0,0xFD,quat_report,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};  
  Wire.beginTransmission(BNO_ADDRESS);   
  Wire.write(quat_setup, sizeof(quat_setup));            
  Wire.endTransmission();
  }

//***********************************************************************************************************************************************
/* This code tares  and stores results in flash after correct positioning and calibration (follow Tare procedure Hillcrest BNO080 Tare function Usage Guide 1000-4045)
* Make sure to run  rotation vector (0x05) and watch h_est and stat (should be << 10 deg and 3 respectively)
* NOTE: tare and calibration are different things
* NOTE: to undo tare persist requires calibration of the device followed by another tare persist with the device oriented y = north bfz = vertical before. 
*/

void TARE()
  {
  uint8_t RV;               // rotation vector used to tare defined in quat_report
  if(quat_report == 0x05) RV = 0x00;
  if(quat_report == 0x08) RV = 0x01; 
  if(quat_report == 0x09) RV = 0x02; 
  uint8_t tare_now[16] = {16,0,2,0,0xF2,0,0x03,0,0x07,RV,0,0,0,0,0,0};                //0x07 means all axes 0x04 = Z axis only; based on rotation vector
  uint8_t tare_persist[16] = {16,0,2,0,0xF2,0,0x03,0x01,RV,0,0,0,0,0,0,0};
  Wire.beginTransmission(BNO_ADDRESS);
  Wire.write(tare_now, sizeof(tare_now));
  //Wire.write(tare_persist, sizeof(tare_persist));                                  // uncomment  for tare persist;
  Wire.endTransmission();              
  }

//***********************************************************************************************************************************************
/* The calibration data in RAM is always updated in background. 'Save DCD'  push the RAM record into the FLASH for reload @ next boot.
   The algorithm will only update the calibration data when it feels the current accuracy is good. You don't need to care about the status or heading error.
   Save  before power off or whenever you would like to do. (Hillcrest information)  
 */

void save_DCD()
  {                                             
  uint8_t save_dcd[16] = {16,0,2,0,0xF2,0,0x06,0,0,0,0,0,0,0,0,0};
  Wire.beginTransmission(BNO_ADDRESS);  
  Wire.write(save_dcd, sizeof(save_dcd));
  Wire.endTransmission();    
  }

//***********************************************************************************************************************************************
/* This code disables the calibration running in the background of the accelerometer gyro and magnetometer.  
 * sensors can be set individually on and off (chapter 6.4.7.1)
 * P0 = accelerometer, P1 = gyro; P2 =magnetometer; P4 = planar accelerometer (without Z axis)
 * 0 = disable; 1 = enable
 */

void ME_cal(uint8_t P0, uint8_t P1, uint8_t P2, uint8_t P4)
  {
  uint8_t me_cal[16] = {16,0,2,0,0xF2,0,0x07,P0,P1,P2,0,P4,0,0,0,0};
  Wire.beginTransmission(BNO_ADDRESS);              
  Wire.write(me_cal, sizeof(me_cal));
  Wire.endTransmission();  
  }

//***********************************************************************************************************************************************
