/* BNO080 Rock bottom code for Arduino Atmega 328P 16Mhz (Arduino Nano etc)
 by: JP Schramel
 date: August 25, 2017
 
 Disclaimer:
 Freeware:  The code is provided "AS IS",without warranty of any kind.
 
 # Demonstrates basic  functionality 9DOF fused quaternions up to 400Hz data rate.
 # Stores calibration data in flash.
 # Tares the quaternion output to (1,0,0,0) in any arbitrary position
 # Enables control about dynamic automatic calibration
 # heading accurracy estimation
 
 
 It uses I2C.h library @ 400kHz for communication.
 Two buttons store dynamic calibration  data in flash and for tare function:
 
 Other feature reports than quaternions (gravities, linear acceleration etc) can be implemented similarly (see HillcrestLab data sheets and application notes)

 Data  are available via Serial.print. Note that this may take significant time and limits max data rate.
 
 Hardware requirements:
 
 ATmega 328p based boards (Nano..., 16MHz) 
 BNO080 breakout board e.g. Hillcrest FSM300
 Needs a 3V3 supply and connect I2C Bus via a level converter 5V --> 3V3!!!  SDA = A4; SCL = A5. 
 
 LED via 470 Ohm --> pin 5 (pin 13), to ground. 
 Button pin A3 to GND => Tare function
 Button pin 6 to GND  =  Calibration function
  
*/

/*************************************
                INCLUDES
 *************************************/
 
#include <I2C.h>                       // This library allows you to communicate with I2C / TWI devices 

/*************************************
                DEFINITIONS
 *************************************/

#define Led                       13              // Led  (Arduino nano = 13) to measure loop duration (3)
#define btn_TARE                  A3              // input pin for button TARE
#define btn_CAL                   6               // input pin for button TARE
int plot_interval                 = 100;          // plot interval in ms

#define BNO_ADDRESS               0x4A            // Device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD

uint8_t cargo[23]; 
uint8_t next_data_seqNum          = 0x10;         // next data sequence number 

float q0,q1,q2,q3;                                // quaternions q0 = qw 1 = i; 2 = j; 3 = k;
float h_est;                                      // heading accurracy estimation
uint8_t stat;                                     // Status (0-3)

                               

const uint8_t quat_report        = 0x05;          // defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),                                                 // without magnetometer : game rotation vector (0x08), AR/VR Game (0x29)
const int reporting_frequency    = 200;            // reporting frequency in Hz  // note that serial output strongly reduces data rate

const uint8_t B0_rate            = 1000000 / reporting_frequency;              //calculate LSB (byte 0)
const uint8_t B1_rate            = B0_rate >> 8;                               //calculate byte 1

                               
/******* Conversions *************/

#define QP(n)                       (1.0f / (1 << n))                   // 1 << n ==  2^-n
#define radtodeg                    (180.0f / PI)



/*************************************
                 SETUP
*************************************/

void setup() {

// pin definitions
  pinMode(Led, OUTPUT);
  pinMode(btn_TARE, INPUT_PULLUP);
  pinMode(btn_CAL, INPUT_PULLUP); 
  digitalWrite(Led, HIGH);              // initial state of LED (on)

// communication 
  Serial.begin(115200);                 // 115200 baud  
  I2c.begin();                          // start I2C communication   
  I2c.setSpeed(1);                      // set I2C to 400kHz

  Serial.println ("*********************************************************************");
  Serial.println (" Rock bottom code for  BNO080 on Arduino Nano etc. (Atmega 328p) V1.0 2017-08-24");
  Serial.println ("*********************************************************************");

// BNO settings
 
  while (I2c.read(BNO_ADDRESS,0) != 0);         //wait until device is responding (32 kHz XTO running)  
  delay(100);                            //needed to accept feature command; minimum not tested
  set_feature_cmd_QUAT();                // set the required feature report data rate  are generated  at preset report interva 
  ME_cal(1,1,1,0);                       // switch autocal on @ booting (otherwise gyro is not on)

}


  /*************************************
                     LOOP
   *************************************/
void loop() {

  get_QUAT();                           // get actual QUAT data (if new are available)
  
 if (millis() - plot_interval > 0){ 
        Serial.print ("S "); Serial.print (stat);
        Serial.print ("; E "); Serial.print (h_est + 0.05f,1);                   // including rounding
        Serial.print ("; q0 "); Serial.print (q0 + 0.00005f,4);                  // = qw (more digits to find out north direction (y axis N --> q0 = 1)
        Serial.print ("; q1 "); Serial.print (q1 + 0.0005f,3);
        Serial.print ("; q2 "); Serial.print (q2 + 0.0005f,3);
        Serial.print ("; q3 "); Serial.println (q3 + 0.0005f,3);
        plot_interval = millis();
 }
 
                           
  if (stat == 3)  digitalWrite(Led, !digitalRead(Led));                   // blink Led every loop run (--> to measure loop duration);             
  else digitalWrite(Led, LOW);                                            // status Led 


// *************  buttons *******************************
  
  if(digitalRead(btn_TARE) == LOW ){                        // button pressed  stores actual phi as mean value and saves actual calibration to flash
    delay(200);
    while(digitalRead(btn_TARE) == LOW);                    // wait for button release
    //actions follow  here
    TARE();                                                 
   }    


   if(digitalRead(btn_CAL) == LOW){
     delay(200);
     while(digitalRead(btn_CAL) == LOW );       // wait for button release
    
     //actions follow here
     save_DCD();                                     // store cal in flash
     delay(200);
     ME_cal(0,0,1,0);                                //autocal acc + gyro stop; magnetometer  cal continues
    }
 //*******************************************************
 
}                                                    // loop ends here

/*************************************
             Subroutines
/*************************************/

/*******************************************************************************************************************
/* This code reads quaternions 
 * kind of reporting (quat_report) is defined above
 */
void get_QUAT(){                                                               
  if (quat_report == 0x08 || quat_report == 0x29) I2c.read(BNO_ADDRESS,21, cargo); 
  else I2c.read(BNO_ADDRESS,23, cargo);
  
      if((cargo[9] == quat_report) && ((cargo[10]) == next_data_seqNum )){      // check for report and incrementing data seqNum
        next_data_seqNum = ++cargo[10];                                         // predict next data seqNum              
        stat = cargo[11] & 0x03;                                                // bits 1:0 contain the status (0,1,2,3)  
    
        q1 = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        q2 = (((int16_t)cargo[16] << 8) | cargo[15] );
        q3 = (((int16_t)cargo[18] << 8) | cargo[17] );
        q0 = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        q0 *= QP(14); q1 *= QP(14); q2 *= QP(14); q3 *= QP(14);                  // apply Q point (quats are already unity vector)

       if (quat_report == 0x05 || quat_report == 0x09 || quat_report == 0x28 ){  // heading accurracy only in some reports available
        h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
        h_est *= QP(12);                                                         // apply Q point 
        h_est *= radtodeg;                                                       // convert to degrees                
       }
       
      
      }
}

//************************************************************************
//                COMMANDS
//************************************************************************
/* This code activates quaternion output  at defined rate
*/
void set_feature_cmd_QUAT(){                                                                 // quat_report determines the kind of quaternions (see data sheets)
  uint8_t quat_setup[20] = {0,2,0,0xFD,quat_report,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};  
  I2c.write(BNO_ADDRESS, sizeof(quat_setup)+1, quat_setup, sizeof(quat_setup));            // sizeof(quat_setup)+1 is the LSB length in the SHTP header 
}

//***********************************************************************************************************************************************
/* This code tares  and stores results in flash after correct positioning and calibration (follow Tare procedure Hillcrest BNO080 Tare function Usage Guide 1000-4045)
* Make sure to run  rotation vector (0x05) and watch h_est and stat (should be << 10 deg and 3 respectively)
* NOTE: tare and calibration are different things
*/

void TARE(){
  uint8_t tare_now[15] = {0,2,0,0xF2,0,0x03,0,0x07,0,0,0,0,0,0,0};                //0x07 means all axes 0x04 = Z axis only; based on rotation vector
  uint8_t tare_persist[15] = {0,2,0,0xF2,0,0x03,0x01,0,0,0,0,0,0,0,0};
  I2c.write(BNO_ADDRESS, sizeof(tare_now)+1, tare_now, sizeof(tare_now));
               
  //I2c.write(BNO_ADDRESS, sizeof(tare_persist)+1, tare_persist, sizeof(tare_persist));  // uncomment  for tare persist;
  // NOTE: to undo tare persist requires calibration of the device followed by another tare persist with the device oriented y = north bfz = vertical before. 

 }

//***********************************************************************************************************************************************
/* The calibration data in RAM is always updated in background. 'Save DCD'  push the RAM record into the FLASH for reload @ next boot.
   The algorithm will only update the calibration data when it feels the current accuracy is good. You don't need to care about the status or heading error.
   Save  before power off or whenever you would like to do. (Hillcrest information)  
 */

void save_DCD(){                                             
  uint8_t save_dcd[15] = {0,2,0,0xF2,0,0x06,0,0,0,0,0,0,0,0,0};
  I2c.write(BNO_ADDRESS, sizeof(save_dcd)+1, save_dcd, sizeof(save_dcd));   
}

//***********************************************************************************************************************************************
/* This code disables the calibration running in the background of the accelerometer gyro and magnetometer.  
 * sensors can be set individually on and off (chapter 6.4.7.1)
 * P0 = accelerometer, P1 = gyro; P2 =magnetometer; P4 = planar accelerometer (without Z axis)
 * 0 = disable; 1 = enable
 */

void ME_cal(uint8_t P0, uint8_t P1, uint8_t P2, uint8_t P4){
  uint8_t me_cal[15] = {0,2,0,0xF2,0,0x07,P0,P1,P2,0,P4,0,0,0,0};           
  I2c.write(BNO_ADDRESS, sizeof(me_cal)+1, me_cal, sizeof(me_cal));
}

//***********************************************************************************************************************************************

/* Utilities (Quaternion mathematics)
 // q0 = qw
  q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3  = 1
   
 // Gravity vector from quaternions
  
  gx = q1 * q3 - q0 * q2;               // R20
  gy = q2 * q3 + q0 * q1;               // R21
  gz = q0 * q0 + q3 * q3 -0.5f;         // R22
  norm = sqrtf(gx * gx + gy * gy + gz * gz);                                                              
  norm = 1.0f / norm;
  gx *= norm; gy *= norm; gz *= norm;
 
 // Euler angles from quaternions (radians)   

  yaw   =  atan2((q1 * q2 + q0 * q3), ((q0 * q0 + q1 * q1) - 0.5f));   
  pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
  roll  =  atan2((q0 * q1 + q2 * q3), ((q0 * q0 + q3 * q3) - 0.5f));
  yaw *= radtodeg;    pitch *= radtodeg;    roll *= radtodeg; 
# Rotation matrix (half)    
R00 = q0 * q0 + q1 * q1 -0.5f; 
R01 = q1 * q2 - q0 * q3;
R02 = q1 * q3 + q0 * q2;
R10 = q1 * q2 + q0 * q3;
R11 = q0 * q0 + q2 * q2 -0.5; 
R12 = q2 * q3 - q0 * q1;
R20 = q1 * q3 - q0 * q2;
R21 = q2 * q3 + q0 * q1;
R22 = q0 * q0 + q3 * q3 -0.5f; 
 
//***********************************************************************************
tare multiplication 
//before tare store  tare coefficients (^-1 = konjugate komplex)
    t0 = q0;                                                                        
    t1 = -q1;
    t2 = -q2;
    t3 = -q3;
  // qt are the tared coefficients
  qt0 = q0 * t0 - q1 * t1 - q2 * t2 - q3 * t3;      // multiplication matrix
  qt1 = q0 * t1 + q1 * t0 + q2 * t3 - q3 * t2;
  qt2 = q0 * t2 - q1 * t3 + q2 * t0 + q3 * t1;
  qt3 = q0 * t3 + q1 * t2 - q2 * t1 + q3 * t0;

alternative:
//before tare store  tare coefficients (^-1 = konjugate komplex)
    t0 = q0;                                                                        
    t1 = q1;
    t2 = q2;
    t3 = q3;
  // qt are the tared coefficients
  qt0 =  q0 * t0 + q1 * t1 + q2 * t2 + q3 * t3; // qt are the tared quaternions to achieve 1,0,0,0 at moment of tare
  qt1 = -q0 * t1 + q1 * t0 - q2 * t3 + q3 * t2; // konjugated multiplication matrix t1, t2, t3 negative sign
  qt2 = -q0 * t2 + q1 * t3 + q2 * t0 - q3 * t1;
  qt3 = -q0 * t3 - q1 * t2 + q2 * t1 + q3 * t0;
//*************************************************************************************
*/
