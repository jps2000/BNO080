/* BNO080 Basic Example Code for Arduino Atmega 328P 16Mhz (Arduino Nano etc)
 by: JP Schramel
 date: June 9, 2017
 
 Disclaimer:
 Freeware:  The code is provided "AS IS",without warranty of any kind.
 
 Demonstrate basic  functionality comprising high performance quaternions up to 400Hz data rate.
 Test: # Gravity vector (quaternions)should be as stable as possible while doing fast linear movements (forward/backward).
       # Acurrate response to periodic and fast tilt, no error accumulation during  periodic movements.
 
 It uses I2C.h library @ 400kHz to communicate.
 Two buttons store dynamic calibration  data in flash and for tare function:
 
 Other feature reports than quaternions, gravities, linear acceleration etc) can be implemented similarly (see HillcrestLab data sheets and application notes)

 Data  are available via Serial.print. Note that this may take significant time and limit max data rate provided by the BNO080

 Uncomment "SerialDebug" to read full advertisement and communication. 
 
 Uncomment "I2c.scan" to report connected I2C devices (BNO080 address is 0x4A or 0x4B)
 
 Hardware requirements:
 
 ATmega 328p based boards (Nano..., 16MHz) 
 BNO080 needs a 3V3 supply.
  I2C Bus needs a level converter 5V --> 3V3!!!  SDA = A4; SCL = A5. 
 
 NRST pin  --> pin7
 LED via 470 Ohm --> pin 5 (pin 13),
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

#define SerialDebug                              // uncomment to get full reports for debugging

#define Led                       5              // Led  (Arduino nano = 13) to measure loop duartion
#define btn_LEFT                  A3             // input pin for button LEFT
#define btn_RIGHT                 6              // input pin for button RIGHT

#define NRST                      7              // Reset pin
#define btn_TARE                  A3             // input pin for button TARE
#define btn_CAL                   6              // input pin for button TARE

#define BNO_ADDRESS             0x4A             // Device address when SA0 Pin 17 = 0; 0x4B otherwise

float q0;                                           // quaternions q0 = qw 1 = i; 2 = j; 3 = k;
float q1;
float q2;
float q3;

float h_est;                                       // heading accurracy estimation
uint8_t stat;                                      // Status (0-3)

float gx;                                          //gravity
float gy;
float gz;

float lax;                                         //linear acceleration
float lay;
float laz;

float vx;                                          //velocity
float vy;
float vz;
float delta_t;                                     //report interval

int len                           = 0;
uint8_t next_data_seqNum          = 0x0A;          // reading next data sequence number start with 10 to catch up earlier
uint8_t seqNum_2                  = 0;             // writing sequence number channel 2 
uint8_t cargo[300];                                

uint8_t quat_report               = 0x05;          // defines kind of rotation vector  (0x05), geomagnetic (0x09) or AR/VR (0x28)
#define reporting_frequency       200              // reporting frequency in Hz
// report interval need to be longer than loop durtion not to miss any report ! --> code checks for incrementing seqNum  
// note that serial output strongly reduces data rate

uint8_t B0_rate                  = 1000000 / reporting_frequency;              //calculate LSB (byte 0)
uint8_t B1_rate                  = (1000000 / reporting_frequency) >> 8;       //calculate byte 1

                               
/******* Constants *************/
 
#define QP8                         0.00390625f     // Q point factor to convert to float (*) (QPn = 2^-n)
#define QP12                        0.000244141f
#define QP14                        0.0000610352f
#define radtodeg                    57.2958f        // constant 180/PI 



/*************************************
                 SETUP
*************************************/

void setup() {
 
  pinMode(NRST, OUTPUT);  
  pinMode(Led, OUTPUT);
  pinMode(btn_TARE, INPUT_PULLUP);
  pinMode(btn_CAL, INPUT_PULLUP);
  
  digitalWrite(Led, HIGH);              // initial state of LED (on)
  digitalWrite(NRST, LOW);              // keep in reset
  
  Serial.begin(115200);                 // 115200 baud 
  
  I2c.begin();                          // start I2C communication   
  I2c.setSpeed(1);                      // set I2C to 400kHz

  Serial.println ("*********************************************************************");
  Serial.println (" Basic code for BNO080 on Arduino Nano (Atmega 328p) V1.0 2017-09-06");
  Serial.println ("*********************************************************************");
  //I2c.scan();                         // uncomment to report devices on the I2C
  Serial.println ();


  /************************************************
            INITIALIZE BNO
   ************************************************/
 
  digitalWrite(NRST, HIGH);              // release reset
  delay(2500);                           // wait until advert etc is available (also without serial) (up to 2500??)
  
  #ifdef SerialDebug
  do get_message();                      // read out all cargos
  while(len > 0);    
  #endif
 
  ME_cal(1,1,1,0);                       // switch autocal on @ booting (otherwise gyro is not on)
  
  set_feature_cmd_QUAT();                // set the required feature report (QUAT; AV/VR), data are generated  at preset report interva 



}
  /*************************************
                     LOOP
   *************************************/
void loop() {

  get_QUAT();                           // get actual QUAT data (if new are available)
                           
  if (stat == 3)  digitalWrite(Led, !digitalRead(Led));                   // blink Led every loop run (--> to measure loop duration); stat == 3             
  else digitalWrite(Led, LOW);                                            // status Led 


// *************  buttons *******************************
  
  if(digitalRead(btn_TARE) == LOW ){                        // button pressed  stores actual phi as mean value and saves actual calibration to flash
    delay(100);
    while(digitalRead(btn_TARE) == LOW);                    // wait for button release
    //action comes here

    TARE();                                                 // needs rotation vector (QUAT 0x05)
   }    


   if(digitalRead(btn_CAL) == LOW){
       delay(200);
       while(digitalRead(btn_RIGHT) == LOW );       // wait for button release
    
    //action comes here
    save_DCD();
    delay(200);
    ME_cal(0,0,1,0);                               //autocal of magneto only
    }

}

   /*************************************
                  Subroutines
    *************************************/

/*******************************************************************************************************************
/* This code reads quaternions and looks for new data (incrementing sequence number)
 * kind of reporting is defined above
 * Rotation vector (0x05) 
 * Geomagnetic rotation vector (0x09)
 * AV/VR stabilized Rotation vector (0x28)
 */
void get_QUAT(){                                                               
  I2c.read(BNO_ADDRESS, 4, cargo);
  
  if(cargo[0] != 0){                                                            // to reduce I2C bus and computing --> read header first
    
    I2c.read(BNO_ADDRESS, 23, cargo);                                           // length is predicted (war 23)
      
      if((cargo[9] == quat_report) && ((cargo[10]) == next_data_seqNum )){           // check for report and incrementing data seqNum
        next_data_seqNum = ++cargo[10];                                         // predict next data seqNum    
        stat = cargo[11] & 0x03;                                                // bits 1:0 contain the status  
    
        q1 = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        q2 = (((int16_t)cargo[16] << 8) | cargo[15] );
        q3 = (((int16_t)cargo[18] << 8) | cargo[17] );
        q0 = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        q0 *= QP14; q1 *= QP14; q2 *= QP14; q3 *= QP14;                                   // apply Q point (quats are already unity vector)
  
        h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                          // heading accurracy estimation  
        h_est *= QP12;                                                                      // apply Q point 
        h_est *= radtodeg;                                                                  // convert heading accurracy to degrees                
    
        Serial.print ("S "); Serial.print (stat);
        Serial.print ("; E "); Serial.print (h_est + 0.05f,1);                    // including rounding
        Serial.print ("; q0 "); Serial.print (q0 + 0.00005f,4);                   // = qw ( more digits to find out north direction (y axis N --> q0 = 1)
        Serial.print ("; q1 "); Serial.print (q1);
        Serial.print ("; q2 "); Serial.print (q2);
        Serial.print ("; q3 "); Serial.println (q3);

      }
   }
}

/****************************************************************************************************************************************** 
 *  This code reads gravity and checks for new data (incrementing sequence number) 
 */
 
void get_GRAV(){
  float norm;
  I2c.read(BNO_ADDRESS, 21, cargo);                                        // 21 =  predicted lenght incl header    
  if((cargo[9] == 0x06) && (cargo[10] == next_data_seqNum)){     //check for gravity sensor (0x06) and new data seqNum

    next_data_seqNum = ++cargo[10];                                        // predict next data seqNum before updating   

    stat = cargo[11] & 0x03;                                               //bit 1:0 are the status 
  
    gx = (((int16_t)cargo[14] << 8) | cargo[13] ); 
    gy = (((int16_t)cargo[16] << 8) | cargo[15] );
    gz = (((int16_t)cargo[18] << 8) | cargo[17] );

    /*gx *= QP8 ; gy *= QP8; gz *= QP8;                                  // calculate results in ms^-2
    */
    norm = sqrtf(gx + gx + gy * gy + gz * gz);                           // normalize                                     
    norm = 1.0f / norm;
    gx *= norm; gy *= norm; gz *= norm;
   
    Serial.print ("S "); Serial.print (stat); 
    Serial.print ("; gx ");Serial.print (gx); 
    Serial.print ("; gy ");Serial.print (gy);
    Serial.print ("; gz ");Serial.println (gz);

 }
}

/****************************************************************************************************************************************** 
 *  This code reads linear acceleration and checks for new data (incrementing sequence number)
 *  Velocity is calculated for dead reckoning wit delta_t ( reporting interval + delay
 */
void get_LINAC(){    
 I2c.read(BNO_ADDRESS, 21, cargo);                                         // 21 =  predicted lenght incl header
 if((cargo[9] == 0x04) && ((cargo[10]) == next_data_seqNum)){              // check for linacc data (0x04) and new data seqNum
    next_data_seqNum = ++cargo[10];                                        // predict next data seqNum before updating    
    stat = cargo[11] & 0x03;                                               // bit 1:0 are the status others are the MSB of delay; need to be here because bit later shift destroys data

    
    delta_t = 20.0f  + (uint32_t)cargo[12] + ((uint32_t)cargo[11] >> 2) * 10.0f  ;    // calculate delta_t  in milliseconds (report interval + delay)   
    
    lax = (((int16_t)cargo[14] << 8) | cargo[13] ); 
    lay = (((int16_t)cargo[16] << 8) | cargo[15] );
    laz = (((int16_t)cargo[18] << 8) | cargo[17] );
    
    lax *= QP8; lay *= QP8; laz *= QP8;                                                // convert to ms^-2 

    vx = lax * delta_t ;                                                               // velocity in mm s^-1
    vy = lay * delta_t; 
    vz = laz * delta_t; 
  

    Serial.print ("S "); Serial.print (stat); 
    /*
    Serial.print ("; d_t "); Serial.print (delta_t);
    Serial.print ("; lax ");Serial.print (lax); 
    Serial.print ("; lay ");Serial.print (lay);
    Serial.print ("; laz ");Serial.print (laz);
    */
    Serial.print ("; vx ");Serial.print (vx); 
    Serial.print ("; vy ");Serial.print (vy);
    Serial.print ("; vz ");Serial.println (vz);

 }
}
//************************************************************************
//                COMMANDS
//************************************************************************
/* This code activates quaternion output  at a dedicated data rate
*/
void set_feature_cmd_QUAT(){                                                                 // report 0x05 = rotation vector, 0x28 = AR / VR Stabilized rotation vector
 uint8_t quat_setup[20] = {0,2,seqNum_2,0xFD,quat_report,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};   
  
  I2c.write(BNO_ADDRESS,21,quat_setup,20);                                                   // register adress (21) is the LSB length in the SHTP header 
  ++seqNum_2;                                                                                //update write seq number
  
   Serial.println (); 
   Serial.println ("******** set feature command quat **********");

   #ifdef SerialDebug
   delay(200);                                                                              // wait until response is available
   get_message();                                                                           // to report feature response and remaining data
   get_message();                                                                          
   get_message();
   #endif  
}

//***********************************************************************************************************************************************
/* This code activates gravity-free acceleration at a dedicated data rate
*/
void set_feature_cmd_LINAC(){                 
 uint8_t linac_setup[20] = {0,2,seqNum_2,0xFD,0x04,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};      // report interval 20ms  50Hz

  I2c.write(BNO_ADDRESS,21,linac_setup,20);                                                   // register adress is the LSB of the header
  ++seqNum_2;                                                                                 //update write seq number

  Serial.println ();  
  Serial.println ("******** set feature command linac ********** ");
 
#ifdef SerialDebug  
  delay(200);                                                                              // wait until response is available
  get_message();                                                                           // to report feature response and remaining data
  get_message();                                                                          
  get_message(); 
  #endif
}

//***********************************************************************************************************************************************
/* This code activates gravity (vertical orientation) at a dedicated data rate */

void set_feature_cmd_GRAV(){                 
 
 uint8_t grav_setup[20] = {0,2,seqNum_2,0xFD,0x06,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};    // report interval 5ms   200Hz

  I2c.write(BNO_ADDRESS,21,grav_setup,20);                                                   // register adress is the LSB of the header
  ++seqNum_2;                                                                                //update write seq number


   Serial.println ();   
   Serial.println ("******** set feature command grav ********** "); 

   #ifdef SerialDebug
   delay(200);                                                                              // wait until response is available
   get_message();                                                                           // to report feature response and remaining data
   get_message();                                                                          
   get_message(); 
   #endif 
}

//***********************************************************************************************************************************************
/* This code tares  and stores results in flash after correct positioning and calibration (follow Tare procedure Hillcrest BNO080 Tare function Usage Guide 1000-4045)
* Make sure to run  rotation vector (0x05) and watch h_est and stat (should be << 10 deg and 3 respectively)
* Note that tare and calibration are different things
*/

void TARE(){
  uint8_t tare_now[15] = {0,2,seqNum_2,0xF2,0,0x03,0,0x07,0,0,0,0,0,0,0};                //0x07 means all axes 0x04 = Z axis only; based on rotation vector
  uint8_t tare_persist[15] = {0,2,seqNum_2,0xF2,0,0x03,0x01,0,0,0,0,0,0,0,0};

  I2c.write(BNO_ADDRESS,16,tare_now,15);                                                 // register adress is the LSB of the header 
  ++seqNum_2; 
  
  I2c.write(BNO_ADDRESS,16,tare_persist,15);                                             // register adress is the LSB of the header
  ++seqNum_2; 


  Serial.println ();
  Serial.println ("******** TARE all axes and store ********** ");                       // there is no reporting on action

}

//***********************************************************************************************************************************************
/* The calibration data in RAM is always updated in background. 'Save DCD'  push the RAM record into the FLASH for reload @ next boot.
   The algorithm will only update the calibration data when it feels the current accuracy is good. You don't need to care about the status or heading error.
   Save  before power off or whenever you would like to do. (Hillcrest information)  
 */

void save_DCD(){
  Serial.println ();
  Serial.println ("******** save DCD ********** ");                                               
  
  uint8_t save_dcd[15] = {0,2,seqNum_2,0xF2,0,0x06,0,0,0,0,0,0,0,0,0};
  I2c.write(BNO_ADDRESS,16,save_dcd,15);                                             // 16 is the LSB of the header 15 bytes to follow
  ++seqNum_2;

   #ifdef SerialDebug  
   delay(200);                                                                        // wait until response is available
   get_message();                                                                    // to report feature response and remaining data
   get_message();                                                                          
   get_message();  
   #endif     
}

//***********************************************************************************************************************************************
/* This code disables the calibration running in the background of the accelerometer gyro and magnetometer.  
 * sensors can be set individually on and off
 * P0 = accelerometer, P1 = gyro; P2 =magnetometer; P4 = planar accelerometer (without Z axis)
 */

void ME_cal(uint8_t P0, uint8_t P1, uint8_t P2, uint8_t P4){
  Serial.println ();
  Serial.println ("******** configure ME cal ********** "); 
  
  
  uint8_t me_cal[15] = {0,2,seqNum_2,0xF2,0,0x07,P0,P1,P2,0,P4,0,0,0,0};           // disable all calibrations (chapter 6.4.7.1)
  
  I2c.write(BNO_ADDRESS,16,me_cal,15);                                             // 16 is the LSB of the header 15 bytes to follow
  ++seqNum_2;
  
  #ifdef SerialDebug    
  delay(200);                                                                      // wait until response is available
  get_message();                                                                  // to report feature response and remaining data
  get_message();                                                                          
  get_message();
  #endif
     
}

//***********************************************************************************************************************************************
/* This code reads  the calibration setting
 */
void get_cal(){
  Serial.println ();
  Serial.println ("******** CAL status********** ");                                               
  
  uint8_t save_dcd[15] = {0,2,seqNum_2,0xF2,0,0x07,0,0,0,1,0,0,0,0,0};
  I2c.write(BNO_ADDRESS,16,save_dcd,15);                                             // 16 is the LSB of the header 15 bytes to follow
  ++seqNum_2;

  #ifdef SerialDebug  
  delay(200);                                                                        // wait until response is available
  get_message();                                                                    // to report feature response and remaining data
  get_message();                                                                          
  get_message();  
  #endif
       
}


//***********************************************************************************************************************************************
/* This code reads the header and fetches available data, The I2C routine can read max 32 bytes at once (or max 256). 
 * As sometimes more bytes are waiting multiple reads has to be performed
 * 
 */
void get_message(){
  int i = 0;
  int j = 0;  

//******************* header******************************
  I2c.read(BNO_ADDRESS, 4,cargo);
  
  len = (((uint16_t)(cargo[1] & 0x7F) << 8)| cargo[0] ) ;       // MSB 14:0 remaining cargo length need to block Bit 15. (Bit 15 HIGH means continued 
    
  #ifdef SerialDebug
    if (len != 0){
      Serial.println ("");  
      Serial.print ("ch");Serial.print (cargo[2]);     
      Serial.print (": len "); Serial.print (len);
      Serial.print (" seq_num "); Serial.print (cargo[3]);
      Serial.print (" data: ");
    } 
  #endif
                                                  
//*********************** read channels***************************
  if(len != 0){                                                       // read only if there is something to report                                       
    while ((len - 28) >= 0){                                          // 28 = 32 - 4 
      i = 0;                                                          // i = pointer within a read
      I2c.read(BNO_ADDRESS, 32);                                      // dump   part of the cargo incl header (max 32, may be also > 256)  
      while(I2c.available() > 0){                                     // every read generates one header (length + 4)
        if (i < 4){I2c.receive();}                                    // cargo contain data without header
        else{cargo[j++] = I2c.receive();}
        i++;   
        }         
      len -= 28;                                                      // update cargo length (30-4)
    }

    I2c.read(BNO_ADDRESS, len);                                       // dump remaining part of the cargo incl header
    i = 0;                                                            // i = pointer within a read
    while(I2c.available() > 0){
        if (i < 4){I2c.receive();}
        else{cargo[j++] = I2c.receive();}
        i++;   
    }

  #ifdef SerialDebug       
  for(i = 0; i < j; i++){Serial.print (cargo[i],HEX);Serial.print (",");} // Print  cargo without header                
  #endif                                         
 
  }
}

//***********************************************************************************************************************************************

/* Utilities (Quaternion mathematics)
 
 # q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3  = 1
   
 # Gravity vector from quaternions
  
  gx = q1 * q3 - q0 * q2;
  gy = q0 * q1 + q2 * q3;
  gz = q0 * q0 + q3 * q3 -0.5f; 
  
  //calculate unity vector
  norm = sqrtf(gx + gx + gy * gy + gz * gz);                           // normalize                                     
  norm = 1.0f / norm;
  gx *= norm; gy *= norm; gz *= norm;
 
 # Euler angles from quaternions (radians)   

  yaw   =  atan2((q1 * q2 + q0 * q3), ((q0 * q0 + q1 * q1) - 0.5f));   
  pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
  roll  =  atan2((q0 * q1 + q2 * q3), ((q0 * q0 + q3 * q3) - 0.5f));
    
 # Rotation matrix (half)
 
 R00 = q1 * q1 + q2 * q2 -0.5f; // = 0.5f - q3 * q3 - q4 * q4;
 R01 = q2 * q3 - q1 * q4;
 R02 = q2 * q4 + q1 * q3;
 R10 = q2 * q3 + q1 * q4;
 R11 = q1 * q1 + q3 * q3 -0.5; // = 0.5f - q2 * q2 - q4 * q4;
 R12 = q3 * q4 - q1 * q2;
 R20 = q2 * q4 - q1 * q3;
 R21 = q3 * q4 + q1 * q2;
 R22 = q1 * q1 + q4 * q4 -0.5f; // = 0.5f -q2 * q2 -q3 * q3;

*/
