#include <Wire.h>
#include <nRF5x_BLE_API.h>
int k=0;
#define Addr 0x68


#define BME280_ADDRESS 0x76
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;
 int8_t  dig_H1;
 int16_t dig_H2;
 int8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;


#define TXRX_BUF_LEN                      200



#ifdef RBL_NRF51822
#define DIGITAL_OUT_PIN                   D2
#define DIGITAL_IN_PIN                    A4
#define PWM_PIN                           D3
#define SERVO_PIN                         D5
#define ANALOG_IN_PIN                     A5
//#define ANALOG_IN_PIN_X                   A2
#endif

#ifdef BLE_NANO
#define DIGITAL_OUT_PIN                   D2
#define DIGITAL_IN_PIN                    D3
#define PWM_PIN                           D4
#define SERVO_PIN                         D5
#define ANALOG_IN_PIN                     A3
#define ANALOG_IN_PIN_X                   A2
#endif

#ifdef RBL_BLEND2
#define DIGITAL_OUT_PIN                   D2
#define DIGITAL_IN_PIN                    D3
#define PWM_PIN                           D4
#define ANALOG_IN_PIN_X                   A2
#define ANALOG_IN_PIN_Y                   A3
#define ANALOG_IN_PIN_Z                   A4
#define SOUND_IN_PIN                      A5
#endif

BLE                                       ble;
Ticker                                    ticker;
//Servo                                    myservo;

static boolean analog_enabled = false;
static byte old_state         = LOW;

// The uuid of service and characteristics
static const uint8_t service1_uuid[]        = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]     = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]     = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]   = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0};
uint8_t rx_value[TXRX_BUF_LEN] = {0};

// Create characteristic
GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));


void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  //Serial.println("Disconnected!");
  //Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t index;
  uint16_t bytesRead = TXRX_BUF_LEN;

  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
   // Serial.print("bytesRead: ");
    //Serial.println(bytesRead, HEX);
    for(index=0; index<bytesRead; index++) {
      //Serial.print(buf[index], HEX);
      //Serial.print(" ");
    }
    //Serial.println("");
    //Process the data
    if (buf[0] == 0x01) {
      // Command is to control digital out pin
      if (buf[1] == 0x01)
        digitalWrite(DIGITAL_OUT_PIN, HIGH);
      else
        digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
    else if (buf[0] == 0xA0) {
      // Command is to enable analog in reading
      if (buf[1] == 0x01)
        {analog_enabled = true;
         k=1;}
      else{
        analog_enabled = false;
         k=2;}
    }
    else if (buf[0] == 0x02) {
      // Command is to control PWM pin
      //analogWrite(PWM_PIN, buf[1]);

      ////send gyro data
     if (buf[1] == 0x01)
        {analog_enabled = true;
         k=3;}
      else{
        analog_enabled = false;
         k=4;}
    }
    else if (buf[0] == 0x03)  {
      // Command is to control Servo pin
//      myservo.write(buf[1]);
//digitalWrite(DIGITAL_OUT_PIN, LOW);
if (buf[1] == 0x01)
        {analog_enabled = true;
         k=5;}
      else{
        analog_enabled = false;
         k=6;}

    }
    else if (buf[0] == 0x04) {
      //analog_enabled = false;
//      myservo.write(0);
      //analogWrite(PWM_PIN, 0);
      //digitalWrite(DIGITAL_OUT_PIN, LOW);
      //old_state = LOW;
      if (buf[1] == 0x01)
        {analog_enabled = true;
         k=7;}
      else{
        analog_enabled = false;
         k=8;}
    }
  }
}


void m_status_check_handle() {
  


  /////////////////////////////////////////////////////////////////////////////////////////////////
  // If digital in changes, report the state
  /*if (digitalRead(DIGITAL_IN_PIN) != old_state) {
    old_state = digitalRead(DIGITAL_IN_PIN);
    if (digitalRead(DIGITAL_IN_PIN) == HIGH) {
      buf[0] = (0x0A);
      buf[1] = (0x01);
      buf[2] = (0x00);
      ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 3);
    }
    else {


      Serial.print("Inside else");
      buf[0] = (0x0A);
      buf[1] = (0x00);
      buf[2] = (0x00);
      ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 3);
    }
  }*/
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Gyro




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//BME




void setup()
{
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    //Serial.begin(9600);
    Wire.begin();

    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    readTrim();                    //


//////////////////////////////////////////////////////////////////////////////////////////////

// Initialise I2C communication as MASTER   
Wire.begin();  
// Initialise Serial Communication, set baud rate = 9600  
//Serial.begin(9600);
 // Start I2C Transmission  
Wire.beginTransmission(Addr);  
// Select Range register  
Wire.write(0x0F);  
// Configure full scale range 2000 dps  
Wire.write(0x80);  
// Stop I2C Transmission  
Wire.endTransmission();
 // Start I2C Transmission  
Wire.beginTransmission(Addr);  
// Select Bandwidth register  
Wire.write(0x10);  
// Set bandwidth = 200 Hz  
Wire.write(0x04);  
// Stop I2C Transmission  
Wire.endTransmission();  
delay(300);

/////////////////////////////////////////////////////////////////////////////////////////////////

    ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"TXRX", sizeof("TXRX") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)"Simple Controls");
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();

  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  pinMode(DIGITAL_IN_PIN, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);

  // Default to internally pull high, change it if you need
  digitalWrite(DIGITAL_IN_PIN, HIGH);

  //myservo.attach(SERVO_PIN);
  //myservo.write(0);

  ticker.attach_us(m_status_check_handle, 200000);

  //Serial.println("Advertising Start!");
}


void loop()
{
    /*double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;

    readData();

    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;


    uint16_t hum_act_new= (uint16_t)hum_act;
    uint16_t press_act_new= (uint16_t)press_act;
    uint16_t temp_act_new= (uint16_t)temp_act;
    Serial.print("TEMP : ");
    Serial.print(temp_act_new);
    Serial.print(" DegC  PRESS : ");
    Serial.print(press_act_new);
    Serial.print(" hPa  HUM : ");
    Serial.print(hum_act_new);
    Serial.println(" %");    

    delay(1000);*/

////////////////////////////////////////////////////////////////////////////////

//Gyro

   /* unsigned int data[6];  
// Start I2C Transmission  
Wire.beginTransmission(Addr);  
// Select Gyrometer data register 
Wire.write(0x02);  
// Stop I2C Transmission  
Wire.endTransmission();
 // Request 6 bytes of data  
Wire.requestFrom(Addr, 6);  
// Read 6 bytes of data 
// xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb  
if(Wire.available() == 6)  
{    
data[0] = Wire.read();    
data[1] = Wire.read();    
data[2] = Wire.read();    
data[3] = Wire.read();    
data[4] = Wire.read();    
data[5] = Wire.read();  
}  
delay(300);    
// Convert the data  
int xGyro = ((data[1] * 256) + data[0]);  
int yGyro = ((data[3] * 256) + data[2]);  
int zGyro = ((data[5] * 256) + data[4]);    
// Output data to the serial monitor  
Serial.print("X-Axis of Rotation:  ");  
Serial.println(xGyro);  Serial.print("Y-Axis of Rotation:  ");  
Serial.println(yGyro);  Serial.print("Z-Axis of Rotation:  ");  
Serial.println(zGyro);  
delay(500);*/

/////////////////////////////////////////////////////////////////////////////////////
    ble.waitForEvent();


    uint8_t buf[11];

  if (analog_enabled && k==1) {
    // if analog reading enabled
    // Read and send out
    uint16_t value = analogRead(ANALOG_IN_PIN_X);
    uint16_t value1 = analogRead(ANALOG_IN_PIN_Y);
    uint16_t value2 = analogRead(ANALOG_IN_PIN_Z);
    buf[0] = (0x0B);
    buf[1] = (value >> 8);
    buf[2] = (value);
    buf[3] = (0x0B);
    buf[6] = (0x0B);
    buf[4] = (value1 >> 8);
    buf[5] = (value1);
    buf[9] = (0x0A);
    buf[10] = (0x0B);
    buf[7] = (value2 >> 8);
    buf[8] = (value2);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 9);
  }
  ////////////////////////////////////////////////////////////////////////////////////////

  //Gyro

   if (analog_enabled && k==3) {


    //Serial.print("Inside Gyro");
    // if analog reading enabled
    // Read and send out

        unsigned int data[6];  
// Start I2C Transmission  
Wire.beginTransmission(Addr);  
// Select Gyrometer data register 
Wire.write(0x02);  
// Stop I2C Transmission  
Wire.endTransmission();
 // Request 6 bytes of data  
Wire.requestFrom(Addr, 6);  
// Read 6 bytes of data 
// xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb  
if(Wire.available() == 6)  
{    
data[0] = Wire.read();    
data[1] = Wire.read();    
data[2] = Wire.read();    
data[3] = Wire.read();    
data[4] = Wire.read();    
data[5] = Wire.read();  
}  
delay(300);    
// Convert the data  
int xGyro = ((data[1] * 256) + data[0]);  
int yGyro = ((data[3] * 256) + data[2]);  
int zGyro = ((data[5] * 256) + data[4]);    
// Output data to the serial monitor  
/*Serial.print("X-Axis of Rotation:  ");  
Serial.println(xGyro);  Serial.print("Y-Axis of Rotation:  ");  
Serial.println(yGyro);  Serial.print("Z-Axis of Rotation:  ");  
Serial.println(zGyro);*/
   // uint16_t value = analogRead(ANALOG_IN_PIN_X);
    //uint16_t value1 = analogRead(ANALOG_IN_PIN_Y);
    //uint16_t value2 = analogRead(ANALOG_IN_PIN_Z);
    buf[0] = (0x0B);
    buf[1] = (xGyro >> 8);
    buf[2] = (xGyro);
    buf[3] = (0x0B);
    buf[6] = (0x0B);
    buf[4] = (yGyro >> 8);
    buf[5] = (yGyro);
    buf[9] = (0x0B);
    buf[10] = (0x0B);
    buf[7] = (zGyro >> 8);
    buf[8] = (zGyro);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 9);
  }

  /////////////////////////////////////////////////////////////////////////////////////////



  ///////////////////////////////////////////////////////////////////////////////////////////////
  //BME
  if (analog_enabled && k==5) {


    //Serial.print("Inside Environment");
    // if analog reading enabled
    // Read and send out
   // uint16_t value = analogRead(ANALOG_IN_PIN_X);
    //uint16_t value1 = analogRead(ANALOG_IN_PIN_Y);
    //uint16_t value2 = analogRead(ANALOG_IN_PIN_Z);

    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;

    readData();

    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;


    uint16_t hum_act_new= (uint16_t)hum_act;
    uint16_t press_act_new= (uint16_t)press_act;
    uint16_t temp_act_new= (uint16_t)temp_act;
   /* Serial.print("TEMP : ");
    Serial.print(temp_act_new);
    Serial.print(" DegC  PRESS : ");
    Serial.print(press_act_new);
    Serial.print(" hPa  HUM : ");
    Serial.print(hum_act_new);
    Serial.println(" %");   */
    buf[0] = (0x0B);
    buf[1] = (hum_act_new >> 8);
    buf[2] = (hum_act_new);
    buf[3] = (0x0B);
    buf[6] = (0x0B);
    buf[4] = (press_act_new >> 8);
    buf[5] = (press_act_new);
    buf[9] = (0x0B);
    buf[10] = (0x0B);
    buf[7] = (temp_act_new >> 8);
    buf[8] = (temp_act_new);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 9);
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////////////

  //MIC

  if (analog_enabled && k==7) {
    // if analog reading enabled
    // Read and send out
    int value = analogRead(SOUND_IN_PIN);
    //uint16_t value1 = analogRead(ANALOG_IN_PIN_Y);
    //uint16_t value2 = analogRead(ANALOG_IN_PIN_Z);
    buf[0] = (0x0B);
    buf[1] = (value >> 8);
    buf[2] = (value);
    /*buf[3] = (0x0A);
    buf[4] = (0x0B);
    buf[5] = (value1 >> 8);
    buf[6] = (value1);
    buf[7] = (0x0A);
    buf[8] = (0x0B);
    buf[9] = (value2 >> 8);
    buf[10] = (value2);*/
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, 3);
  }

   // Serial.println(k);
}
void readTrim()
{
    uint8_t data[32],i=0;                      // Fix 2014/04/06
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);       // Fix 2014/04/06
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }

    Wire.beginTransmission(BME280_ADDRESS);    // Add 2014/04/06
    Wire.write(0xA1);                          // Add 2014/04/06
    Wire.endTransmission();                    // Add 2014/04/06
    Wire.requestFrom(BME280_ADDRESS,1);        // Add 2014/04/06
    data[i] = Wire.read();                     // Add 2014/04/06
    i++;                                       // Add 2014/04/06

    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);        // Fix 2014/04/06
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F); // Fix 2014/04/06
    dig_H6 = data[31];                                   // Fix 2014/04/06
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}


void readData()
{
    //Serial.println("Inside read");
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    //Serial.println("After begin");
    Wire.write(0xF7);
    //Serial.println("After write");
    Wire.endTransmission();
    //Serial.println("After end");
    Wire.requestFrom(BME280_ADDRESS,8);
    //Serial.println("After request");
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
        //Serial.println("Inside while of read");
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}
