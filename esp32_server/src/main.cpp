/* Libraries and includes */
#include <MPU6050.h>
#include <I2Cdev.h>
#include <WiFi.h>
/* Libraries and includes */

/* Function prototypes */
void raw_data_processing(struct raw_data *input_data, struct processed_data *output_data);
uint8_t gyro_calibration(struct raw_data *input_data);
uint8_t pmu6050_init();
uint8_t get_data(MPU6050 *sensor, raw_data *sensor_raw_data, processed_data *sensor_processed_data);
uint8_t data_sender(struct processed_data *to_send);
/* Function prototypes */

/* Constants and magic nums */
const char* ssid = "yourssid";                                              // Your network SSID                                                    
const char* password =  "yournetworkpass";                                  // Your network password
const uint16_t frame_delay = 2000;                                          // Delay between mesurments
const float acc_sens = 16384.0;                                             // MPU6050 accelerometer sensitivity
const float gyro_sens = 131.0;                                              // MPU6050 gyroscope sensitivity
const uint16_t calibration_raws = 200;                                      // Num of calibration measurments
/* Constants and magic nums */

/* Structures and objects */
struct raw_data                                                             // To store raw data from MPU6050
{   
   int16_t ax;           ///////////////////////////////
   int16_t ay;           // Accelerometer measurments //
   int16_t az;           ///////////////////////////////
   int16_t gx;           ///////////////////////////////
   int16_t gy;           //   Gyroscope measurments   //
   int16_t gz;           ///////////////////////////////
   float gx0;            ///////////////////////////////
   float gy0;            //   Zero drift correction   //
   float gz0;            ///////////////////////////////
} origin_raw;

struct processed_data                                                        // To store procesed data from MPU6050
{   
   float ax;              ///////////////////////////////
   float ay;              //      Acceleration         //
   float az;              ///////////////////////////////
   float gx;              ///////////////////////////////
   float gy;              //        Angle speed        //
   float gz;              ///////////////////////////////
   float angle_gx;        ///////////////////////////////                     TODO: There still zero drift. Need to fix 
   float angle_gy;        //      Current angle        //
   float angle_gz;        ///////////////////////////////
} origin_processed;

WiFiServer controllerServer(54000);                                           // Create a server object. 54000 - listening port
MPU6050 origin;                                                               // Create class for MPU6050 sensor
WiFiClient user;                                                              // Create a client object
/* Structures and objects */


void setup(void) 
{

    Serial.begin(115200);                                                     // UART init
    WiFi.begin(ssid, password);                                               // WiFi init
    while (WiFi.status() != WL_CONNECTED)                                     // Wait for wifi connection
    {
      delay(1000);                                                            
      Serial.println("Connecting to WiFi..");
    }
    Serial.print("Connected to the WiFi network ");                           // 
    Serial.print(ssid);                                                       // Send connection information to uart
    Serial.print(" on IP ");                                                  //
    Serial.println(WiFi.localIP());                                           //
    controllerServer.begin();                                                 // Server init
    Wire.begin();                                                             // I2C bus init
    //origin.setSleepEnabled(true);                                           // Set MPU6050 to sleep mode // Dissable: cause calibration errors

}

void loop() 
{  

    user = controllerServer.available();                                      // Bind server with client if any

    if(user)                                                                  // If user connected
    {
      Serial.print("User connected from IP ");                                //
      Serial.print(user.remoteIP());                                          // Send connection information to UART
      Serial.print(" on port ");                                              //
      Serial.println(user.remotePort());                                      //
      Serial.println("Dissabling serial connection");
      delay(1000);
      Serial.end();                                                           // Stop UART. All data will be send over wifi                             
      pmu6050_init();                                                         // MPU6050 init. No deinit currently
                                                                              // TODO: add mpu6050_deinit
      while (user.connected())                                                // While user avaliable
      {
        get_data(&origin, &origin_raw, &origin_processed);                    // Get raw data and process it
        data_sender(&origin_processed);                                       // Send data to user
        delayMicroseconds(frame_delay);                                       // Wait time between measurements
      };
      user.stop();                                                            // When user disconected
      Serial.begin(115200);                                                   // Start UARt connection again
      Serial.println("User disconnected");
    } else 
    {
      Serial.println("Waiting for user");
      delay(1000);
    }

}

uint8_t pmu6050_init()                                                        // MPU6050 initialization function
{
    origin.initialize();                                                      // Set base settings and configure PWR_MGMT_1 register
    origin.setFullScaleGyroRange(MPU6050_GYRO_FS_250);                        // Set gyroscope sensitivity
    origin.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);                        // Set accelerometer sensitivity
    origin.setDLPFMode(MPU6050_DLPF_BW_20);                                   // Set noise filter
    user.println("MPU6050 initalized");

    // NOTE: There is a calibration function in MPU6050 library
    // TODO: Improve calibration function or replace with CalibrateGyro in MPU6050.cpp (3285 line of code) 
    gyro_calibration(&origin_raw);                                            // Calibrate gyscope to minimize zero drift
      
    user.println("Calibration done");
    return(1);
}

uint8_t get_data(MPU6050 *sensor, raw_data *sensor_raw_data, processed_data *sensor_processed_data)     // Function to receive raw data and process it.
// It takes pointers to structures, so can be used to upscale project with more sensors.
{
    sensor->getMotion6(&sensor_raw_data->ax, &sensor_raw_data->ay, &sensor_raw_data->az, &sensor_raw_data->gx, &sensor_raw_data->gy, &sensor_raw_data->gz);
    // Receive raw data from sensor and store it
    raw_data_processing(sensor_raw_data, sensor_processed_data); // Process raw data
    return(1);
}

uint8_t data_sender(struct processed_data *to_send)              // Send all data to user client. NOTE: data move toooooo fast. 
                                                                 // TODO: Write windows client with GUI or add web interface
{
    user.print("Acceleration axis x: ");
    user.print(to_send->ax);
    user.println(" m/s2");
    user.print("Acceleration axis y: ");
    user.print(to_send->ay);
    user.println(" m/s2");
    user.print("Acceleration axis z: ");
    user.print(to_send->az);
    user.println(" m/s2");
    user.print("Angle speed axis x: ");
    user.print(to_send->gx);
    user.println(" deg/sec");
    user.print("Angle speed axis y: ");
    user.print(to_send->gy);
    user.println(" deg/sec");
    user.print("Angle speed axis z: ");
    user.print(to_send->gz);
    user.println(" deg/sec");
    user.print("Rotation axis x:");
    user.print(to_send->angle_gx);
    user.println(" degrees");
    user.print("Rotation axis y:");
    user.print(to_send->angle_gy);
    user.println(" degrees");
    user.print("Rotation axis z:");
    user.print(to_send->angle_gz);
    user.println(" degrees");
    user.println(' ');
    return(1);
}

void raw_data_processing(struct raw_data *input_data, struct processed_data *output_data)
// Raw data processor. Takes pointers to two structures, raw data as input and processed data as output.
// Main objectives: convert measured data to SI and calculate rotation
{
    output_data->ax = input_data->ax / acc_sens;          //////////////////
    output_data->ay = input_data->ay / acc_sens;          // Acceleration //
    output_data->az = input_data->az / acc_sens;          //////////////////

    output_data->gx = (input_data->gx - input_data->gx0) / gyro_sens;         /////////////////
    output_data->gy = (input_data->gy - input_data->gy0) / gyro_sens;         // Angle speed //
    output_data->gz = (input_data->gz - input_data->gz0) / gyro_sens;         /////////////////

    output_data->angle_gx = output_data->angle_gx + output_data->gx * frame_delay / 1000000.0;          // Calculate rotation by incrementing
    output_data->angle_gy = output_data->angle_gy + output_data->gy * frame_delay / 1000000.0;          // angle speed in time
    output_data->angle_gz = output_data->angle_gz + output_data->gz * frame_delay / 1000000.0;          // 1000000 - to milliseconds
    // NOTE: Collects errors with time. Еakes the initial position as zero
    // TODO: Add angle calculation from accelerometer data
    // TODO: Increase accuracy by mixing two angle data
}

uint8_t gyro_calibration(struct raw_data *input_data)           // Calculates gyroscope zero drift.
// Copy of comment from line 118
// NOTE: There is a calibration function in MPU6050 library
// TODO: Improve calibration function or replace with CalibrateGyro in MPU6050.cpp (3285 line of code) 
{
    for(uint16_t raw = 0; raw < calibration_raws; raw++)
    {
      
      origin.getMotion6(&input_data->ax, &input_data->ay, &input_data->az, &input_data->gx, &input_data->gy, &input_data->gz);
      input_data->gx0 += input_data->gx;          ///////////////////////////////////////
      input_data->gy0 += input_data->gy;          // Collect angle speed in idle state //
      input_data->gz0 += input_data->gz;          ///////////////////////////////////////

    }

    input_data->gx0 /= calibration_raws;          //////////////////////////////////////////////////////
    input_data->gy0 /= calibration_raws;          // Divide measured value to num of calibration raws //
    input_data->gz0 /= calibration_raws;          //////////////////////////////////////////////////////

    return(1);
}