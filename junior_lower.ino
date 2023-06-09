#include "farduino_constants.h"
// Must come after farduino_constants.h !
#include "rtttl_songs.h"
#include "farduino_types.h"
#include "farduino_utilities.h"
#include "ring_buffer.h"

#include "junior-rocket-state.hpp"
#include "state-reactions.hpp"

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#ifdef USE_SD_CARD
#define USE_STANDARD_SPI_LIBRARY 2  // See SdFatConfig.h
#include <SPI.h>
#include <SdFat.h>
#endif

#include <RF24.h>

//#define farduino_maple_v1 1
#define farduino_maple_v2 1
//#define farduino_maple_v3 1



#define isdigit(n) (n >= '0' && n <= '9')

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low  = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

char my_name[16];
char my_line[64];
char response[64];

unsigned int sample_count = 0;
unsigned int file_count = 0;

double temperature;
double pressure;
double altitude;

double raw_acc[3];
double raw_omega[3];
double raw_B[3];

double acc[3];
double omega[3];
double omega_0[3] = { 0.0, 0.0, 0.0 };
double B[3];

double norm_acc;
double norm_omega;

unsigned char GPS_checksum = 0;
byte unsigned GPS_pointer = 0;
bool receiving_GPS;
char GPS_sentence[128];

//the pressure sensor is necessary
Adafruit_BMP280 met;

MPU9250 imu_mpu9250;
bool mpu9250_present = false;

Adafruit_BNO055 imu_bno055 = Adafruit_BNO055(-1, 0x28);
bool bno055_present = false;

byte flight_address[6] = "LOWER";
byte ground_address[6] = "GRND1";

char request[32] = "abcdefghijklmnopqrstuvwxyz01234";

CircularBuffer ring(256);
RF24 radio_nrf24(NRF24_CE_PIN, NRF24_CS_PIN);

bool nrf24l01_present = false;


#ifdef USE_SD_CARD
//SD constants and variables
const uint8_t chipSelect = PB12;
SPIClass spi2;
SdFat sd(&spi2);

bool SD_present = false;
SdFile dataFile;
bool file_exists;
#endif

StateReactions state_reactions(radio_nrf24);
far::junior::JuniorRocketState state_machine(state_reactions);

void setup() {

  double altitude;
  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  double sigma_w;

  unsigned long mean_count;

  inertial_measurement_t imu_data;

  //set all pyro pins as outputs and inactive
  pinMode(PYRO0, OUTPUT);
  digitalWrite(PYRO0, LOW);
  pinMode(PYRO1, OUTPUT);
  digitalWrite(PYRO1, LOW);
  pinMode(PYRO2, OUTPUT);
  digitalWrite(PYRO2, LOW);
  pinMode(PYRO3, OUTPUT);
  digitalWrite(PYRO3, LOW);

  //join I2C bus
  Wire.begin();
  Wire.setClock(100000);

  //open USB com port
  #ifndef RASPBERRYPI_PICO
  Serial.begin(115200);
  #else
  Serial.begin(115200);
  #endif

  //serial port #1 set to 9600 8n1 for GPS data stream
  Serial1.begin(9600);

  delay(2000);
  Serial.println(F("FARduino Maple v0.2"));


  nrf24l01_present = radio_nrf24.begin();
  if (nrf24l01_present) {
    Serial.print(F("Initializing nrf24L01+..."));

    radio_nrf24.setPayloadSize(32);
    radio_nrf24.setAutoAck(true);
    radio_nrf24.setPALevel(RF24_PA_LOW);
    radio_nrf24.openWritingPipe(ground_address);
    radio_nrf24.openReadingPipe(1, flight_address);
    radio_nrf24.startListening();

    Serial.println(F("done."));
  } else {
    Serial.println(F("<!> nrf24L01+ not detected"));
  }

  //initialize I2C devices

  mpu9250_present = imu_mpu9250.testConnection();
  if (mpu9250_present) {
    Serial.print(F("Initializing MPU9250..."));
    imu_mpu9250.initialize();
    //set gyroscope scale to +/-2000°/s
    imu_mpu9250.setFullScaleGyroRange(MPU9250_GYRO_FS_2000);
    //set acceleration scale to +/-16g
    imu_mpu9250.setFullScaleAccelRange(MPU9250_ACCEL_FS_16);
    Serial.println(F("done."));
  } else {
    Serial.println("<!> MPU9250 not detected");
  }

  bno055_present = imu_bno055.begin();
  if (bno055_present) {
    Serial.print(F("Initializing BNO055..."));
    imu_bno055.setExtCrystalUse(true);
    Serial.println("ready.");
  } else {
    Serial.println("<!> BNO055 not detected");
  }

  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  #ifdef USE_SD_CARD
  SD_present = sd.begin(chipSelect, SPI_FULL_SPEED);
  if (!SD_present) {
    Serial.println("<!> no SD card found");
  } else {
    Serial.println("SD card initialized");
    //search for next free file dataXXXX.txt
    do {
      sprintf(my_name, "data%04d.txt", file_count++);
      Serial.print("checking ");
      Serial.println(my_name);

      file_exists = !dataFile.open(my_name, O_CREAT | O_WRITE | O_EXCL);
      if (!file_exists) {
        Serial.print("file ");
        Serial.print(my_name);
        Serial.println(" created");
      }
    } while (file_exists);
  }
  #endif
  

  if (!met.begin(0x76)) {
    Serial.println(F("<!> BMP280 not detected"));
    //halt program if no pressure sensor is detected
    exit(-1);
  }
  
  //indicate readiness for operator
  play_rtttl(never_song);

}



//central LOOP

void loop() {

  char sentence[100];

  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  unsigned long flight_time;
  unsigned long mean_count;
  unsigned long delta_time;
  double delta_pressure;

#ifdef farduino_maple_v1
  if (mpu9250_present) {
    get_mpu9250_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    
    acc[0] = raw_acc[0]/ONE_G;
    acc[1] = raw_acc[1]/ONE_G;
    acc[2] = raw_acc[2]/ONE_G;
  
    //calculate norm of acceleration vector in g
    norm_acc = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  
    omega[0] = raw_omega[0]/ONE_DEG_PER_SECOND;
    omega[1] = raw_omega[1]/ONE_DEG_PER_SECOND;
    omega[2] = raw_omega[2]/ONE_DEG_PER_SECOND;
  
    norm_omega = sqrt(raw_omega[0] * raw_omega[0] + raw_omega[1] * raw_omega[1] + raw_omega[2] * raw_omega[2]);
  
    construct_IMU_sentence(imu_timestamp, acc, omega, raw_B, &sentence[0]);
    send_sentence_to_all(&sentence[0]);
  }
#elsif
  if (bno055_present) {
    get_bno055_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    
    acc[0] = raw_acc[0]/ONE_G;
    acc[1] = raw_acc[1]/ONE_G;
    acc[2] = raw_acc[2]/ONE_G;
  
    //calculate norm of acceleration vector in g
    norm_acc = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  
    omega[0] = raw_omega[0]/ONE_DEG_PER_SECOND;
    omega[1] = raw_omega[1]/ONE_DEG_PER_SECOND;
    omega[2] = raw_omega[2]/ONE_DEG_PER_SECOND;
  
    norm_omega = sqrt(raw_omega[0] * raw_omega[0] + raw_omega[1] * raw_omega[1] + raw_omega[2] * raw_omega[2]);
    
    construct_IMU_sentence(imu_timestamp, acc, omega, raw_B, &sentence[0]);
    send_sentence_to_all(&sentence[0]);
  }
#endif

  
  get_MET_data(met_timestamp, temperature, pressure, altitude);

  construct_MET_sentence(met_timestamp, pressure, temperature, altitude, &sentence[0]);
  send_sentence_to_all(&sentence[0]);
  
  bool gps_available = get_GPS_data();
  if (gps_available) {
    send_sentence_to_all(&GPS_sentence[0]);
  }


  #ifdef USE_SD_CARD
  sample_count++;

  if (SD_present) {
    // Force data to SD and update the directory entry to avoid data loss.
    if (!dataFile.sync() || dataFile.getWriteError()) {
      //sd.errorHalt(F("write error"));
    }

    if (sample_count >= MAX_SAMPLE_COUNT && state_reactions.safe_to_flush_sd_card()) {
      dataFile.close();

      do {
        sprintf(my_name, "data%04d.txt", file_count++);
        Serial.print("creating file ");
        Serial.println(my_name);
        file_exists = !dataFile.open(my_name, O_CREAT | O_WRITE | O_EXCL);
        if (file_exists) {
          Serial.println("file exists.");
        } else {
          Serial.println("new file created");
        }
      } while (file_exists);

      sample_count = 0;
    }
  }
  #endif
  state_machine.drive(imu_timestamp, pressure, norm_acc);
}


void send_sentence(const char* message) {

  ring.write(message, strlen(message));

  radio_nrf24.stopListening();
  while (ring.filling() >= 32) {
    ring.read(&response[0], 32);
    radio_nrf24.writeFast(&response[0], 32);
  }

  radio_nrf24.txStandBy();
  radio_nrf24.startListening();
}


bool get_GPS_data(void) {

  char cipher = 0;
  char first_char;
  char second_char;
  unsigned char received_checksum;

  while (Serial1.available()) {
    cipher = Serial1.read();

    if (cipher == '$') {
      GPS_checksum = 0;
      GPS_pointer = 0;
      receiving_GPS = true;
    }

    GPS_sentence[GPS_pointer] = cipher;
    if (GPS_pointer < 128) {
      GPS_pointer++;
    }

    if (cipher == '*') {
      receiving_GPS = false;
    } else if ((cipher != '$') && receiving_GPS) {
      GPS_checksum ^= cipher;
    } else if (cipher == 0x0A) {
      first_char = GPS_sentence[GPS_pointer - 4];
      second_char = GPS_sentence[GPS_pointer - 3];

      if (first_char > '9') {
        first_char = first_char - 'A' + 10;
      } else {
        first_char = first_char - '0';
      }

      if (second_char > '9') {
        second_char = second_char - 'A' + 10;
      } else {
        second_char = second_char - '0';
      }

      received_checksum = (first_char << 4) + second_char;

      //Serial1.println(received_checksum,HEX);
      //Serial1.println(GPS_checksum,HEX);

      if (GPS_checksum == received_checksum) {
        GPS_sentence[GPS_pointer] = 0;
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}




void get_mpu9250_data(unsigned long& timestamp, double& acc_x, double& acc_y, double& acc_z, double& omega_x, double& omega_y, double& omega_z, double& mag_x, double& mag_y, double& mag_z) {

  int16_t ax, ay, az;
  int16_t wx, wy, wz;
  int16_t Bx, By, Bz;

  timestamp = get_timestamp();
  imu_mpu9250.getMotion9(&ax, &ay, &az, &wx, &wy, &wz, &Bx, &By, &Bz);

  acc_x = (double)ax;
  acc_y = (double)ay;
  acc_z = (double)az;

  omega_x = (double)wx - omega_0[0];
  omega_y = (double)wy - omega_0[1];
  omega_z = (double)wz - omega_0[2];

  mag_x = (double)Bx;
  mag_y = (double)By;
  mag_z = (double)Bz;
}


void get_bno055_data(unsigned long& timestamp, double& acc_x, double& acc_y, double& acc_z, double& omega_x, double& omega_y, double& omega_z, double& mag_x, double& mag_y, double& mag_z) {

  sensors_event_t magneticData, angVelocityData, accelData;

  timestamp = get_timestamp();

  imu_bno055.getEvent(&magneticData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_bno055.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  acc_x = accelData.acceleration.x;
  acc_y = accelData.acceleration.y;
  acc_z = accelData.acceleration.z;

  omega_x = angVelocityData.gyro.x;
  omega_y = angVelocityData.gyro.y;
  omega_z = angVelocityData.gyro.z;

  mag_x = magneticData.magnetic.x;
  mag_y = magneticData.magnetic.y;
  mag_z = magneticData.magnetic.z;
}





void get_MET_data(unsigned long& timestamp, double& temperature, double& pressure, double& altitude) {

  timestamp = get_timestamp();
  temperature = met.readTemperature();
  pressure = met.readPressure() / 100.0;
  if(const auto ground_pressure = state_machine.ground_pressure())
  {
    altitude = met.readAltitude(*ground_pressure);
  }
  else
  {
    altitude = -1.0;
  }
}


void mean_pressure(int n, double& mean_p, double& sigma_p) {

  double sum = 0.0;
  double sum2 = 0.0;

  unsigned long timestamp;
  double temp;
  double p;
  double h;

  for (int k = 0; k < n; k++) {
    get_MET_data(timestamp, temp, p, h);

    sum += p;
    sum2 += p * p;
  }

  mean_p = sum / n;
  sigma_p = sqrt((sum2 - n * mean_p * mean_p) / (n - 1));
}


void mean_inertial(int n, inertial_measurement_t& data) {

  double sum_acc[3] = { 0.0, 0.0, 0.0 };
  double sum_acc2[3] = { 0.0, 0.0, 0.0 };

  double sum_w[3] = { 0.0, 0.0, 0.0 };
  double sum_w2[3] = { 0.0, 0.0, 0.0 };

  double sum_B[3] = { 0.0, 0.0, 0.0 };
  double sum_B2[3] = { 0.0, 0.0, 0.0 };



  unsigned long timestamp;
  double norm_acc[3];
  double norm_omega[3];
  double B[3];

  for (int k = 0; k < n; k++) {
    get_mpu9250_data(timestamp, norm_acc[0], norm_acc[1], norm_acc[2], norm_omega[0], norm_omega[1], norm_omega[2], B[0], B[1], B[2]);

    for (int j = 0; j < 3; j++) {
      sum_acc[j] += norm_acc[j];
      sum_acc2[j] += norm_acc[j] * norm_acc[j];

      sum_w[j] += norm_omega[j];
      sum_w2[j] += norm_omega[j] * norm_omega[j];

      sum_B[j] += B[j];
      sum_B2[j] += B[j] * B[j];
    }
  }


  for (int j = 0; j < 3; j++) {
    data.mean_a[j] = sum_acc[j] / n;
    data.sigma_a[j] = sqrt((sum_acc2[j] - n * data.mean_a[j] * data.mean_a[j]) / (n - 1));

    data.mean_w[j] = sum_w[j] / n;
    data.sigma_w[j] = sqrt((sum_w2[j] - n * data.mean_w[j] * data.mean_w[j]) / (n - 1));

    data.mean_B[j] = sum_B[j] / n;
    data.sigma_B[j] = sqrt((sum_B2[j] - n * data.mean_B[j] * data.mean_B[j]) / (n - 1));
  }
}

void send_sentence_to_all(const char* sentence)
{
    Serial.print(sentence);
    #ifdef USE_SD_CARD
    if (SD_present) {
      dataFile.print(sentence);
    }
    #endif
    if (nrf24l01_present) {
      send_sentence(sentence);
    }
}
