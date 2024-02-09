/* Basic Multi Threading Arduino Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// Please read file README.md in the folder containing this example.

// #include <Arduino.h>
#include <WiFi.h>
#include "thingProperties.h"
#include<ADS1115_WE.h> 
#include <Wire.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define ANALOG_INPUT_PIN A0

#define LED_R 14
#define LED_G 15
#define LED_B 16

#define SERIAL_PRINT_RATE 115200

#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

int buf_iot_downlinker_len = 0;
int buf_iot_downlinker[10] = {0};


// Define two tasks for Blink & AnalogRead.
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskWiFiConnect( void *pvParameters );
void TaskWiFiCheckConn( void *pvParameters );
void TaskManageCloud( void *pvParameters );
void TaskDownlinkManager( void *pvParameters );
TaskHandle_t analog_read_task_handle; // You can (don't have to) use this to be able to manipulate a task from somewhere else.

QueueHandle_t iot_dwn_queue;
QueueHandle_t adc_dat_queue;
const int iot_dwn_maxlen = 10;
const int adc_dat_queue_maxlen = 1000;

typedef struct {
  int t_millis;
  float v_ch_0;
  float v_ch_1;
  float v_ch_d01;
} vin_read_t;


void printWifiStatus();
float get_tick_sec();
bool onOTARequestCallback();
float readChannel(ADS1115_MUX channel);

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(SERIAL_PRINT_RATE);

  for (int timened = millis() + 10000, j; millis() < timened ; j++)
  {
    Serial.printf( "[%i] 'booting'...",  millis());

    for (int i = j; i > 0; i--)
    {
      Serial.print(".");
    }
    Serial.print(" \n\r");
    
    delay(500);
  }


  
  Serial.printf("\n\rTASKS INITIALIZING\n\r");
  Serial.printf("TASKS INITIALIZING\n\r");
  Serial.printf("TASKS INITIALIZING\n\r");

  Serial.end();

  // vTaskSuspendAll();
  
  Serial.begin(SERIAL_PRINT_RATE);

  Serial.printf("\n\r### TASK ACQDAT START ###\n\r");

  // Set up two tasks to run independently.
  uint32_t ACQDAT_delay = 250; // Delay between changing state on LED pin
  xTaskCreate(
    TaskACQDAT
    ,  "Task ACQDAT" // A name just for humans
    ,  1<<15        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  (void*) &ACQDAT_delay // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  4  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    );

  Serial.printf("\n\r### TASK SERIAL SEND START ###\n\r");

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreate(
    TaskSerialSend
    ,  "Serial Send"
    ,  1<<14  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  10  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    );

  // // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  // xTaskCreatePinnedToCore(
  //   TaskAnalogRead
  //   ,  "Analog Read"
  //   ,  2048  // Stack size
  //   ,  NULL  // When no parameter is used, simply pass NULL
  //   ,  10  // Priority
  //   ,  &analog_read_task_handle // With task handle we will be able to manipulate with this task.
  //   ,  ARDUINO_RUNNING_CORE // Core on which the task will run
  //   );

    
  Serial.printf("\n\r### TASK WIFIMANAGER START ###\n\r");

  xTaskCreate(
    TaskWiFiConnect
    ,  "Task WiFi Connect" // A name just for humans
    ,  32768        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  5  // Priority ()
    ,  NULL // Task handle is not used here - simply pass NULL
    );


  Serial.printf("\n\r### TASK CLOUD START ###\n\r");
    
  xTaskCreate(
    TaskManageCloud
    ,  "Task manage arduino cloud" // A name just for humans
    ,  32768        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  5  // Priority ()
    ,  NULL // Task handle is not used here - simply pass NULL
    );
        
  Serial.printf("\n\r### QUEUE adc_dat CREATING ###\n\r");

  adc_dat_queue = xQueueCreate(adc_dat_queue_maxlen, sizeof(vin_read_t));
  // Check if the queue was successfully created
  if(adc_dat_queue == NULL){
    Serial.println("adc_dat_queue could not be created. Halt.");
    while(1) delay(1000); // Halt at this point as is not possible to continue
  }
        
  Serial.printf("\n\r### QUEUE DWN CREATING ###\n\r");

  iot_dwn_queue = xQueueCreate(iot_dwn_maxlen, sizeof(int));
  // Check if the queue was successfully created
  if(iot_dwn_queue == NULL){
    Serial.println("iot_dwn_queue could not be created. Halt.");
    while(1) delay(1000); // Halt at this point as is not possible to continue
  }

  Serial.printf("\n\r### TASK DOWNLINKER START ###\n\r");

  xTaskCreate(
    TaskDownlinkManager
    ,  "Task do stuff with downlinks" // A name just for humans
    ,  8192        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  5  // Priority ()
    ,  NULL // Task handle is not used here - simply pass NULL
    );
  

  Serial.printf("\n\r### SUCCESSFUL LAUNCH 🚀🚀🚀 ###\n\r");
  Serial.printf("\n\r### SUCCESSFUL LAUNCH 🚀🚀🚀 ###\n\r");
  // Serial.printf("\n\r### SUCCESSFUL LAUNCH ###\n\r");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.


  // Crashes the system??
  // xTaskResumeAll();
  
}

void loop(){

  delay(10);

  // Here acts as idle task 

  // Delete the task after 10 seconds 
  // if(analog_read_task_handle != NULL){ // Make sure that the task actually exists
  //   delay(10000);
  //   vTaskDelete(analog_read_task_handle); // Delete task
  //   analog_read_task_handle = NULL; // prevent calling vTaskDelete on non-existing task
  // }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskACQDAT(void *pvParameters){  // This is a task.
  uint32_t blink_delay = *((uint32_t*)pvParameters);


  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  Wire.begin();

  // ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
  while (!adc.init())
  {
    Serial.println("ADS1115 not connected!");
    vTaskDelay(1000);
  }

  // see for settings:    https://github.com/wollewald/ADS1115_WE/blob/master/examples/Single_Shot/Single_Shot.ino
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range
  // adc.setCompareChannels(ADS1115_COMP_0_GND); //uncomment if you want to change the default
  // adc.setConvRate(ADS1115_860_SPS ); //uncomment if you want to change the default
  adc.setConvRate(ADS1115_475_SPS); //uncomment if you want to change the default
  // adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default


  for (int count = 0 ; ; count++)
  {
    vin_read_t vs = {0};
    vs.t_millis = millis();
    // vs.v_ch_0 = readChannel(ADS1115_COMP_0_GND);
    // vs.v_ch_1 = readChannel(ADS1115_COMP_1_GND);
    vs.v_ch_d01 = readChannel(ADS1115_COMP_0_1);

    xQueueSend(adc_dat_queue, (void*) &vs, 0);

    // LEDs are Active Low!! 
    // https://docs.arduino.cc/tutorials/nano-esp32/cheat-sheet#:~:text=These%20pins%20are%20so,%2C%20like%20this%3A
    digitalWrite(LED_R, count % 4 == 0);
    digitalWrite(LED_G, count % 4 == 1);
    digitalWrite(LED_B, count % 4 == 2);

  }
}

#define AVG_BUCKET 1024
#define RESISTOR 150.0
#define CT_RATIO (100.0 / 0.050)
void TaskSerialSend(void *pvParameters){  // This is a task.

  vin_read_t vr = {0};
  
  float v_calc[AVG_BUCKET] = {0};
  float v_diff[AVG_BUCKET] = {0};
  int t_calc[AVG_BUCKET] = {0};
  int i = 0;
  int t_last = 0;

  for (;;)
  {

    if(adc_dat_queue != NULL)
    { 
      if (xQueueReceive(adc_dat_queue, (void*) &vr, 0) == pdTRUE)
      {
        t_calc[i] = vr.t_millis;
        // adcread = vr.v_ch_1 - vr.v_ch_0;
        // v_calc[i] = vr.v_ch_1 - vr.v_ch_0;
        // v_calc[i] = vr.v_ch_d01 / 2.0;
        v_calc[i] = vr.v_ch_d01;
        i = ++i % AVG_BUCKET;

        float sum = 0;
        float square = 0;
        float max = 0;
        int t_min = t_calc[i];
        for (int j = 0; j < AVG_BUCKET; j++)
        {
          sum += abs(v_calc[j]);
          square += v_calc[j] * v_calc[j];
          t_min = t_calc[j] < t_min ? t_calc[j] : t_min ;
          // max = abs(vr.v_ch_1) > max ? abs(vr.v_ch_1) : max;
          // max = abs(vr.v_ch_0) > max ? abs(vr.v_ch_0) : max;
          max = abs(vr.v_ch_d01) > max ? abs(vr.v_ch_d01) : max;
        }
        
        adc_avg = sum / AVG_BUCKET;
        adc_max = max; 
        // square /= (float)(t_calc[i] - t_calc[(i + 1) % AVG_BUCKET]);
        // square /= (float)(t_calc[i] - t_min);
        square /= (float)(AVG_BUCKET);
        adc_rms = sqrt(square);
        adcread = v_calc[i];
        adc_i_rms = adc_rms * CT_RATIO / RESISTOR;
        adc_i_avg = adc_avg * CT_RATIO / RESISTOR;
        Serial.print(" t::");
        Serial.print(vr.t_millis);
        Serial.print(" \tnow::");
        Serial.print(v_calc[i]);
        Serial.print(" \tavg::");
        Serial.print(adc_avg);
        Serial.print(" \tiavg::");
        Serial.print(adc_i_avg);
        Serial.print(" \tirms::");
        Serial.print(adc_i_rms);
        Serial.print(" \tmax:");
        Serial.println(adc_max);


        t_last = vr.t_millis;
        // vTaskDelay(1);
      }
    }
  
    vTaskDelay(1);

  }
}

void TaskWiFiConnect(void *pvParameters)
{  // This is a task.


  vTaskDelay(1000);

  for (;;)
  {
    printWifiStatus();

    vTaskDelay(5000);
  }

}

void TaskManageCloud( void *pvParameters)
{

  vTaskDelay(5000);
  
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  while (  !ArduinoCloud.begin(ArduinoIoTPreferredConnection)  )
  {
    Serial.printf("\r[%f] In connection progress...\n", get_tick_sec());
    vTaskDelay(1000);
  }

  /* Setup OTA callback */
  ArduinoCloud.onOTARequestCb(onOTARequestCallback);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();


  for (;;)
  {
    ArduinoCloud.update();
    vTaskDelay(500);
  }
}

void TaskDownlinkManager( void *pvParameters)
{

  int downlinked = 0;

  for (;;)
  {
    
    xQueueReceive(iot_dwn_queue, &downlinked, portMAX_DELAY );

    Serial.printf("[%f] DOWNLINKED: %i \n\r", get_tick_sec(), downlinked);
  }
}


void printWifiStatus() 
{
  
  // print the SSID of the network you're attached to:
  // char buffer[100];
  
  Serial.printf("\r[%f] SSID: ", get_tick_sec());
  Serial.print(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("\tIP Address: ");
  Serial.print(ip);

  // print the received signal strength:
  long rssi_l = WiFi.RSSI();
  rssi = rssi_l;
  Serial.print("\t (RSSI):");
  Serial.print(rssi);
  Serial.print(" dBm\n\r");

}




/*
  Since Downlinker is READ_WRITE variable, onDownlinkerChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onDownlinkerChange()  
{
  // Add your code here to act upon Downlinker change

  xQueueSend(iot_dwn_queue, (void *)&downlinker, 0);

  downlink_count = downlink_count != NULL ? downlink_count + 1: 1;

  downlinker = 0;
}

float get_tick_sec()
{
  // return (float)xTaskGetTickCount() / xPortGetTickRateHz();
  return millis() / 1000.0;
}

bool onOTARequestCallback()
{
  return true;
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}


/*
  Since Adcread is READ_WRITE variable, onAdcreadChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onAdcreadChange()  {
  // Add your code here to act upon Adcread change
}