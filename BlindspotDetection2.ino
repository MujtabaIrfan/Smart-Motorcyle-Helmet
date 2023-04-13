#include <Arduino.h>     // every sketch needs this
#include <Wire.h>        // instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.2.0

TFLI2C tflI2C;
TFLI2C tflI2C2;

//right lidar 
int16_t tfDist2; 
int16_t tfAddr2 = 0x70;
const int LIST_SIZE2 = 20;
const int THRESHOLD2 = 1; 
int dist_list2[LIST_SIZE2];
int list_index2 = 0; 


int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // use this default I2C address or
                                // set variable to your own value

const int LIST_SIZE = 20;
const int THRESHOLD = 1;
//min and max values taken from standard width of a highway lane. 
//SUBJECT TO CHANGE UPON FURTHER TESTING
const int DIST_MINIMUM = 50; //minumum distance to be detected
const int DIST_MAXIMUM = 560; //maximum distance


int dist_list[LIST_SIZE];
int list_index = 0;

//variable for the RED LED 
#define RED_LED 4
#define RED_LED2 2

                               

void setup()
{
    Serial.begin(230400);  // initialize serial port
    Wire.begin();           // initialize Wire library
   
    pinMode(RED_LED, OUTPUT);       
    pinMode(RED_LED2, OUTPUT); 
}

void loop()
{
    int blidar1 = left_blindspot(); 
    int blidar2 = right_blindspot();

}

int left_blindspot()
{
  if( tflI2C.getData( tfDist, tfAddr)) // If read okay...
      {
          //Serial.print("Dist: ");
          //Serial.println(tfDist);          // print the data...
      }
      else tflI2C.printStatus();           // else, print error.


      if (tfDist >= DIST_MINIMUM && tfDist <= DIST_MAXIMUM ){

        dist_list[list_index] = tfDist;

        // check if the list is full
        if (list_index == LIST_SIZE - 1) {
          // compare the values in the list with the assigned threshold value 
          bool isAlert = true;
          for (int i = 0; i < LIST_SIZE - 1; i++) {
            int diff = abs(dist_list[i] - dist_list[i+1]);
            if (diff > THRESHOLD) {
              isAlert = false;
              //Serial.println(tfDist);
              break;
            }
          }

          // print alert if the values are closely related
          if (isAlert) {
            digitalWrite(RED_LED, HIGH);
            Serial.println("LEFT BLINDSPOT ACTIVATED!");
            Serial.println(tfDist);
            //delay(1000);
            //digitalWrite(RED_LED, LOW);
          }

          // shift the list to the left to make room for a new value
          for (int i = 0; i < LIST_SIZE - 1; i++) {
            dist_list[i] = dist_list[i+1];
          }
        } else {
          // increment the index if the list is not yet full
          list_index++;
        }
        

      }
      else digitalWrite(RED_LED, LOW);

    
}

int right_blindspot()
{
  if( tflI2C2.getData( tfDist2, tfAddr2)) // If read okay...
      {
          //Serial.print("Dist: ");
          //Serial.println(tfDist);          // print the data...
      }
      else tflI2C2.printStatus();           // else, print error.


      if (tfDist2 >= DIST_MINIMUM && tfDist2 <= DIST_MAXIMUM ){

        dist_list2[list_index2] = tfDist2;

        // check if the list is full
        if (list_index2 == LIST_SIZE2 - 1) {
          // compare the values in the list with the assigned threshold value 
          bool isAlert = true;
          for (int i = 0; i < LIST_SIZE2 - 1; i++) {
            int diff = abs(dist_list2[i] - dist_list2[i+1]);
            if (diff > THRESHOLD) {
              isAlert = false;
              //Serial.println(tfDist);
              break;
            }
          }

          // print alert if the values are closely related
          if (isAlert) {
            //int ALERT = activate_alert2();
            digitalWrite(RED_LED2, HIGH);
            Serial.println("RIGHT BLINDSPOT ACTIVATED!");
            Serial.println(tfDist2);
            //delay(1000);
            //digitalWrite(RED_LED2, LOW);

          }

          // shift the list to the left to make room for a new value
          for (int i = 0; i < LIST_SIZE2 - 1; i++) {
            dist_list2[i] = dist_list2[i+1];
          }
        } else {
          // increment the index if the list is not yet full
          list_index2++;
        }

      } else digitalWrite(RED_LED2, LOW);

}