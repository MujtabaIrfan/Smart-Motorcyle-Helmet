
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>  // TFLuna-I2C Library v.0.2.0


const int LIST_SIZE = 10;
const int THRESHOLD = 1;

int dist_list[LIST_SIZE];
int list_index = 0;

//variable for the RED LED 
int RED_LED = 4;

const uint8_t newaddr = 0x70;

TFLI2C tflI2C;

// Use these defaults or insert your own values
const int8_t  tfAddr = TFL_DEF_ADR;    // default I2C address
//uint8_t tfAddr = 0x1A; 
uint16_t tfFrame = TFL_DEF_FPS;   // default frame rate
uint16_t tfAddr1;

// device variables passed back by getData
int16_t  tfDist = 0 ;   // distance in centimeters
int16_t  tfFlux = 0 ;   // signal quality in arbitrary units
int16_t  tfTemp = 0 ;   // temperature in 0.01 degree Celsius

// other device variables
uint16_t tfTime = 0;    // device clock in milliseconds
uint8_t  tfVer[3];      // device version number
uint8_t  tfCode[14];    // device serial number

// sub-loop counter for Time display
uint8_t tfCount = 0;

//  This is a group of various sample
//  commands that can be called at setup.
void sampleCommands( uint8_t adr)
{
    Serial.print( "Device Address: ");
    Serial.println( adr);

    Serial.print("System Reset: ");
    if( tflI2C.Soft_Reset( adr))
    {
        Serial.println( "Passed");
    }
    else tflI2C.printStatus();  // `printStatus()` is for troubleshooting,
                                //  It's not necessary for operation.
    delay(500);

    Serial.print( "Get Firmware Version: ");
    if( tflI2C.Get_Firmware_Version( tfVer, adr))
    {
      Serial.print( tfVer[2]);
      Serial.print( ".");
      Serial.print( tfVer[1]);
      Serial.print( ".");
      Serial.println( tfVer[0]);
    }
    else tflI2C.printStatus();    
    delay(500);

    Serial.print( "Get Serial Number: ");
    if( tflI2C.Get_Prod_Code( tfCode, adr))
    {
      for( uint8_t i = 0; i < 14; ++i)
      {
        Serial.print( char( tfCode[i]));
      }
      Serial.println();
    }
    else tflI2C.printStatus();
    delay(500);

    // In main 'loop', command to print
    // device time in milliseconds is
    // called every 10 loops.
    Serial.print( "Get Time: ");
    if( tflI2C.Get_Time( tfTime, adr))
    {
      Serial.println(  tfTime);
    }
    else tflI2C.printStatus();
    delay(500);

    Serial.print( "Set Frame Rate to: ");
    if( tflI2C.Set_Frame_Rate( tfFrame, adr))
    {
      Serial.println(  tfFrame);
    }
    else tflI2C.printStatus();
    delay(500);
    
    //  Read frame rate back from the device
    Serial.print( "Get Frame Rate: ");
    if( tflI2C.Get_Frame_Rate( tfFrame, adr))
    {
      Serial.println(  tfFrame);
    }
    else tflI2C.printStatus();
    delay(500);

}

void setup()
{
    Serial.begin( 115200);  // Initialize Serial port
    Wire.begin();           // Initialize Wire library

    Serial.println( "TFLI2C example code"); // say "Hello!"
    Serial.println( "4 NOV 2021");          // and add date

    // Execute a group of commands.
    // Comment this out if not needed.

    
    //tflI2C.Hard_Reset(tfAddr);
    //delay(500);
    //if(tflI2C.Set_I2C_Addr(newaddr, tfAddr))
    //  Serial.println("POOP");
    //delay(500);
    //if(tflI2C.Save_Settings(newaddr))
    //  Serial.println("AAAAA");
    //delay(500);
    //if(tflI2C.Soft_Reset(tfAddr))
    //  Serial.println("BBBBBB");
    //delay(500);
    //if(tflI2C.Soft_Reset(newaddr))
    //  Serial.println("CCCCCC");


    //sampleCommands(newaddr);
    //tflI2C.Soft_Reset(0x1A);
    //while(1);

    pinMode(RED_LED, OUTPUT);

}

void loop()
{
    // If data is read without error...
    if( tflI2C.getData( tfDist, tfFlux, tfTemp, newaddr))
    {
        Serial.print("Dist: ");      // ...print distance,
        Serial.print(tfDist);
        //Serial.print(" | Flux: ");   // ...print quality
        //Serial.print(tfFlux);

        // Convert temperature from hundredths
        // of a degree to a whole number and...
        tfTemp = int16_t( tfTemp / 100);

        //Serial.print(" | Temp: ");     // ...print temperature.
       // Serial.println( tfTemp);
    }
    else tflI2C.printStatus();        // else, print error status.
    //sampleCommands(tfAddr);
    //tflI2C.Set_I2C_Addr(0x1A, tfAddr);
    //tflI2C.Soft_Reset(0x1A);


    // Every ten loops, print device time
    // in milliseconds and reset the counter.
    if( tfCount < 10) ++tfCount;
    else
    {
        Serial.print( "Get Time: ");
        tflI2C.Get_Time( tfTime, tfAddr);
        Serial.println(  tfTime);
        tfCount = 0;
    }

    
// // add the distance to the list
//   dist_list[list_index] = tfDist;

//   // check if the list is full
//   if (list_index == LIST_SIZE - 1) {
//     // compare the values in the list
//     bool isAlert = true;
//     for (int i = 0; i < LIST_SIZE - 1; i++) {
//       int diff = abs(dist_list[i] - dist_list[i+1]);
//       if (diff > THRESHOLD) {
//         isAlert = false;
//         break;
//       }
//     }

//     // print alert if the values are closely related
//     if (isAlert) {
//       int ALERT = activate_alert();
//       //Serial.println("ALERT!");
//     }

//     // shift the list to the left to make room for a new value
//     for (int i = 0; i < LIST_SIZE - 1; i++) {
//       dist_list[i] = dist_list[i+1];
//     }
//   } else {
//     // increment the index if the list is not yet full
//     list_index++;
//   }

  //delay(100); // delay for stabilitu
  delay(550);
}

int activate_alert()
 {

    // turn on the LED and Vibration motors
  digitalWrite(RED_LED, HIGH);
  delay(1000);
  digitalWrite(RED_LED, LOW);

  
  
  
}
    

    //elay(50);


