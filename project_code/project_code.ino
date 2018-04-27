#include <Keypad.h>

const byte ROWS = 4;        //four rows
const byte COLS = 3;        //three columns
const byte NUM_CHARS = 12;  //number of characters
const byte SPEAKER = 10;    //speaker pin
const byte ANALOG_IN = A0;  //microphone pin
const byte TONE_TIME = 200; //duration of each tone
const byte THRESHOLD = 39;  //noise threshold above which a button press is assumed


const double CERTAINTY = 0.0018;  //if any spectral magnitude of frequency band is > this, triggers output
                                  //this is a second measure that is supposed to prevent false positives

byte rowPins[ROWS] = {8, 7, 6, 5}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 3, 2}; //connect to the column pinouts of the keypad

//layout of the array of characters on the numerical keypad
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

//create a keypad object for easy number detection
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

uint32_t startTime;  //start time of sampling
uint32_t endTime;    //end time of sampling

int16_t signalSamples[128]; //sampled input at A0

double sampleFreq;  //sampling frequency (Hz)
double result;      //current Fourier spectral magnitude
double max_result;  //maximum Fourier spectral magnitude

uint8_t maxI; //character/frequency index of maximum correlation

double frequencies[] = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3300, 3500, 3700, 3900};

char charList[] = "*#0123456789";

/************* FUNCTION PROTOTYPES *************/

double Single_Point_DFT(int16_t inputArray[], int, int, float, float);

byte indexof(char *arr, char key)
{  
  for(int i = 0; i<NUM_CHARS; i++)
  {
    if(arr[i] == key)
      return i;
  }

  return -1;
}


void setup(){
  Serial.begin(9600);

  pinMode(ANALOG_IN, INPUT);
  pinMode(SPEAKER, OUTPUT);
}

void loop() {

  char key = keypad.getKey();

  bool sound = false;

  byte index;
  
  if (key){
    index = indexof(charList, key);

  tone(SPEAKER, frequencies[index], TONE_TIME); 
    
  }

  for (int i = 0; i<20; i++){

    delayMicroseconds(500);

    if(analogRead(ANALOG_IN) >= THRESHOLD) {
      sound = true;
      break;

    }
  }

    if(sound) {
    
    startTime = micros();
    
    for(uint8_t n = 0; n<128; n++)
    {
      signalSamples[n] = analogRead(ANALOG_IN);
    }
    
    endTime = micros();
  
    max_result = 0;
  
    maxI = 0;
  
    sampleFreq = 128.0*1000000.0/(endTime - startTime);
    
    for (int i = 0; i < NUM_CHARS; i++){
  
        result = Single_Point_DFT(signalSamples, 0, 127, sampleFreq, frequencies[i]);
  
        if(result > max_result) {
          
          maxI = i;

          max_result = result;

        }
         
    }

  if(max_result > CERTAINTY) {

  Serial.print("You Pressed: ");
  Serial.println(charList[maxI]); 

  
  }

}
  
}


double Single_Point_DFT(int16_t inputArray[], int startingIndex, int endingIndex, float sampleFreq, float freqToSampleFor){
  double N = endingIndex - startingIndex + 1; //N=number of samples
  double k = freqToSampleFor * N / sampleFreq; 
  double radiansPerSample = 2.0 * 3.14159 * k / N;

  double RealSum = 0;
  double ImaginarySum = 0;
  for (int n = 0; n <= endingIndex - startingIndex; n++){
    double angle = n * radiansPerSample;
    double voltage = inputArray[startingIndex + n] * (5.0 / 1023.0);

    RealSum += voltage * cos(angle);
    ImaginarySum -= voltage * sin(angle);
  }

  return (RealSum*RealSum + ImaginarySum*ImaginarySum) / N;
}






