/*
This code finds the DFT of frequencies defined at the keypad pins for button press detection
*/

#include <Keypad.h>
#include <LiquidCrystal.h>


const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
const byte TONE_TIME = 150;
const byte NUM_CHARS = 12;    //number of characters
const byte SPEAKER = 10;      //digital pin for speaker output
const byte MIC_INPUT = A0;    //microphone pin
const byte NUM_SAMPLES = 128; //number of samples for signal, must be a power of 2

const byte THRESHOLD = 38;        //noise threshold above which a button press is assumed
const double CERTAINTY = 0.002;   //if any spectral magnitude of frequency band is > this, triggers output
                                  //this is a second measure that is supposed to prevent false positives

const char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
                                      
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; //list of pins for LCD screen

unsigned short signalSamples[NUM_SAMPLES]; //sampled microphone signal, only need 16 bit integers (conserves memory)

unsigned short currReading;   //current microphone reading for attempting to filter room noise

unsigned int startTime;   //starting time of sampling (ms)
unsigned int endTime;     //ending time of sampling (ms)

double samplingFreq;  //current sampling frequency (Hz)
double result;        //current Fourier spectral magnitude
double max_result;    //maximum Fourier spectral magnitude

byte maxI;  //character/frequency index of maximum correlation

char key;   //current pressed key

bool sound; //true if noise detected above a certain threshold

byte index; //index of character corresponding to a specific frequency

byte rowPins[ROWS] = {34, 32, 30, 28};  //connect to the row pinouts of the keypad
byte colPins[COLS] = {26, 24, 22};      //connect to the column pinouts of the keypad

//discrete frequencies associated with keypad characters
double frequencies[] = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3300, 3500, 3700, 3900};

char charList[] = "*#0123456789"; //list of characters found on keypad

//construct keypad object
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//construct LCD object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/************* FUNCTION PROTOTYPES *************/
//computes the Discrete Fourier Transform of a sampled signal for a single frequency
double singleFreqDFT(unsigned short *, double, float);

//aids in finding the index of frequency assigned to a character
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

  //initialize LCD with number of rows and columns
  lcd.begin(16, 2);
  
  //print heading to LCD
  lcd.print("Button Pressed:");

  //setup I/O pins
  pinMode(MIC_INPUT, INPUT);
  pinMode(SPEAKER, OUTPUT);
  
}


void loop() {

  //obtain pressed key
  key = keypad.getKey();

  sound = false; //assume no sound initially

  //if a key is pressed, then find the index of the key
  if (key){
    index = indexof(charList, key);

  //play a tone corresponding to the frequency index
  tone(SPEAKER, frequencies[index], TONE_TIME); 
    
  }

  //loop for 1 ms, looking for a signal
  for (int i = 0; i<20; i++){

    delayMicroseconds(50);  //delay before testing

    //if the signal crosses a certain threshold, then assume a tone was played
    if(analogRead(MIC_INPUT) >= THRESHOLD) {
      sound = true;
      break;

    }
  }


  //if it was determined that a sound was 'heard'
  if(sound) {
    
  //then perform DFT on a sampled version of the signal 

    startTime = micros(); //obtain current clock pre-sampling

    //obtain N samples of signal
    for(int n = 0; n < NUM_SAMPLES; n++)
    {
      signalSamples[n] = analogRead(MIC_INPUT);
    }

    endTime = micros(); //find ending time after sampling

    //obtain current sampling frequency in Hz
    samplingFreq = NUM_SAMPLES*1000000.0/((double)(endTime - startTime));
  
    max_result = 0; //initialize maximum DFT magnitude found during analysis
  
    maxI = 0;       //initialize frequency index of maximum correlation

    //iterate through frequencies
    for (int i = 0; i < NUM_CHARS; i++){

        //find the DFT at the current frequency in the list
        result = singleFreqDFT(signalSamples, samplingFreq, (float)frequencies[i]);

        //keep track of the maximum magnitude and its index
        if(result > max_result) {
          
          maxI = i;
          max_result = result;
        }
         
    }

    //second layer of defense against false positives: 
    //this ensures that max DFT magnitude implies button press
    if(max_result > CERTAINTY) {

      //print number decided for the DFT
      lcd.setCursor(0, 1);
      lcd.print(charList[maxI]); 

  }

  
}

}

//computes the Discrete Fourier Transform of a sampled signal for a single frequency
double singleFreqDFT(unsigned short *signalArray, //sampled signal
                     double sampleFreq,           //sampling rate (Hz)
                     float targetFrequency){      //frequency to compute DFT magnitude for
  
  double omega = 2.0 * PI * targetFrequency / sampleFreq;  //find omega0 in radians

  double realPart = 0;        //real component of DFT, corresponding to cosine
  double imaginaryPart = 0;   //imaginary component of DFT, corresponding to sine
  double currV;               //current voltage

  //iterate through all samples
  for (int k = 0; k < NUM_SAMPLES; k++) {

    //we need to convert each amplitude of signal from Arduino integer reading to voltage
    currV = (double)signalArray[k] * (5.0 / 1023.0);

    //find real and imaginary components of DFT, using voltage and angle (omega0*k) in radians
    realPart += currV * cos(k * omega);
    imaginaryPart -= currV * sin(k * omega);
  }
 
  //find and return normalized Fourier magnitude of current sampled signal
  return sqrt(realPart*realPart + imaginaryPart*imaginaryPart) / (double)NUM_SAMPLES;
}






