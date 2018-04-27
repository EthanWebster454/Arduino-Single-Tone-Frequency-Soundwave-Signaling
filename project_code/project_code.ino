#include <math.h>

const byte NUM_CHARS = 12;    //number of characters
const byte ANALOG_IN = A0;    //microphone pin
const byte NUM_SAMPLES = 128; //number of samples for signal

const byte THRESHOLD = 39;    //noise threshold above which a button press is assumed
const double CERTAINTY = 0.0018;      //if any spectral magnitude of frequency band is > this, triggers output
                                      //this is a second measure that is supposed to prevent false positives
const int SAMPLING_FREQUENCY = 8000;  //according to Nyquist theorem, sampling rate must be twice maximum frequency
const int SAMPLING_PERIOD = (int)(1000000*(1.0/SAMPLING_FREQUENCY)); //time in between taking samples

unsigned int startTime;  //start time of sampling
unsigned int endTime;    //end time of sampling

unsigned short signalSamples[NUM_SAMPLES]; //sampled input at A0

double result;      //current Fourier spectral magnitude
double max_result;  //maximum Fourier spectral magnitude

byte maxI;  //character/frequency index of maximum correlation

//discrete frequencies associated with keypad characters
double frequencies[] = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3300, 3500, 3700, 3900};

char charList[] = "*#0123456789"; //list of characters found on keypad

/************* FUNCTION PROTOTYPES *************/
//computes the Discrete Fourier Transform of a sampled signal for a single frequency
double singleFreqDFT(int16_t signalArray[], int, int, float);


void setup(){

  //initialize serial i/o
  Serial.begin(9600);

  //setup I/O pins
  pinMode(ANALOG_IN, INPUT);
}

void loop() {


  if(analogRead(ANALOG_IN) >= THRESHOLD) {
    
    for(int n = 0; n<128; n++) {
      
      signalSamples[n] = analogRead(ANALOG_IN);

      delayMicroseconds(SAMPLING_PERIOD);
    }
  
    max_result = 0;
  
    maxI = 0;
    
    for (int i = 0; i < NUM_CHARS; i++){
  
        result = singleFreqDFT(signalSamples, 0, NUM_SAMPLES-1, frequencies[i]);
  
        if(result > max_result) {
          
          maxI = i;

          max_result = result;

        }
         
    }

    if(max_result > CERTAINTY) {

      Serial.print("You pressed: ");
      Serial.println(charList[maxI]); 

  
  }

}

delay(1);
  
}


double singleFreqDFT(int16_t signalArray[],   //sampled signal
                     int startingTime,        //starting time in microseconds
                     int endingTime,          //ending time in microseconds
                     float targetFrequency){  //frequency to compute DFT magnitude for
    
  double N = endingTime - startingTime + 1;             //find number of samples
  double k = targetFrequency * N / SAMPLING_FREQUENCY;  //find ratio of target frequency to sampling frequency
  double omega = 2.0 * M_PI * k / N;                    //find radians / num_samples

  double realPart = 0;        //real component of DFT, corresponding to cosine
  double imaginaryPart = 0;   //imaginary component of DFT, corresponding to sine
  double theta;               //current angle, in radians
  double currV;               //current voltage

  //iterate through samples
  for (int n = 0; n <= endingTime - startingTime; n++){

    //current angle is simply the index times the omega0
    theta = n * omega;

    //we need to convert the signal from Arduino reading to real voltage
    currV = signalArray[startingTime + n] * (5.0 / 1023.0);

    //find real and imaginary components of DFT
    realPart += currV * cos(theta);
    imaginaryPart -= currV * sin(theta);
  }

  //find and return normalized Fourier magnitude of current sampled signal
  return (realPart*realPart + imaginaryPart*imaginaryPart) / N;
}






