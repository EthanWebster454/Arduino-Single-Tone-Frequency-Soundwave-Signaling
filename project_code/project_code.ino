/*
This code finds the DFT of frequencies defined at the keypad pins for button press detection
*/

const byte NUM_CHARS = 12;    //number of characters
const byte MIC_INPUT = A0;    //microphone pin
const byte NUM_SAMPLES = 128; //number of samples for signal, must be a power of 2

const byte HI_THRESHOLD = 39;         //noise threshold above which a button press is assumed
const byte LO_THRESHOLD = 35;         //noise threshold below which a button press is assumed
const double CERTAINTY = 0.0018;      //if any spectral magnitude of frequency band is > this, triggers output
                                      //this is a second measure that is supposed to prevent false positives

unsigned short signalSamples[NUM_SAMPLES]; //sampled microphone signal, only need 16 bit integers (conserves memory)

unsigned short currReading;   //current microphone reading for attempting to filter room noise

unsigned int startTime;
unsigned int endTime;

double samplingFreq;  //current sampling frequency (Hz)
double result;        //current Fourier spectral magnitude
double max_result;    //maximum Fourier spectral magnitude

byte maxI;  //character/frequency index of maximum correlation

//discrete frequencies associated with keypad characters
double frequencies[] = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3300, 3500, 3700, 3900};

char charList[] = "*#0123456789"; //list of characters found on keypad

/************* FUNCTION PROTOTYPES *************/
//computes the Discrete Fourier Transform of a sampled signal for a single frequency
double singleFreqDFT(unsigned short *, double, float);


void setup(){

  //initialize serial i/o
  Serial.begin(9600);

  //setup I/O pins
  pinMode(MIC_INPUT, INPUT);
}

void loop() {

  currReading = analogRead(MIC_INPUT);  //obtain microphone reading

  //only perform DFT if the microphone reading is outside a pre-determined bi-level threshold
  //the idea behind this is to filter out room noise and only analyze speaker noise
  if(currReading >= HI_THRESHOLD || currReading <= LO_THRESHOLD) {

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

      Serial.print("You pressed: ");
      Serial.println(charList[maxI]); 

  }

}

delayMicroseconds(500);
  
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
  return (realPart*realPart + imaginaryPart*imaginaryPart) / (double)NUM_SAMPLES;
}






