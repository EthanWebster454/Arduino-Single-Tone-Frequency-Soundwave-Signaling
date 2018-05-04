#include <Keypad.h>

//discrete frequencies associated with keypad characters
double frequencies[] = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3300, 3500, 3700, 3900};

char charList[] = "*#0123456789"; //list of characters found on keypad

const byte SPEAKER = 10;    //speaker pin
const byte ROWS = 4;        //four rows
const byte COLS = 3;        //three columns
const byte TONE_TIME = 250; //duration of each tone
const byte NUM_CHARS = 12;  //number of characters on keypad

byte rowPins[ROWS] = {8, 7, 6, 5};  //connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 3, 2};     //connect to the column pinouts of the keypad

byte index; //index of key in frequency array

char key;   //key that is pressed, if any


//layout of the array of characters on the numerical keypad
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

//create a keypad object for easy number detection
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//finds the index of a character in the string array
int indexof(char *arr, char key)
{  
  for(int i = 0; i<NUM_CHARS; i++)
  {
    if(arr[i] == key)
      return i;
  }

  return -1;
}

void setup() {
  
  pinMode(SPEAKER, OUTPUT);

}

void loop() {

  key = keypad.getKey();

  if (key){
    
    index = indexof(charList, key);

    tone(SPEAKER, frequencies[index], TONE_TIME); 
    
  }


  delay(10);


}
