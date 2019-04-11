/* ALLEN AND KEITH AUDIO MIXER CODE
 *  
 *  
 *  Analog inputs
 *    A0-A3 are Channel 1 to Channel 4 faders
 *    A4-A5 are the Main and Aux mix output faders
 *    A6-A9 are Channel 1 to Channel 4 aux levels
 *    [A10 or pin 49] is headphone volume control - how do you use pin A10?
 *    
 *  Digital inputs
 *    29,30,31,24 are the Channel 1 to Channel 4 Mutes (latching buttons) - pin 31 is used, we need to move channel 3 mute
 *    27 and 28 are the Main and Aux Mix Mute buttons  (latching buttons)
 *    9 and 10, 25 and 26, 54 and 55 are pairs for each of the rotary encoders
 *    33-36 are the four menu  buttons (momentary buttons)
 *
 *  Screens
 *    I2C Bus number zero has SCL on pin 7 and SDA on pin 8
 *    I2C Bus number one has SCL on pin 37 and SDA on pin 38
 *    I2C Bus number two has SCL on pin 3 and SDA on pin 4
 *    I2C Bus number three has SCL on pin 57 and SDA on pin 56
 * 
 *    SPI bus number 0 has MOSI on pin 11, MISO on pin 12, SCK on pin 13
 *    SPI bus number 1 has MOSI on pin 0, MISO on pin 1, SCK on pin 32
 *    SPI bus number 2 has MOSI on pin 44, MISO on pin 45, SCK on pin 46
*/

#include <FrequencyTimer2.h>
#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <Bounce.h>
#include <math.h>
#include <Wire.h>

#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h" //big screen
#include "Adafruit_SSD1306.h" //small screens
#include "Adafruit_TPA2016.h" //class d amps

int spi_speed = 30000000;
IntervalTimer ADC_timed_interrupt;

//this integer holds the current state of the menu
int menustate;

//these hold the analog values from the faders
int Channel1Fader;
int Channel2Fader;
int Channel3Fader;
int Channel4Fader;

int mainOutFader;
int auxOutFader;

//these hold the analog values from the aux mix pots
int Channel1AuxLevel;
int Channel2AuxLevel;
int Channel3AuxLevel;
int Channel4AuxLevel;

//this holds the analog value from the headphone volume control pot
int HeadphoneVolumeControl;

//these hold the digital values for all the mute buttons
int Channel1MuteButton;
int Channel2MuteButton;
int Channel3MuteButton;
int Channel4MuteButton;

int mainMixMuteButton;
int auxMixMuteButton;

//these are used to hold the acumulated values for the rotary encoders
int RotaryEncoder1Val;
int RotaryEncoder2Val;
int RotaryEncoder3Val;


//this allows the user to apply the parametric EQs to each of the input channels
int currently_selected_menu_channel;

//this keeps the number of the channel that the headphone monitor is tapped into
int currently_selected_headphone_channel;

//this keeps the value of the currently selected output
int currently_selected_output;

//filter coefficients

//Parametric
double channel_1_band_1_corner_freq;
double channel_2_band_1_corner_freq;
double channel_3_band_1_corner_freq;
double channel_4_band_1_corner_freq;

double channel_1_band_1_gain;
double channel_2_band_1_gain;
double channel_3_band_1_gain;
double channel_4_band_1_gain;

double channel_1_band_2_center_freq;
double channel_2_band_2_center_freq;
double channel_3_band_2_center_freq;
double channel_4_band_2_center_freq;

double channel_1_band_2_gain;
double channel_2_band_2_gain;
double channel_3_band_2_gain;
double channel_4_band_2_gain;

double channel_1_band_2_q;
double channel_2_band_2_q;
double channel_3_band_2_q;
double channel_4_band_2_q;

double channel_1_band_3_corner_freq;
double channel_2_band_3_corner_freq;
double channel_3_band_3_corner_freq;
double channel_4_band_3_corner_freq;

double channel_1_band_3_gain;
double channel_2_band_3_gain;
double channel_3_band_3_gain;
double channel_4_band_3_gain;

//Graphic
double main_mix_low_band_gain;
double aux_mix_low_band_gain;

double main_mix_middle_band_gain;
double aux_mix_middle_band_gain;

double main_mix_high_band_gain;
double aux_mix_high_band_gain;


//set up encoders
Encoder knob1(9, 10);
Encoder knob2(25, 26);
Encoder knob3(54, 55);

//set up bounce library for the 4 menu buttons
Bounce backbutton = Bounce(33, 10); 
Bounce nextbutton = Bounce(34, 10); 
Bounce prevbutton = Bounce(35, 10); 
Bounce entrbutton = Bounce(36, 10);



//define main display
#define TFT_CS 9
#define TFT_DC 10 //this needs to change
#define MOSI 11
#define SCK 13
#define TFT_RST -1 // RST can be set to -1 if you tie it to Arduino's reset
#define MISO 12

Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, MOSI, SCK, TFT_RST, MISO);



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1

Adafruit_SSD1306 Channel_1_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 Channel_2_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 Channel_3_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_SSD1306 Channel_4_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_SSD1306 Output_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

Adafruit_TPA2016 main_and_aux_mix_amp = Adafruit_TPA2016();
Adafruit_TPA2016 headphone_amp = Adafruit_TPA2016();



#define NUM_SAMPLES 20 //is this enough?

//these will be circular buffers used to draw out the waveform
uint16_t channel1_samples[NUM_SAMPLES];
uint16_t channel2_samples[NUM_SAMPLES];
uint16_t channel3_samples[NUM_SAMPLES];
uint16_t channel4_samples[NUM_SAMPLES];

int samples_buffer_current_index;

void setup() 
{//initialization and setup of all initial values and initial state
 
 
 
  //Initialize screens
  Channel_1_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Channel_2_display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  Channel_3_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Channel_4_display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  Output_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  Channel_1_display.clearDisplay();
  Channel_2_display.clearDisplay();
  Channel_3_display.clearDisplay();
  Channel_4_display.clearDisplay();
  Output_display.clearDisplay();
 
  
 
  main_and_aux_mix_amp.begin(TPA2016_I2CADDR,&Wire);
  headphone_amp.begin(TPA2016_I2CADDR,&Wire1); //handles these two on separate i2c busses since they have the same address
 
  samples_buffer_current_index = 0;
  
  //initializing the circular buffers associated with each of the input channels
  //these values will be used to draw out the waveform
  for(int i = 0; i < NUM_SAMPLES; i++)
  {//initialize with zeroes
    channel1_samples[i] = 0;
    channel2_samples[i] = 0;
    channel3_samples[i] = 0;
    channel4_samples[i] = 0;
  }
 

  
  //initialize values for encoders
  knob1.write(0);
  knob2.write(0);
  knob3.write(0);
   
  
  backbutton.update();
  nextbutton.update();
  prevbutton.update();
  entrbutton.update();
 
  //initialize with the current channel for the menu set to 1
  currently_selected_menu_channel = 1;
 
  //intitialize with the main output selected
  currently_selected_output = 1;
 
  //initialize the value for the current headphone channel
  currently_selected_headphone_channel = 0; //not listening to any channel
 
  
  
  //set up screens
  tft.begin();
  tft.fillScreen(HX8357_WHITE);
  tft.setRotation(1);
 
  //poll initial values of all controls
  poll_controls();

  //initialize filters with zero values
  //look at the MATLAB code for the eq

  //setup the timed interrupt with intervaltimer
  ADC_timed_interrupt.begin(sample,21);
 
 
 //initalize DAC
  analogWriteFrequency(A21,4000000);
  analogWriteFrequency(A22,4000000);
  analogWriteResolution(16);
 
  menustate = 1;
}

//these hold the values read from the ADCs
volatile uint16_t current_sample1;
volatile uint16_t current_sample2;
volatile uint16_t current_sample3;
volatile uint16_t current_sample4;

void sample()
{//this function is attached to the timed interrupt
  digitalWrite(33, HIGH);//begin converting
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
 
  current_sample1 = SPI.transfer16(0);
  current_sample2 = SPI.transfer16(0);
  current_sample3 = SPI.transfer16(0);
  current_sample4 = SPI.transfer16(0);
 
  SPI.endTransaction();
  digitalWrite(33, LOW);//converting finished
 
  //store these samples
  channel1_samples[samples_buffer_current_index] = 0;
  channel2_samples[samples_buffer_current_index] = 0;
  channel3_samples[samples_buffer_current_index] = 0;
  channel4_samples[samples_buffer_current_index] = 0;
 
  //increment the index for the circular buffer, and reset to zero if it is past the bounds
  samples_buffer_current_index++;
  if(samples_buffer_current_index == NUM_SAMPLES)
    samples_buffer_current_index = 0;
 
 
  //How are we doing all these?
  
  //apply parametric eq for input 1
  //apply parametric eq for input 2
  //apply parametric eq for input 3
  //apply parametric eq for input 4
  
  //apply compression for input 1
  //apply compression for input 2
  //apply compression for input 3
  //apply compression for input 4
  
  //do the mixing operation for main mix
  //do the mixing operation for aux mix
  
  //apply graphic eq to main mix
  //apply graphic eq to aux mix
  
  //apply compression to main mix
  //apply compression to aux mix
 
  //OUTPUT THE APPROPRIATE CHANNEL FOR THE HEADPHONES
  switch(currently_selected_headphone_channel){
   case 1:
    //output input 1's value
    break;
   case 2:
    //output input 2's value
    break;
   case 3:
    //output input 3's value
    break;
   case 4:
    //output input 4's value
    break;
   case 5:
    //output main output's value
    break;
   case 6:
    //output aux mix's value
    break;
   default:
    break;//i.e. ==0, listening to nothing
   }
 
  //OUTPUT THE MAIN AND AUX MIXES THROUGH THE DAC PINS
  
}

//the menu to be displayed on the larger screen is structured as follows
// 1 - the welcome screen
// 2,3,4,5 - the top level menu options (Parametric eq, graphic eq, compression, monitor channel select) - enter to toggle main/aux
// 6,7,8 - the bands of the parametric eq (band 1, band 2, band 3) - enter to toggle main/aux
// 9 - the graphic eq bands (three bands adjusted by the three knobs)
// 10-15 - the different channels in the compression menu (in1, in2, in3, in4, mainout, auxout, in order)
// 16-21,22 - the monitor channel selections (in1, in2, in3, in4, mainout, auxout, and none (22), in order)

void back_button()
{
  if(menustate >= 1 && menustate <= 5)
  {//this is the top level menu - go to the 'welcome' screen
    menustate = 1;
  }
  else if(menustate >= 6 && menustate <= 8)
  {//the menu starts in the parametric eq - go to the parametric eq top level menu option
    menustate = 2;
  }
  else if(menustate == 9)
  {//the menu starts in the graphic eq - go to the graphic eq top level menu option
    menustate = 3;
  }
  else if(menustate >= 10 && menustate <= 15)
  {//the menu starts in the compression menu - go to the compression top level menu option
    menustate = 4;
  }
  else if(menustate >= 16 && menustate <= 22)
  {//the menu starts in the monitor channel select menu - go to the monitor channel select top level menu option
    menustate = 5;
  }
}

void next_button()
{
  switch(menustate)
  {
    case 5://wraparound for the top level menu
      menustate = 1;
      break;
    case 8://wraparound for the parametric eq submenu
      menustate = 6;
      break;
    case 9://only one option in this submenu - no update neccesary
      break;
    case 15://wraparound for the compression submenu
      menustate = 10;
      break;
    case 22://wraparound for the monitor channel select submenu
      menustate = 16;
      break;
    default://in general, except for the preceding 5 cases, this increments the menustate
      menustate++;
      break;
  }
}

void previous_button()
{
  switch(menustate)
  {
    case 1://wraparound for the top level menu
      menustate = 5;
      break;
    case 6://wraparound for the parametric eq submenu
      menustate = 8;
      break;
    case 9://only one option in this submenu - no update neccesary
      break;
    case 10://wraparound for the compression submenu
      menustate = 15;
      break;
    case 16://wraparound for the monitor channel select submenu
      menustate = 22;
      break;
    default://in general, except for the preceding 5 cases, this decrements the menustate
      menustate--;
      break;
  }
}

void enter_button()
{
  switch(menustate)
  {
    case 2://you are selecting the parametric eq submenu
      menustate = 6;
      break;
    case 3://you are selecting the graphic eq submenu
      menustate = 9;
      break;
    case 4://you are selecting the compression submenu
      menustate = 10;
      break;
    case 5://you are selecting the monitor channel select submenu
      menustate = 16;
      break;
    case 6:
    case 7:
    case 8:
    case 9://IN THE PARAMETRIC EQ SETTINGS, THIS HAS TO CHANGE THE SELECTED CHANNEL, AS WELL
      currently_selected_menu_channel++; //move to the next channel
      if(currently_selected_menu_channel == 5) //if it's past the end, reset it to the first channel
        currently_selected_menu_channel = 1;        
      break;
   case 10:
     if(currently_selected_output == 1){
      currently_selected_output = 2;
     }else{
      currently_selected_output = 1;
     }
     break;
   case 16: //set the headphone monitor to listen to input 1
      break;
   case 17: //set the headphone monitor to listen to input 2
      break;
   case 18: //set the headphone monitor to listen to input 3
      break;
   case 19: //set the headphone monitor to listen to input 4
      break;
   case 20: //set the headphone monitor to listen to main out
      break;
   case 21: //set the headphone monitor to listen to aux out
      break;
   case 22: //set the headphone monitor to listen to nothing
      break;
    default:
      break;
  }
}
void peakfiltercalc(double dBgain, double freq, double Q, double &c0, double &c1, double &c2, double &c3, double &c4)
{
  double srate = 48000;
  
  double A = pow(10, (dBgain / 40));
    double b0, b1, b2, a0, a1, a2;
    double omega = 2 * 3.14159 * (freq / srate);
    double sn = sin(omega);
    double cs = cos(omega);
    double alpha = sn / (2 * Q);
    //double beta = sqrt(2 * A); // this wasn't being used anywhere
   
     b0 = 1 + (alpha * A);
     b1 = -2 * cs;
     b2 = 1 - (alpha * A);
     a0 = 1 + (alpha / A);
     a1 = -2 * cs;
     a2 = 1 - (alpha / A);
    
    c0 = b0 / a0;
    c1 = b1 / a0;
    c2 = b2 / a0;
    c3 = a1 / a0;
    c4 = a2 / a0;

}
  
void notchfiltercalc(double dBgain, double freq, double Q, double &c0, double &c1, double &c2, double &c3, double &c4)
{
  double srate = 48000;
  
  //double A = pow(10, dBgain / 40); //this isn't being used anywhere
  double O = 2 * 3.14159 * (freq / srate);
  
  double sn = sin(O);
  double cs = cos(O);
  
  double alpha = sn / (2 * Q);
  //double beta = sqrt(2 * A); - this isn't being used anywhere
    
  double b0 = 1;
  double b1 = -2 * cs;
  double b2 = 1;
  double a0 = 1 + alpha;
  double a1 = -2 * cs;
  double a2 = 1 - alpha;
  
  
  //return these, or have values in by reference
  c0 = b0 / a0;
  c1 = b1 / a0;
  c2 = b2 / a0;
  c3 = a1 / a0;
  c4 = a2 / a0;
}

void lowshelffiltercalc(double dBgain, double freq, double &c0, double &c1, double &c2, double &c3, double &c4){
     double srate = 48000;

     double A = pow(10, (dBgain / 40));
     double b0, b1, b2, a0, a1, a2;
     double omega = 2 * 3.14159 * (freq / srate);
     double sn = sin(omega);
     double cs = cos(omega);

     double beta = sqrt(2 * A);

     b0 = A*( (A+1) - (A-1)*cs + beta*sn );
     b1 = 2*A*( (A-1) - (A+1)*cs );
     b2 = A*( (A+1) - (A-1)*cs - beta*sn );
     a0 = (A+1) + (A-1)*cs + beta*sn;
     a1 = -2*( (A-1) + (A+1)*cs );
     a2 = (A+1) + (A-1)*cs - beta*sn;


     c0 = b0 / a0;
     c1 = b1 / a0;
     c2 = b2 / a0;
     c3 = a1 / a0;
     c4 = a2 / a0;
}

void highshelffiltercalc(double dBgain, double freq, double &c0, double &c1, double &c2, double &c3, double &c4){
    double srate = 48000;
  
    double A = pow(10, (dBgain / 40));
    double b0, b1, b2, a0, a1, a2;
    double omega = 2 * 3.14159 * (freq / srate);
    double sn = sin(omega);
    double cs = cos(omega);
    
    double beta = sqrt(2 * A);

    b0 = A*( (A+1) - (A-1)*cs + beta*sn );
    b1 = -2*A*( (A-1) - (A+1)*cs );
    b2 = A*( (A+1) - (A-1)*cs - beta*sn );
    a0 = (A+1) + (A-1)*cs + beta*sn;
    a1 = 2*( (A-1) + (A+1)*cs );
    a2 = (A+1) + (A-1)*cs - beta*sn;

    c0 = b0 / a0;
    c1 = b1 / a0;
    c2 = b2 / a0;
    c3 = a1 / a0;
    c4 = a2 / a0;
}

uint16_t apply_filter(uint16_t inp, uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2, double c0, double c1, double c2, double c3, double c4){
  uint16_t output1 = (((c0 * ((inp))) + (c1 * ((x1))) + (c2 * ((x2))) - (c3 * ((y1))) - (c4 * ((y2)))));
  x2 = (x1);
  x1 = (inp);
  y2 = (y1);
  y1 = (output1);
  return output1;
}

void poll_controls()
{//check the values from all the UI controls and update their values
  
  Channel1Fader = analogRead(14);
  Channel2Fader = analogRead(15);
  Channel3Fader = analogRead(16);
  Channel4Fader = analogRead(17);
  mainOutFader = analogRead(18);
  auxOutFader = analogRead(19);
  Channel1AuxLevel = analogRead(20);
  Channel2AuxLevel = analogRead(21);
  Channel3AuxLevel = analogRead(22);
  Channel4AuxLevel = analogRead(23);
  HeadphoneVolumeControl = analogRead(49);
  Channel1MuteButton  = digitalRead(29);
  Channel2MuteButton  = digitalRead(30);
  Channel3MuteButton  = digitalRead(31);
  Channel4MuteButton  = digitalRead(24);
  mainMixMuteButton   = digitalRead(27);
  auxMixMuteButton    = digitalRead(28);
  

  //if(abs(val=knob.read())) - if the read returns a nonzero value, this if statement will evaluate to true.
    //positive integers make if evaluate true - the value is significant if it is nonzero, positive or negative, thus, abs()
  if(abs(RotaryEncoder1Val = knob1.read())) encoder1_update(RotaryEncoder1Val);//update the values, based upon the current menustate
     
  if(abs(RotaryEncoder2Val = knob2.read())) encoder2_update(RotaryEncoder2Val);
    
  if(abs(RotaryEncoder3Val = knob3.read())) encoder3_update(RotaryEncoder3Val);
    
     
 
  if(backbutton.update())
    if(backbutton.risingEdge())
      back_button();
  
  if(nextbutton.update())
    if(nextbutton.risingEdge())
      next_button();
  
  if(prevbutton.update())
    if(prevbutton.risingEdge())
      previous_button();
  
  if(entrbutton.update())
    if(entrbutton.risingEdge())
      enter_button();
 
}      

double scale_fader(uint16_t fader_value){
   double temp_fader_val = fader_value / 1023;
   double dBmin = -80;
   double dBmax = +20;
   double range = dBmax - dBmin;

   double zeroShape = pow(10,(dBmin/20));
   double unityFix = 1 / (1 + zeroShape);

   double scaled_gain = pow(10,((range*temp_fader_val-dBmin)/20)-zeroShape)*unityFix;

   return scaled_gain;
  
}
     
     
//Encoder update functions - 
     //these will need to be sensitive to the current state of the menu, 
     //i.e. menustate will effect what the value is used for in each function
     
//relevant menustates -
     //6 - adjust parameters of band 1 of the parametric equalizer
     //7 - adjust parameters of band 2 of the parametric equalizer
     //8 - adjust parameters of band 3 of the parametric equalizer
     
     //9 - adjust the gains of each band of the graphic equalizer
     
     //10 - adjust the parameters of input 1's compression
     //11 - adjust the parameters of input 2's compression
     //12 - adjust the parameters of input 3's compression
     //13 - adjust the parameters of input 4's compression
     //14 - adjust the parameters of main output's compression
     //15 - adjust the parameters of aux output's compression
     
//parametric eq's have 
     //Q, gain, center_frequency
     
//graphic equalizer has
     //gain for 'bass', gain for 'mid', gain for 'high'
     
//compression has 
     //threshold(dB), ratio, third knob does nothing
     

void encoder1_update(int increment)
{
   knob1.write(0); //write zero value so as to allow that deviation from zero to be significant
   switch(menustate)//depends on where you are in the menu
   {
    case 6:// this is band 1 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 1 corner frequency
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }
     break;
    case 7://this is band 2 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 2 center frequency
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }
    case 8:// this is band 3 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 3 corner frequency
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }

    case 9://handle the graphic eq - this is for the low frequency one
     switch(currently_selected_output){
      case 1:
       break;
      case 2:
       break;
      default:
       break;
     }
    default:
     break;
   }
}

void encoder2_update(int increment)
{
  knob2.write(0);//write zero value so as to allow that deviation from zero to be significant
   switch(menustate)//depends on where you are in the menu
   {
    case 6:// this is band 1 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 1 gain
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }
     break;
    case 7://this is band 2 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 2 gain
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }
    case 8:// this is band 3 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 3 gain
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }

    case 9://handle the graphic eq - this is for the middle frequency one
     switch(currently_selected_output){
      case 1:
       break;
      case 2:
       break;
      default:
       break;
     }
    default:
     break;
   }
}

void encoder3_update(int increment)
{
  knob3.write(0);//write zero value so as to allow that deviation from zero to be significant
   switch(menustate)//depends on where you are in the menu
   {
    case 7://this is band 2 of parametric eq
     switch(currently_selected_menu_channel){
      case 1://update channel 1's parametric eq band 2 q factor
      case 2://channel 2
      case 3://etc
      case 4:
      default:
       break;
     }

    case 9://handle the graphic eq - this is for the higher frequency one
     switch(currently_selected_output){
      case 1:
       break;
      case 2:
       break;
      default:
       break;
     }
    default://you don't need to use the encoders
     break;
   }
}
     
void update_screens()
{
  tft.fillScreen(HX8357_WHITE);
  switch(menustate)
  {
    case 1://show the welcome screen
      tft.setTextSize(5);
      tft.setCursor(75, 100);
      tft.setTextColor(HX8357_BLACK);  
      tft.println("ALLEN");
      tft.setTextColor(HX8357_RED);
      tft.setCursor(225, 100);
      tft.println("&");
      tft.setTextColor(HX8357_BLACK);
      tft.setCursor(255, 100);
      tft.println("KEITH");
      tft.setTextSize(3);
      tft.setCursor(75, 150);
      tft.println("DIGITAL AUDIO MIXER");
      
      break;
    case 2://show the parametric EQ top level menu option
     tft.setTextSize(5);
      tft.setCursor(55, 100);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Parametric EQ");
      tft.setCursor(65, 140);
      tft.setTextSize(3);
      tft.println("Press Enter to select");
      break;
    case 3://show the graphic EQ top level menu option
       tft.setTextSize(5);
      tft.setCursor(110, 100);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Graphic EQ");
      tft.setCursor(65, 140);
      tft.setTextSize(3);
      tft.println("Press Enter to select");
      break;
    case 4://show the compression top level menu option
      tft.setTextSize(5);
      tft.setCursor(90, 100);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression");
      tft.setCursor(65, 140);
      tft.setTextSize(3);
      tft.println("Press Enter to select");
      break;
    case 5://show the monitor channel select top level menu option
      tft.setTextSize(5);
      tft.setCursor(20, 100);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Monitor Channel");
      tft.setCursor(65, 140);
      tft.setTextSize(3);
      tft.println("Press Enter to select");
      break;
    case 6://show the band 1 parametric EQ selection

      tft.setTextSize(5);
      tft.setCursor(45, 45);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 1");


      tft.setCursor(75, 95);
      tft.setTextSize(3);
      tft.println("Low shelf");

      tft.setCursor(50,155);
      tft.setTextSize(2);
      tft.println("Corner Frequency:");

      tft.setCursor(50,195);
      tft.setTextSize(2);
      tft.println("Gain:");

      tft.setCursor(270, 60);
      
      
      tft.setTextSize(3);
      switch(currently_selected_menu_channel){
        case 1:
          tft.println("Channel 1");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_1_band_1_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_1_band_1_gain);
          break;
        case 2:
          tft.println("Channel 2");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_2_band_1_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_2_band_1_gain);
          break;
        case 3:
          tft.println("Channel 3");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_3_band_1_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_3_band_1_gain);
          break;
        case 4:
          tft.println("Channel 4");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_4_band_1_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_4_band_1_gain);
          break;
        default:
          break;
      }

      break;
    case 7://show the band 2 parametric EQ selection
      tft.setTextSize(5);
      tft.setCursor(45, 45);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 2");


      tft.setCursor(75, 95);
      tft.setTextSize(3);
      tft.println("Peak Filter");

      tft.setCursor(50,155);
      tft.setTextSize(2);
      tft.println("Center Frequency:");

      tft.setCursor(50,195);
      tft.setTextSize(2);
      tft.println("Gain:");

      
      tft.setCursor(50,235);
      tft.setTextSize(2);
      tft.println("Q Factor:");

      tft.setCursor(270, 60);
      
      
      tft.setTextSize(3);
      switch(currently_selected_menu_channel){
        case 1:
          tft.println("Channel 1");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_1_band_2_center_freq);
          tft.setCursor(250,195);
          tft.print(channel_1_band_2_gain);
          tft.setCursor(250,235);
          tft.print(channel_1_band_2_q);
          break;
        case 2:
          tft.println("Channel 2");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_2_band_2_center_freq);
          tft.setCursor(250,195);
          tft.print(channel_2_band_2_gain);
          tft.setCursor(250,235);
          tft.print(channel_2_band_2_q);
          break;
        case 3:
          tft.println("Channel 3");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_3_band_2_center_freq);
          tft.setCursor(250,195);
          tft.print(channel_3_band_2_gain);
          tft.setCursor(250,235);
          tft.print(channel_3_band_2_q);
          break;
        case 4:
          tft.println("Channel 4");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_4_band_2_center_freq);
          tft.setCursor(250,195);
          tft.print(channel_4_band_2_gain);
          tft.setCursor(250,235);
          tft.print(channel_4_band_2_q);
          break;
        default:
          break;
      }

      
      break;
    case 8://show the band 3 parametric EQ selection

      tft.setTextSize(5);
      tft.setCursor(45, 45);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Band 3");


      tft.setCursor(75, 95);
      tft.setTextSize(3);
      tft.println("High shelf");

      tft.setCursor(50,155);
      tft.setTextSize(2);
      tft.println("Corner Frequency:");

      tft.setCursor(50,195);
      tft.setTextSize(2);
      tft.println("Gain:");

      tft.setCursor(270, 60);
      
      
      tft.setTextSize(3);
      switch(currently_selected_menu_channel){
        case 1:
          tft.println("Channel 1");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_1_band_3_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_1_band_3_gain);
          break;
        case 2:
          tft.println("Channel 2");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_2_band_3_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_2_band_3_gain);
          break;
        case 3:
          tft.println("Channel 3");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_3_band_3_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_3_band_3_gain);
          break;
        case 4:
          tft.println("Channel 4");
          tft.setCursor(250,155);
          tft.setTextSize(2);
          tft.print(channel_4_band_3_corner_freq);
          tft.setCursor(250,195);
          tft.print(channel_4_band_3_gain);
          break;
        default:
          break;
      }

      break;
    case 9://show the bands of the graphic EQ
        tft.setTextSize(5);
        tft.setCursor(45, 45);
        tft.setTextColor(HX8357_BLACK);
        tft.println("Graphic EQ");
        tft.setTextSize(2);
      if(currently_selected_output==1){
        //OUTPUT STUFF ABOUT THE MAIN OUTPUT
        tft.setCursor(45, 130);
        tft.println("Here's where you'd see some stuff");
        tft.setCursor(45, 145);
        tft.println("about the main output");
      }else if(currently_selected_output == 2){
        //OUTPUT STUFF ABOUT THE AUX OUTPUT
        tft.setCursor(45, 130);
        tft.println("Here's where you'd see some stuff");
        tft.setCursor(45, 145);
        tft.println("about the aux output");
      }else{
        //YOU DID IT WRONG
      }
    case 10://show the input 1 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 1");
      break;
    case 11://show the input 2 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 2");
      break;
    case 12://show the input 3 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 3");
      break;
    case 13://show the input 4 compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Input 4");
      break;
    case 14://show the main output compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Main Output");
      break;
    case 15://show the aux output compression options
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Compression Settings - Aux Output");
      break;
    case 16://show the option to select input 1 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 1");
      break;
    case 17://show the option to select input 2 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 2");
      break;
    case 18://show the option to select input 3 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 3");
      break;
    case 19://show the option to select input 4 for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Input 4");
      break;
    case 20://show the option to select main output for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Main Output");
      break;
    case 21://show the option to select aux output for the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("Aux Output");
      break;
    case 22://show the option to turn off the headphone monitor
      tft.setTextSize(5);
      tft.setTextColor(HX8357_BLACK);
      tft.println("None");
      break;
    default:
      break;
      
  }
}

void loop()                     
{//this will run in the time when the interrupt is not running
 //maindisplay
 
  //screens update
  update_screens();
 

  //check the values of all controls
  poll_controls();
}
