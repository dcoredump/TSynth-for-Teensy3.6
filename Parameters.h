//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
byte midiChannel = MIDI_CHANNEL_OMNI;//(EEPROM)
byte midiOutCh = 0;//(EEPROM)
String patchName = INITPATCHNAME;
boolean encCW = true;//This is to set the encoder to increment when turned CW - Settings Option
float oscALevel = 1;
float oscBLevel = 1;
float noiseLevel = 0;
int unison = 0;
int oscFX = 0;
float detune = 0.98;
float lfoSyncFreq = 1.0;
float midiClkTimeInterval = 0.0;
float lfoTempoValue = 1.0;
int pitchBendRange = 12;
float modWheelDepth = 0.2f;
float modWhAmt = 0.0f;
float keytrackingAmount = 0.5;//MIDI CC & settings option
float glideSpeed = 0;
int oscPitchA = 0;
int oscPitchB = 12;
float pitchEnv = 0;
int oscWaveformA = WAVEFORM_SQUARE;
int oscWaveformB = WAVEFORM_SQUARE;
float pwmAmtA = 1;
float pwmAmtB = 1;
float pwmRate = 0.5;
float pwA = 0;
float pwB = 0;
int pwmSource = PWMSOURCELFO;

float filterRes = 1.1;
float filterFreq = 12000;
float filterOctave = 7.0;
float filterMix = 0;
int filterMixStr = 0;//For display
float filterEnv = 0;
float oscLfoAmt = 0;
float oscLfoRate = 4;
int oscLFOWaveform = WAVEFORM_SINE;
int oscLfoRetrig = 0;
int oscLFOMidiClkSync = 0;//MIDI Only
String oscLFOTimeDivStr = "";//For display
float filterLfoRate = 2;
int filterLfoRetrig = 0;
int filterLFOMidiClkSync = 0;
String filterLFOTimeDivStr = "";//For display
float filterLfoAmt = 0;
int filterLfoWaveform = WAVEFORM_SINE;

float filterAttack = 100;
float filterDecay = 350;
float filterSustain = 0.7;
float filterRelease = 300;

float ampAttack = 10;
float ampDecay = 35;
float ampSustain = 1;
float ampRelease = 300;

float fxAmt = 1;
float fxMix = 0;

int velocitySens = 0;//Default off - settings option

boolean vuMeter = false;

//Pick-up - Experimental feature
//Control will only start changing when the Knob/MIDI control reaches the current parameter value
//Prevents jumps in value when the patch parameter and control are different values
volatile boolean pickUp = false;//settings option (EEPROM)
volatile boolean pickUpActive = false;
#define TOLERANCE 2 //Gives a window of when pick-up occurs, this is due to the speed of control changing and Mux reading
uint32_t filterfreqPrevValue = 0;//Need to set these when patch loaded
float filterMixPrevValue = 0.0f;//Need to set these when patch loaded
float resonancePrevValue = 0.0f;//Need to set these when patch loaded
float oscLfoAmtPrevValue = 0.0f;//Need to set these when patch loaded
float oscLfoRatePrevValue = 0.0f;//Need to set these when patch loaded
float filterLfoRatePrevValue = 0.0f;//Need to set these when patch loaded
float filterLfoAmtPrevValue = 0.0f;//Need to set these when patch loaded
float fxAmtPrevValue = 0.0f;//Need to set these when patch loaded
float fxMixPrevValue = 0.0f;//Need to set these when patch loaded
