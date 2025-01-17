#define SETTINGSOPTIONSNO 11
#define SETTINGSVALUESNO 18//Maximum number of settings option values needed
int settingsValueIndex = 0;//currently selected settings option value index

struct SettingsOption
{
  char * option;//Settings option string
  char *value[SETTINGSVALUESNO];//Array of strings of settings option values
  int  handler;//Function to handle the values for this settings option
  int  currentIndex;//Function to array index of current value for this settings option
};

void settingsMIDICh(char * value);
void settingsMIDIOutCh(char * value);
void settingsVelocitySens(char * value);
void settingsKeyTracking(char * value);
void settingsPitchBend(char * value);
void settingsModWheelDepth(char * value);
void settingsEncoderDir(char * value);
void settingsPickupEnable(char * value);
void settingsBassEnhanceEnable(char * value);
void settingsScopeEnable(char * value);
void settingsVUEnable(char * value);
void settingsHandler(char * s, void (*f)(char*));

int currentIndexMIDICh();
int currentIndexVelocitySens();
int currentIndexKeyTracking();
int currentIndexPitchBend();
int currentIndexModWheelDepth();
int currentIndexEncoderDir();
int currentIndexPickupEnable();
int currentIndexBassEnhanceEnable();
int currentIndexScopeEnable();
int currentIndexVUEnable();
int getCurrentIndex(int (*f)());


void settingsMIDICh(char * value) {
  if (strcmp(value, "ALL") == 0) {
    midiChannel = MIDI_CHANNEL_OMNI;
  } else {
    midiChannel = atoi(value);
  }
  storeMidiChannel(midiChannel);
}

void settingsMIDIOutCh(char * value) {
  if (strcmp(value, "Off") == 0) {
    midiOutCh = 0;
  } else {
    midiOutCh = atoi(value);
  }
  storeMidiOutCh(midiOutCh);
}

void settingsVelocitySens(char * value) {
  if (strcmp(value, "Off") == 0) {
    velocitySens = 0;
  } else {
    velocitySens = atoi(value);
  }
}

void settingsKeyTracking(char * value) {
  if (strcmp(value, "None") == 0) keytrackingAmount = 0;
  if (strcmp(value, "Half") == 0)  keytrackingAmount =  0.5;
  if (strcmp(value, "Full") == 0) keytrackingAmount =  1.0;
}

void settingsPitchBend(char * value) {
  pitchBendRange = atoi(value);
  storePitchBendRange(pitchBendRange);
}

void settingsModWheelDepth(char * value) {
  modWheelDepth = atoi(value) / 10.0f;
  storeModWheelDepth(modWheelDepth);
}

void settingsEncoderDir(char * value) {
  if (strcmp(value, "Type 1") == 0) {
    encCW = true;
  } else {
    encCW =  false;
  }
  storeEncoderDir(encCW ? 1 : 0);
}

void settingsBassEnhanceEnable(char * value) {
  if (strcmp(value, "Off") == 0) {
    sgtl5000_1.enhanceBassDisable();
    storeBassEnhanceEnable(0);
  } else {
    sgtl5000_1.enhanceBassEnable();
    storeBassEnhanceEnable(1);
  }
}

void settingsPickupEnable(char * value) {
  if (strcmp(value, "Off") == 0) {
    pickUp = false;
  } else {
    pickUp =  true;
  }
  storePickupEnable(pickUp ? 1 : 0);
}

void settingsScopeEnable(char * value) {
  if (strcmp(value, "Off") == 0) {
    enableScope(false);
    storeScopeEnable(0);
  } else {
    enableScope(true);
    storeScopeEnable(1);
  }
}

void settingsVUEnable(char * value) {
  if (strcmp(value, "Off") == 0) {
    vuMeter = false;
    storeVUEnable(0);
  } else {
    vuMeter = true;
    storeVUEnable(1);
  }
}

//Takes a pointer to a specific method for the settings option and invokes it.
void settingsHandler(char * s, void (*f)(char*) ) {
  f(s);
}

int currentIndexMIDICh() {
  return getMIDIChannel();
}

int currentIndexMIDIOutCh() {
  return getMIDIOutCh();
}

int currentIndexVelocitySens() {
  return velocitySens;
}

int currentIndexKeyTracking() {
  if (keytrackingAmount == 0.0f) return 0;
  if (keytrackingAmount == 0.5f)  return 1;
  if (keytrackingAmount == 1.0f) return 2;
  return 0;
}

int currentIndexPitchBend() {
  return  getPitchBendRange() - 1;
}

int currentIndexModWheelDepth() {
  return (getModWheelDepth() * 10) - 1;
}

int currentIndexEncoderDir() {
  return getEncoderDir() ? 0 : 1;
}

int currentIndexPickupEnable() {
  return getPickupEnable() ? 1 : 0;
}

int currentIndexBassEnhanceEnable() {
  return getBassEnhanceEnable() ? 1 : 0;
}

//Takes a pointer to a specific method for the current settings option value and invokes it.
int getCurrentIndex(int (*f)() ) {
  return f();
}

int currentIndexScopeEnable() {
  return getScopeEnable() ? 1 : 0;
}

int currentIndexVUEnable() {
  return getVUEnable() ? 1 : 0;
}

CircularBuffer<SettingsOption, SETTINGSOPTIONSNO>  settingsOptions;

// add settings to the circular buffer
void setUpSettings() {
  settingsOptions.push(SettingsOption{"MIDI Ch.", {"All", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", '\0'}, settingsMIDICh, currentIndexMIDICh});
  settingsOptions.push(SettingsOption{"Vel. Sens.", {"Off", "1", "2", "3", "4", '\0'}, settingsVelocitySens, currentIndexVelocitySens});
  settingsOptions.push(SettingsOption{"Key Tracking", {"None", "Half", "Full", '\0'}, settingsKeyTracking, currentIndexKeyTracking});
  settingsOptions.push(SettingsOption{"Pitch Bend", {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", '\0'}, settingsPitchBend, currentIndexPitchBend});
  settingsOptions.push(SettingsOption{"MW Depth", {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", '\0'}, settingsModWheelDepth, currentIndexModWheelDepth});
  settingsOptions.push(SettingsOption{"MIDI Out Ch.", {"Off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", '\0'}, settingsMIDIOutCh, currentIndexMIDIOutCh});
  settingsOptions.push(SettingsOption{"Encoder", {"Type 1", "Type 2", '\0'}, settingsEncoderDir, currentIndexEncoderDir});
  settingsOptions.push(SettingsOption{"Pick-up", {"Off", "On", '\0'}, settingsPickupEnable, currentIndexPickupEnable});
  settingsOptions.push(SettingsOption{"Bass Enh.", {"Off", "On", '\0'}, settingsBassEnhanceEnable, currentIndexBassEnhanceEnable});
  settingsOptions.push(SettingsOption{"Oscilloscope", {"Off", "On", '\0'}, settingsScopeEnable, currentIndexScopeEnable});
  settingsOptions.push(SettingsOption{"VU Meter", {"Off", "On", '\0'}, settingsVUEnable, currentIndexVUEnable});
}
