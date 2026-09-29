// This is a general two spouts state machine to run 2 spout tasks (LLLR/2ABT/Pavlovian)
// Sketch runs experiment and communicates with Melanie/Bastijn's Matlab GUI

// Pavel's breakout board for Teensy 4
// PIN2 (PWM) - SpeakerPinL
// PIN3 (PWM) - SyncPin
// PIN4_SLN1 - copy SLN1
// PIN5_SLN2 - copy SLN2

// ANALOG_A1 - SpeakerPinR
// ANALOG_A0 - LeftLickPin
// DAC1 (PIN41) - RightLickPin
// DAC0 (PIN40) - OptoPin

// PIN9 - OptoModPin
// PIN21 - RandomLightPin

// communication with Matlab GUI (serial) - Serial.Print can be slow, therefore:
// serialOutBuffer is a Struct (complex struct which allows for easy manipulating)
// to make sure we never run into timing issues (because loops are getting slow), we store text characters
// in serialOutBuffer. Once per loop, we empty SerialOutBuffer using function updateSerialOutput.
// since we don't know how much we've stored in serialOutBuffer (could be many many characters) and we don't
// want to reduce the cycle time, we write per cycle a fixed number of characters from serialOutBuffer memory
// (24 -> emperical finding). next cycle, we write the next 24 (or less). as long as the number of characters
// is less than 24 per cycle, you're gonna be fine. even if it's a bit more, teensy will catch up in cycles with
// no new info

// load libraries
#include <EEPROM.h>  // EEPROM: memory storage for parameter variables
// #include <math.h>


// the following setting is required for testing
const bool SpoutNotButton = true;  // you use spouts (pull down, true) or buttons (pull up, false)

///////////////////////////////////////////////////////////////////////////////////////////////////////
// --------------------------- Modify pin numbers if needed ---------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Input output pin description //
const byte SyncPin = 3;         // non-periodic sync pulse - 2
const byte ExtSyncPin = 22;     // external TTL sync pulse INPUT (0/3V), idle low
const byte SpeakerPinL = 2;     // left speaker output pin
const byte SpeakerPinR = 15;    // right speaker output pin
const byte LeftLickPin = 14;    // left lick detection - 18
const byte RightLickPin = 41;   // right lick detection - 19
const byte LeftRewardPin = 5;   // hardwired left spout solenoid - 9
const byte RightRewardPin = 4;  // hardwired right spout solenofid - 8

// The following pins are TTLs
const byte LeftCueIndicatorPin = 17;      // speaker output step copy - Left tone - 48
const byte RightCueIndicatorPin = 18;     // speaker output step copy - Right tone - 49
const byte SpeakerTTLPin = 6;             // both speakers ttl
const byte LeftRewardIndicatorPin = 19;   // copy left spout solenoid for data recording device - 6
const byte RightRewardIndicatorPin = 20;  // copy right spout solenoid for data recording device - 7
const byte ITIIndicatorPin = 8;           // pulse that signals ITI period (Modified to 24 by SL, originally 36) - 36
const byte DelayIndicatorPin = 24;        // pulse that signals DELAY period (DR task)
const byte StartStopIndicatorPin = 10;    // start-stop session output pin, short pulse at start and stop - 30
const byte OptoPin = 40;                  // opto control - 22 - 4 (41)
const byte OptoTTLPin = 1;                // send opto copy to reporter LED (12)
const byte OptoModPin = 9;                // opto modulation pin 0-5 V, e.g., for AOM modulation. use PWM (0-255), requires a level converter and RC filter
const byte RandomLightPin = 21;           // random light as background - 24
const byte TrialStartPin = 7;             // send TTL when trial starts - 7
const byte CameraTriggerPin = 11;         // camera trigger, simply 2ms pulses at 50Hz
const byte ErrorLightPin = 13;            // light cue to signal animal made an error (wrong lick)
const byte ExtSyncReportPin = 12;         // report external sync signal on a pin (LED)

// Delayed-Response (DR) task pins
const byte StartCueSpeakerPin = 23;       // dedicated start/go cue speaker (separate pin)
const byte StartCueIndicatorPin = 16;     // start/go cue TTL copy

// The following is not in use
const bool buttonsAreWiredUp = false;  // Are the buttons wired up?
const byte StartStopButtonPin = 39;    // start-stop button - 53
const byte LeftRewardButtonPin = 38;   // left spout manual activation - 11
const byte RightRewardButtonPin = 37;  // right spout manual activation - 12


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------ Task related variables do not have to be modified due to EEPROM storage ------------------ //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// REWARD VARIABLES
unsigned long RewardDelay;  // delay between correct lick (or cue end in Pavlovian) and reward delivery (ms)

// Reward size
unsigned long RewardLeftCurrentSize;   // Duration that left solenoid is left open
unsigned long RewardRightCurrentSize;  // Duration that right solenoid is left open
int RewardLeftMin;                     // min reward size left
int RewardLeftMax;                     // max reward size left
int RewardLeftStepSize;                // reward size step (in ms)
int RewardRightMin;                    // min reward size right
int RewardRightMax;                    // max reward size right
int RewardRightStepSize;               // reward size step (in ms)
int RewardLeftRange[100];              // range of possible rewards for left from min to max
int RewardRightRange[100];             // range of possible rewards for right for min to max
int numRewardLeftRange;                // randomly pick a reward from the range list for left
int numRewardRightRange;               // randomly pick a reward form the range list for right

// Reward probability
int RewardProb;                 // probability of reward being dispensed if correct
int RewardProbList[100];        // reward probability list in steps of 1
bool giveReward = false;            // will we give reward this trial or not (correct spout)
bool giveRewardIncorrect = false;   // will we give reward if incorrect spout (converse probability)
bool correctSelection = false;      // current trial reward or noreward

// Free Reward
int failCounter;        // Tracks consecutive failures
int fails2reward;       // Number of failures before triggering free reward
int freeRewardCounter;  // Counts number of free rewards

// Spout start
int spoutStart;  // 0 = random, 1 = left, 2 = right

// TRIAL AND BLOCK VARIABLES

// TASK MODE (mutually exclusive)
bool taskLLLR;             // LLLR task (default)
bool task2ABT;             // two-armed bandit task (2ABT)
bool taskPavlovian;        // Pavlovian conditioning task
bool taskDelayedResponse;  // Delayed-Response task: CUE -> DELAY -> START_CUE -> SELECTION

// Pavlovian lick-to-start (trial initiation lick)
bool PavlovianLickToStart;  // true = Pavlovian only: after a DELIVERED reward, the animal must lick the block spout (during CONSUMPTION or WAIT_LICK) before the next trial (ITI/ENL) starts

// ITI mode
bool ENLEnabled;  // if true, ITI becomes an enforced non-lick period, otherwise plain ITI: licks recorded but do not reset the trial

// Trial time parameters (all values in milliseconds)
unsigned long ReactionTime;         // maximum reaction period (after Cue) in ms
unsigned long ConsumptionDuration;  // comsumption duration in ms (either reward or noreward)
unsigned long CorrectDuration;      // reward duration in ms
unsigned long IncorrectDuration;    // noreward duration in ms
unsigned long minITI;
unsigned long maxITI;
unsigned long ITIDurationList[100] = { 0 };
int NumITIDurations;  // ** Must equal num items in ITIDurationList list **
unsigned long ITIstep;
unsigned long TaskStartDelay;       // delay (in seconds) between '8' command and first NEW_TRIAL transition

// Delayed-Response (DR) task timing
unsigned long DelayDuration;          // duration (ms) of DELAY state between sample cue and start cue
unsigned long StartCueDuration;       // duration (ms) of START_CUE tone
unsigned long DelayPenaltyDuration;   // delay-period penalty duration (ms); 0 = licks during delay are recorded but not penalized
unsigned long StartCuePenaltyDuration;// start-cue-period penalty duration (ms); 0 = licks during start cue are recorded but not penalized

// Block length
int BlockLengthsLeft[1000];   // Number of trials per block - each new block picks from this list with uniform prob for LEFT
int BlockLengthsRight[1000];  // Number of trials per block - each new block picks from this list with uniform prob for RIGHT
int minBlock;                 // Minimum block length - used for Matlab GUI
int maxBlock;                 // Maximum block length - used for Matlab GUI
int minBlockLeft;             // Minimum block length for left trials
int maxBlockLeft;             // Maximum block length for left trials
int minBlockRight;            // Minimum block length for right trials
int maxBlockRight;            // Maximum block length for right trials
int numBlockLengthsLeft;      // Once fillBlockLengths() is called, this will equal the number of options a block length can be for LEFT
int numBlockLengthsRight;     // Once fillBlockLengths() is called, this will equal the number of options a block length can be for RIGHT

// OPTOGENETIC VARIABLES

// Optogenetic stimulation parameters - when to stimulate
bool OptoActiveDuringITI;          // Opto stim will happen during the ITI of that trial (only when previous trial was rewarded, but not during the very first trial)
bool OptoActiveDuringCue;          // Opto stim will happen during the cue presentation
bool OptoActiveDuringConsumption;  // Opto stim will happen during the Consumption state of that trial (reward/correct trials only)
bool OptoActiveDuringBlockSwitch;  // Opto stim will happen during the first trial after a block switch
bool OptoActiveDuringError;        // Opto stim will happen during error (incorrect lick) trials
bool OptoActiveDuringAnyOutcome;   // Opto stim will happen during any outcome (reward, error, or omission)
bool OptoActiveDuringDelay;        // Opto stim will happen during the DELAY state (DR task)
bool OptoActiveDuringStartCue;     // Opto stim will happen during the START_CUE state (DR task)

// Optogenetic stimulation parameters - how to stimulate
unsigned long OptoStimProb;       // 0.65 Probability of opto stim on that trial (set to 0 to turn off opto)
unsigned long OptoDelay;          // delay from start of ITI/Consuption to opto ON (in ms)
unsigned long OptoDuration;       // total opto ON time (in ms)
bool optoContinuous;              // opto still will be continuous (ideal for inhibition)
unsigned long OptoPulseDuration;  // opto pulse duration (ms)
unsigned long OptoFrequency;      // opto frequency of pulse + ITI (Hz)
unsigned long OptoMod;            // opto modulation 0-100 % (0-5 V) for AOM etc
unsigned long OptoTaperOff;       // opto taper off over time window (ms)
bool OptoModContinuous;           // opto modulation continuously on (for AOM) or during opto (for LED driver)

// Optogenetic stimulation parameters - minor details
int numCorrectTrialsBeforeOpto;  // Opto stim will happen after the Nth correct trial
bool AllowMultiOptoStimBlock;    // allow multiple trials per block to have opto
bool AllowMultiOptoStimITI;      // Stimulate every time the ITI gets reset (ENL_p or Cue_p)
bool AllowMultiOptoStimCue;      // Stimulate every time the Cue gets reset
bool AllowMultiOptoStimDelay;    // Stimulate every time the Delay gets reset (DR task, delay-penalty re-arm)
bool AllowMultiOptoStimStartCue; // Stimulate every time the StartCue gets reset (DR task, startcue-penalty re-arm)
bool ApplyOptoRandomLight;       // apply random light pulse to hide opto, true or false
unsigned long RandomLightTimer;
unsigned long RandomLightInterval;
int RandomLightON;

// Cue variables
unsigned long LeftCueFreq;   // in Hz
unsigned long RightCueFreq;  // in Hz
unsigned long StartCueFreq;  // in Hz - start/go cue (DR task)
unsigned long ToneDuration;  // Duration of the speaker tone in ms

// MISCELLANEOUS VARIABLES

// Error light
unsigned long ErrorLightDuration;  // how long ErrorLightPin stays HIGH after an error (ms); 0 = disabled

// Penalty durations
unsigned long ENLPenaltyDuration;
unsigned long CuePenaltyDuration;

// Speaker mode
bool cuesInstructed;  // false = speaker left and right both produce cue (SpeakerPinL & SpeakerPinR); true = left cue on SpeakerPinL, right cue on SpeakerPinR

// Keep track of session
bool sessionStart;  // if true, session is running
bool sessionEnd;    // if true, syncPulse will stay low

// end of task through Matlab GUI
int maxOmissions;  // max number of consecutive omissions
int maxTimeout;    // max duration of task in min
int maxTrials;     // max total trials before session end - Matlab GUI determines
int maxRewards;    // max total rewards before session end - Matlab GUI determines

// Send trial start pulse
bool sendTrialStartTTL = false;             // if true, start pulse
bool trialStartPulseActive = false;         // check if we are triggering a pulse
unsigned long TrialStartTTLDuration = 500;  // TTL start trial high (ms)


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Define counters -------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Sync pulses
unsigned long TimerSync = 0;             //timer for non-periodic sync pulse
unsigned long SyncPulseInterval = 1000;  //delay to start sync pulse after start task
int SyncPulseStatus = 0;                 //current sync signal status
unsigned long TrialStartTimer = 0;       //timer for trial start TTL

// External sync pulse (INPUT on ExtSyncPin) - polled, edge-detected
int ExtSyncState = 0;                    //last known external sync state (0 = low, 1 = high)
unsigned long ExtSyncRiseTime = 0;       //millis() at last detected rising edge
unsigned long ExtSyncLastFallTime = 0;   //millis() at last detected falling edge (for inter-pulse interval)
bool firstExtSync = true;                //true until the first rising edge of the session has been reported

// Session start timing (used for both first-ExtSync ITI and the post-'8' startup delay)
unsigned long SessionStartTime = 0;      //millis() at the moment '8' was received
unsigned long StartDelayTimer = 0;       //millis() reference for the START_DELAY state

// Trial stats
int TrialNum = 0;           //current trial number
int BlockNum = 0;           //current block number
int TrialInBlock = 0;       //current number of trials of this block
int Num_Reward = 0;         //current number of rewards
int CorrectTrialCount = 0;  // number of correct trials so far this block
int incorrectCounter = 0;
const int LEFT_BLOCK = 0;
const int RIGHT_BLOCK = 1;
int BlockType = LEFT_BLOCK;

// Block counters
int Num_ENLPenalty = 0;          // number of enl penalties per block
int Num_Omissions = 0;           // number of omissions per block
int freeRewardBlockCounter = 0;  // number of free rewards per block
int BlockRewardCounter = 0;      // number of rewards per block
int BlockNoRewardCounter = 0;    // number of no-rewards per block
int leftLickCounter = 0;         // number of left licks per block
int rightLickCounter = 0;        // number of right licks per block
int winStayCounter = 0;          // number of win-repeats per block
int loseSwitchCounter = 0;       // number of lose-switches per block
int winSwitchCounter = 0;        // number of win-switches per block
int loseStayCounter = 0;         // number of lose-repeats per block
int badTrialsCounter = 0;        // number of incorrect trials and omissions after a reward per block
bool lastTrial = true;           // outcome of last trial
bool firstTrial = true;          // outcome of first trial

// Trial structure
bool BlockSwitchAfterCorrectTrials;  // false (default): switch after N total trials; true: switch after N correct trials (applies to LLLR, Pavlovian, DR; ignored for 2ABT)
int BlockLength = 0;            // will pick from BlockLengths[]
unsigned long ITIDuration = 0;  // Current ITI - will pick from ITIDurationList
const int MAX_BLOCKS = 1024;
int TrialsUntilFirstRewardHistory[MAX_BLOCKS];
int TrialsUntilFirstReward = 0;

// Opto bookkeeping
bool OptoEnabled = false;               // master opto enable, no opto stims when false
bool optoTriggeredITI = false;          // are we stimulating
bool optoTriggeredCue = false;          // are we stimulating
bool optoTriggeredConsumption = false;  // are we stimulating (reward/correct trials)
bool optoTriggeredBlockSwitch = false;  // are we stimulating
bool optoTriggeredError = false;        // opto pending for error (incorrect lick) trial
bool optoTriggeredAnyOutcome = false;   // opto pending for any-outcome trial
bool optoTriggeredDelay = false;        // opto pending for DELAY state (DR task)
bool optoTriggeredStartCue = false;     // opto pending for START_CUE state (DR task)
bool duringOptoStim = false;
bool duringOptoStimDelay = false;
unsigned long optoStartTime = 0;
bool optoPulseActive = false;          // keep track of pulses
bool optoIPIActive = false;            // keep track of time in opto IPI
bool optoBlockDone = false;            // keep track if opto has occured in this block
bool optoTrialDone = false;            // keep track if opto has occured in this block
bool optoCueLick = false;              // keep track if animal licked during cue opto
unsigned long optoStartPulse = 0;      // keep track of start opto pulse
unsigned long optoTimeInPulse = 0;     // keep track of time in opto pulse
bool optoBlockFirstTrial = false;      // keep track of first trial after block switch
unsigned long optoIPI = 0;             // keep track of opto IPI
unsigned long timeInStim = 0;          // keep track of time in stimulation
unsigned long optoTaperStartTime = 0;  // keep track of start time in opto tapering
bool optoTaperActive = false;          // keep track of opto tapering off
unsigned long timeInTaper = 0;         // keep track of current time in opto tapering
unsigned long tmpOptoMod = 0;          // keep track of current opto modulation for tapering
bool manualOptoTestActive = false;     // true while the GUI manual opto test switch is ON

// Lick-to-start bookkeeping (Pavlovian)
bool correctLickAfterReward = false;   // true once the animal licked the block spout during CONSUMPTION and/or WAIT_LICK of a rewarded trial
bool rewardDeliveredThisTrial = false;  // true if the REWARD state was entered this trial (lick-to-start only required after a delivered reward)

// Error light bookkeeping
bool errorLightActive = false;          // is the error light currently on
unsigned long errorLightStartTime = 0;  // timestamp when error light was turned on

// Detection related
bool LeftLickOccuring = false;
bool RightLickOccuring = false;
unsigned long LeftLickStartTime = 0;   //timestamp of last L lick
unsigned long RightLickStartTime = 0;  //timestamp of last R lick

// Reward related
int LeftRewardButtonStatus = 0;      //left reward button status
int RightRewardButtonStatus = 0;     //right reward button status
unsigned long LeftRewardTimer = 0;   //timer for left reward button
unsigned long RightRewardTimer = 0;  //timer for right reward button

// Timestamp related
unsigned long LickStartTime = 0;  //timestamp for current lick
unsigned long ITIStartTime = 0;   //timestamp for beginning of ITI
unsigned long PenaltyStartTime = 0;
unsigned long CueStartTime = 0;          //timestamp for cue onset
unsigned long CueOffTime = 0;            //timestamp for cue off
unsigned long SelectionStartTime = 0;    //timestamp for selection window
unsigned long RewardStartTime = 0;       //timestamp for solenoid on (choice lick) or current timestamp after cue off
unsigned long RewardOffTime = 0;         //timestamp for solenoid off
unsigned long ConsumptionStartTime = 0;  //timestamp for Consumption
unsigned long CameraTrigStartTime = 0;   //timestamp for starting the session (used for triggering camera)
unsigned long DelayStartTime = 0;        //timestamp for DELAY state entry (DR task)
unsigned long StartCueStartTime = 0;     //timestamp for START_CUE state entry (DR task)
unsigned long ManualStartCueTime = 0;    //timestamp for manual start-cue test (GUI button)
bool manualStartCueActive = false;       //true while the GUI manual start-cue test is playing

// Buffer String for serial printing
String serialOutBuffer = "";
bool pausePrint = false;

// Loop timing metrics
unsigned long maxLoopTime_us = 0;
unsigned long cumulativeLoopTime_us = 0;
unsigned long loopCounter = 0;
unsigned long loopStartTime_us = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Define states ---------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Declare all possible states here: (order doesn't matter)
enum state {
  IDLE,
  START_DELAY,
  NEW_TRIAL,
  ITI,
  ENL_PENALTY,
  WAIT_LICK,         // Pavlovian lick-to-start: after a delivered reward, if no block-spout lick during CONSUMPTION, wait for it before NEW_TRIAL
  CUE,
  CUE_PENALTY,
  DELAY,             // DR task: delay between sample cue and go cue
  DELAY_PENALTY,     // DR task: penalty for licking during DELAY
  START_CUE,         // DR task: go-cue tone signaling start of selection
  STARTCUE_PENALTY,  // DR task: penalty for licking during START_CUE
  SELECTION,
  REWARD_DELAY,
  PRE_REWARD,
  FREE_REWARD,
  NO_REWARD,
  REWARD,
  CONSUMPTION
};
typedef enum state DualLickState;

// Initialize real time variables //
char SerialInput = '0';  //incoming serial data

// State-Machine related
DualLickState CurrentState = IDLE;  // MAIN behavior state variable for running behavior task
DualLickState NextState = IDLE;     // used for state transitions
int StartStopButtonStatus = 1;      // start-stop button status


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ----------------------------------- EEPROM settings --------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Arrays of variables to be stored. Must be in their respective variable type array. Max 4 KB
int* intParams[] = {  // variables to save to EEPROM - 4 bytes per value
  &minBlock,
  &maxBlock,
  &minBlockLeft,
  &maxBlockLeft,
  &minBlockRight,
  &maxBlockRight,
  &RewardLeftMin,
  &RewardLeftMax,
  &RewardLeftStepSize,
  &RewardRightMin,
  &RewardRightMax,
  &RewardRightStepSize,
  &fails2reward,
  &RewardProb,
  &spoutStart,
  &numCorrectTrialsBeforeOpto,
  &maxTimeout,
  &maxOmissions,
  &maxTrials,
  &maxRewards
};

bool* boolParams[] = {
  // 1 byte per value
  &taskLLLR,
  &task2ABT,
  &taskPavlovian,
  &optoContinuous,
  &OptoActiveDuringBlockSwitch,
  &OptoActiveDuringITI,
  &OptoActiveDuringCue,
  &OptoActiveDuringConsumption,
  &AllowMultiOptoStimITI,
  &AllowMultiOptoStimCue,
  &AllowMultiOptoStimBlock,
  &OptoEnabled,
  &ApplyOptoRandomLight,
  &cuesInstructed,
  &OptoModContinuous,
  &ENLEnabled,
  &OptoActiveDuringError,       // opto during error (incorrect lick) trials
  &OptoActiveDuringAnyOutcome,  // opto during any outcome (reward, error, or omission)
  &BlockSwitchAfterCorrectTrials,
  &taskDelayedResponse,         // Delayed-Response task mode (DR)
  &OptoActiveDuringDelay,       // opto during DELAY state (DR)
  &OptoActiveDuringStartCue,    // opto during START_CUE state (DR)
  &AllowMultiOptoStimDelay,     // re-arm delay opto on delay penalty (DR)
  &AllowMultiOptoStimStartCue,  // re-arm startcue opto on startcue penalty (DR)
  &PavlovianLickToStart         // Pavlovian lick-to-start: wait for block-spout lick after outcome, before next trial
};

unsigned long* ulongParams[] = {
  // 4 bytes per value
  &minITI,
  &maxITI,
  &ITIstep,
  &LeftCueFreq,
  &RightCueFreq,
  &ReactionTime,
  &CuePenaltyDuration,
  &RewardLeftCurrentSize,
  &RewardRightCurrentSize,
  &OptoStimProb,
  &OptoDelay,
  &OptoDuration,
  &OptoPulseDuration,
  &OptoFrequency,
  &OptoMod,
  &OptoTaperOff,
  &CorrectDuration,
  &IncorrectDuration,
  &ToneDuration,
  &ErrorLightDuration,  // how long ErrorLightPin stays HIGH after an error (ms)
  &TaskStartDelay,      // delay (s) after '8' before first NEW_TRIAL
  &RewardDelay,         // delay between correct lick and reward delivery (ms)
  &ENLPenaltyDuration,  // ENL penalty duration (ms) - APPEND ONLY (EEPROM addresses are positional)
  &DelayDuration,           // DR: DELAY state duration (ms)
  &StartCueDuration,        // DR: START_CUE state duration (ms)
  &StartCueFreq,            // DR: start cue frequency (Hz)
  &DelayPenaltyDuration,    // DR: delay penalty duration (ms; 0 disables)
  &StartCuePenaltyDuration  // DR: start cue penalty duration (ms; 0 disables)
};

const int numIntParams = sizeof(intParams) / sizeof(intParams[0]);
const int numBoolParams = sizeof(boolParams) / sizeof(boolParams[0]);
const int numUlongParams = sizeof(ulongParams) / sizeof(ulongParams[0]);


///////////////////////////////////////////////////////////////////////////////////////////////////////
// --------------------------- Start of setup and loop --------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // initialize serial communication
  Serial.begin(115200); // used to be 38400
  analogWriteResolution(8);  // 8-bit resolution for analogWrite (0-255)

  // get EERPOM variables
  getParams();

  // Ensure exactly one task mode is active (default to LLLR on first boot)
  if (!(taskLLLR || task2ABT || taskPavlovian || taskDelayedResponse)) {
    taskLLLR             = true;
    task2ABT             = false;
    taskPavlovian        = false;
    taskDelayedResponse  = false;
  }

  // define pin states
  pinMode(SyncPin, OUTPUT);
  pinMode(ExtSyncPin, INPUT_PULLDOWN);  // external sync TTL input (idle low, 3V active); pulldown keeps line at GND when nothing is connected
  pinMode(LeftRewardPin, OUTPUT);
  pinMode(RightRewardPin, OUTPUT);
  pinMode(LeftRewardIndicatorPin, OUTPUT);
  pinMode(RightRewardIndicatorPin, OUTPUT);
  pinMode(LeftLickPin, INPUT_PULLDOWN);
  pinMode(RightLickPin, INPUT_PULLDOWN);
  pinMode(SpeakerPinL, OUTPUT);
  pinMode(SpeakerPinR, OUTPUT);
  pinMode(SpeakerTTLPin, OUTPUT);
  pinMode(LeftCueIndicatorPin, OUTPUT);
  pinMode(RightCueIndicatorPin, OUTPUT);
  pinMode(StartCueSpeakerPin, OUTPUT);     // DR task: start/go cue speaker
  pinMode(StartCueIndicatorPin, OUTPUT);   // DR task: start/go cue TTL copy
  pinMode(CameraTriggerPin, OUTPUT);
  pinMode(StartStopIndicatorPin, OUTPUT);  //output pin for start and end of task
  pinMode(OptoPin, OUTPUT);                //opto pin
  pinMode(OptoTTLPin, OUTPUT);             //opto reporter pin
  pinMode(OptoModPin, OUTPUT);             //opto modulation pin
  pinMode(ITIIndicatorPin, OUTPUT);
  pinMode(DelayIndicatorPin, OUTPUT);
  pinMode(TrialStartPin, OUTPUT);
  pinMode(ErrorLightPin, OUTPUT);
  if (buttonsAreWiredUp) {
    pinMode(StartStopButtonPin, INPUT);
    pinMode(LeftRewardButtonPin, INPUT);
    pinMode(RightRewardButtonPin, INPUT);
  }
  pinMode(ExtSyncReportPin, OUTPUT);

  // attachInterrupt(digitalPinToInterrupt(RightLickPin), LICK_OFF, RISING);
  // attachInterrupt(digitalPinToInterrupt(LeftLickPin), LICK_ON, FALLING);
  // attachInterrupt(digitalPinToInterrupt(RightRewardButtonPin), DeliverReward, RISING);

  CameraTrigStartTime = millis();
  CurrentState = IDLE;
  SyncPulseStatus = 0;

  digitalWrite(SyncPin, LOW);
  digitalWrite(LeftRewardPin, LOW);
  digitalWrite(RightRewardPin, LOW);
  digitalWrite(LeftRewardIndicatorPin, LOW);
  digitalWrite(RightRewardIndicatorPin, LOW);
  digitalWrite(ITIIndicatorPin, LOW);
  digitalWrite(DelayIndicatorPin, LOW);
  analogWrite(SpeakerPinL, 0);
  analogWrite(SpeakerPinR, 0);
  digitalWrite(SpeakerTTLPin, LOW);
  digitalWrite(CameraTriggerPin, LOW);
  digitalWrite(StartStopIndicatorPin, LOW);
  digitalWrite(OptoPin, LOW);
  digitalWrite(OptoTTLPin, LOW);
  digitalWrite(OptoModPin, LOW);
  digitalWrite(TrialStartPin, LOW);
  digitalWrite(ErrorLightPin, LOW);
  digitalWrite(ExtSyncReportPin, LOW);
  analogWrite(StartCueSpeakerPin, 0);      // DR task: ensure start cue speaker is off
  digitalWrite(StartCueIndicatorPin, LOW); // DR task: ensure start cue indicator is off
  randomSeed(analogRead(3));

  // print task start menu - use serial.print since it won't interfere with the task
  Serial.println("-----------------------------------------------------------------");
  Serial.println("Manual check: 1 -> L-reward; 2 -> R-reward;");
  // Serial.println("Opto shutter: 3 -> blue stim on; 4 -> blue stim off");
  // Serial.println("OptoStim: 5 -> enable; 6 -> disable");
  Serial.println("Trial start/stop: 8 -> start; 9 -> end");
  // Serial.println("Timing Debug Info: 3    Status: 4");
  Serial.println("-----------------------------------------------------------------");
  Serial.println("");
  Serial.println("");
  printDataHeader();              // print data  headers
  serialOutBuffer.reserve(1024);  // pre-allocate to larger size than we'll need
  loopStartTime_us = micros();

  fillRewardProbList();  // called here for an un-changeable probability
}


void loop() {
  checkSerialInput();         // Needed for non-parameter commands - MUST run before updateStateMachine so that '9' sets NextState=IDLE before any state transition fires
  updateSyncPulses();         // generate and send sync pulses
  updateExtSyncDetection();   // detect external sync TTL edges on ExtSyncPin
  lickDetection();            // detect licks
  updateStateMachine();       // keep task running
  updateCameraTrigger();      // trigger camera recording
  updateOptoStim();           // allow opto stim
  updateSerialOutput();       // communicate with usb (matlab)
  randomLightFunction();      // random distraction light for opto
  turnOffManualCue();         // allows to turn cue off when manually triggering rewards
  sendTrialStartPulse();      // send start trial ttl

  // loop timing stats
  unsigned long now_us = micros();
  loopCounter++;
  cumulativeLoopTime_us += (now_us - loopStartTime_us);
  maxLoopTime_us = max(maxLoopTime_us, now_us - loopStartTime_us);
  loopStartTime_us = now_us;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------- State machine LLLR task (dual-lick task) --------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void updateStateMachine() {
  // if we just entered a new state, set <enteredNewState> to true, and print out state info
  bool enteredNewState = false;
  if (NextState != CurrentState) {
    enteredNewState = true;
    CurrentState = NextState;
    outputTrialState();
  }

  //   ==============   STATE MACHINE LOGIC   ==============

  //   ----------------------------   IDLE   ----------------------------
  // - wait until StartStop button pushed
  if (CurrentState == IDLE) {
    if (enteredNewState) {
      // reset all trial variables and hardware
      TrialNum = 0;           //current trial number
      BlockNum = 0;           //current block number
      Num_Reward = 0;         // current number of rewards
      TrialInBlock = 0;       //current number of trials of this block
      CorrectTrialCount = 0;  // number of correct trials so far this block
      incorrectCounter = 0;
      Num_ENLPenalty = 0;
      Num_Omissions = 0;
      failCounter = 0;  // counter for fails until reward
      freeRewardCounter = 0;
      freeRewardBlockCounter = 0;
      BlockRewardCounter = 0;
      BlockNoRewardCounter = 0;
      leftLickCounter = 0;
      rightLickCounter = 0;
      winStayCounter = 0;
      loseSwitchCounter = 0;
      winSwitchCounter = 0;
      loseStayCounter = 0;
      badTrialsCounter = 0;
      lastTrial = true;

      // turn off any signal
      BlockLength = 0;  // will pick from BlockLengths[]
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);
      digitalWrite(ITIIndicatorPin, LOW);
      digitalWrite(DelayIndicatorPin, LOW);
      analogWrite(SpeakerPinL, 0);
      analogWrite(SpeakerPinR, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      analogWrite(StartCueSpeakerPin, 0);       // DR task: stop start cue speaker
      digitalWrite(StartCueIndicatorPin, LOW);  // DR task: stop start cue indicator
      digitalWrite(CameraTriggerPin, LOW);
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
      digitalWrite(ErrorLightPin, LOW);
      errorLightActive = false;
      manualStartCueActive = false;             // ensure manual start cue test is cleared on idle
      correctLickAfterReward = false;           // Pavlovian lick-to-start
      rewardDeliveredThisTrial = false;         // Pavlovian lick-to-start
    }

    //   ----------------------------   START_DELAY   ----------------------------
    // - Pre-task wait window. sessionStart is already true, so internal sync, ExtSync
    //   detection, lick detection, camera trigger, etc. all run as normal. We just hold
    //   off transitioning to NEW_TRIAL until TaskStartDelay seconds have elapsed since '8'.
  } else if (CurrentState == START_DELAY) {

    // start task when delay has passed
    if (millis() - StartDelayTimer >= TaskStartDelay * 1000UL) {
      NextState = NEW_TRIAL;
    }

    //   ----------------------------   NEW_TRIAL   ----------------------------
    // - Update trial/block, Set trial-specific variables (ITI duration, opto stim)
  } else if (CurrentState == NEW_TRIAL) {

    //indicate new trial
    sendTrialStartTTL = true;  // start trial start TTL

    // stop any cue/delay/start-cue opto that might still be running into a new trial
    if (OptoActiveDuringCue == true || OptoActiveDuringDelay == true || OptoActiveDuringStartCue == true) {
      optoCueLick = true;  // potential lick during cue/delay/start-cue opto
    }

    // reset fail counter
    if (fails2reward > 0 && failCounter >= fails2reward) {  // Check if fails until reward threshold has been met, this is here for omissions
      // give a free reward
      NextState = FREE_REWARD;
      failCounter = 0;
      freeRewardCounter += 1;
      freeRewardBlockCounter += 1;
      // write data to matlab gui - free reward
      serialOutBuffer += "Free Rewards";
      serialOutBuffer += '\t';  // add a tab to separate values
      serialOutBuffer += freeRewardCounter;
      serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

    } else {  // stop any cue and reset ITIStartTime
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);
      analogWrite(SpeakerPinL, 0);
      analogWrite(SpeakerPinR, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      analogWrite(StartCueSpeakerPin, 0);        // DR task: also clear start cue speaker
      digitalWrite(StartCueIndicatorPin, LOW);   // DR task: also clear start cue indicator

      ITIStartTime = millis();
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(ITIIndicatorPin, HIGH);

      int randomIdx = random(0, NumITIDurations);
      ITIDuration = ITIDurationList[randomIdx];

      // new trial started
      incrementTrial();
      NextState = ITI;
    }

    //   ----------------------------   ITI   ----------------------------
    // - wait for full ITIDuration (potentially with no licks enforced)
  } else if (CurrentState == ITI) {

    // new state
    if (enteredNewState) {

      // keep track of ITI duration
      ITIStartTime = millis();
      digitalWrite(ITIIndicatorPin, HIGH);

      // check for opto stim first trial after block switch
      if (optoTriggeredBlockSwitch == true) {
        // ENL penalty so repeat Block switch opto
        startOptoStim();
        outputEvent("Opto_blockSwitch_p", millis(), OptoDuration);
      } else if ((OptoActiveDuringBlockSwitch == true) && (optoBlockFirstTrial == true) && (optoTrialDone == false)) {
        // first trial ITI after block switch (with appropiate prob.)
        if (random(100) < OptoStimProb) {
          optoTriggeredBlockSwitch = true;
          optoBlockFirstTrial = false;
          optoTrialDone = true;
          startOptoStim();
          outputEvent("Opto_blockSwitch", millis(), OptoDuration);
          if (!AllowMultiOptoStimBlock) {
            optoBlockDone = true;
          }
        }
      }

      // check for opto stim during the current ITI
      if (optoTriggeredITI == true) {
        // ENL penalty so repeat ITI opto
        startOptoStim();
        outputEvent("Opto_ENL_p", millis(), OptoDuration);
      } else if ((OptoActiveDuringITI == true) && (optoBlockDone == false) && (firstTrial == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTriggeredITI = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_ITI", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if ((CorrectTrialCount >= numCorrectTrialsBeforeOpto) && (lastTrial == true) && (optoTrialDone == false)) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTriggeredITI = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_ITI", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }
    }

    if (ENLEnabled && (LeftLickOccuring || RightLickOccuring)) {
      digitalWrite(ITIIndicatorPin, LOW);
      // check if we need to stimulate ITI again this trial
      if ((AllowMultiOptoStimITI == true) && (optoTriggeredITI == true)) {
        optoTriggeredITI = true;  // was already true
      } else {
        optoTriggeredITI = false;  // avoid another stim
      }
      // check if we need to stimulate BlockSwitch again this trial
      if ((AllowMultiOptoStimBlock == true) && (optoTriggeredBlockSwitch == true)) {
        optoTriggeredBlockSwitch = true;
      } else {
        optoTriggeredBlockSwitch = false;
      }

      // go to next state
      NextState = ENL_PENALTY;
    }

    // end of ITI period
    if (millis() - ITIStartTime >= ITIDuration) {
      digitalWrite(ITIIndicatorPin, LOW);
      optoTriggeredBlockSwitch = false;
      optoBlockFirstTrial = false;
      NextState = CUE;
    }

    //   ----------------------------   ENL_PENALTY   ----------------------------
    // - wait ENLPenaltyDuration, then enter ITI
  } else if (CurrentState == ENL_PENALTY) {

    // reset OptoTaperActive to make sure we could stimulate again if needed
    optoTaperActive = false;

    // new state
    if (enteredNewState) {
      PenaltyStartTime = millis();
      Num_ENLPenalty += 1;
    }
    if (millis() - PenaltyStartTime >= ENLPenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        NextState = ITI;
      }
    }

    //   ----------------------------   WAIT_LICK (Pavlovian lick-to-start)   ----------------------------
    // - only reached when taskPavlovian && PavlovianLickToStart, after CONSUMPTION of a DELIVERED reward,
    //   AND the animal did NOT yet lick the block spout during CONSUMPTION
    // - wait (indefinitely) for a lick on the current block's spout (left block -> left spout, right block -> right spout)
    // - licks on the other spout are recorded (lickDetection) but ignored here
    // - once that lick ENDS we go to NEW_TRIAL -> ITI/ENL (so the lick can't trigger an ENL penalty)
  } else if (CurrentState == WAIT_LICK) {

    // track correct (block-spout) lick
    trackCorrectLickAfterReward();

    // correct lick made and it has ended -> start new trial
    bool blockSpoutLicking = (BlockType == LEFT_BLOCK) ? LeftLickOccuring : RightLickOccuring;
    if (correctLickAfterReward && !blockSpoutLicking) {
      rewardDeliveredThisTrial = false;
      correctLickAfterReward = false;
      NextState = NEW_TRIAL;
    }

    //   ----------------------------   CUE   ----------------------------
    // - play tone
    // - wait for full CueDuration with no licks
  } else if (CurrentState == CUE) {

    // new state
    if (enteredNewState) {
      CueStartTime = millis();

      // check for opto stim during the cue
      if (optoTriggeredCue == true) {
        // Cue penalty so repeat Cue opto
        startOptoStim();
        outputEvent("Opto_cue_p", millis(), OptoDuration);
      } else if ((OptoActiveDuringCue == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTriggeredCue = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_cue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTriggeredCue = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_cue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }

      // present cue
      if (BlockType == LEFT_BLOCK) {
        // left cue: drive both pins in one-speaker mode, SpeakerPinL only in two-speaker mode
        if (cuesInstructed) {
          analogWriteFrequency(SpeakerPinL, LeftCueFreq);
          analogWrite(SpeakerPinL, 128);
          analogWrite(SpeakerPinR, 0);
        } else {
          analogWriteFrequency(SpeakerPinL, LeftCueFreq);
          analogWrite(SpeakerPinL, 128);
          analogWriteFrequency(SpeakerPinR, LeftCueFreq);
          analogWrite(SpeakerPinR, 128);
        }
        digitalWrite(SpeakerTTLPin, HIGH);
        digitalWrite(LeftCueIndicatorPin, HIGH);
      } else {
        // right cue: drive both pins in one-speaker mode, SpeakerPinR only in two-speaker mode
        if (cuesInstructed) {
          analogWrite(SpeakerPinL, 0);
          analogWriteFrequency(SpeakerPinR, RightCueFreq);
          analogWrite(SpeakerPinR, 128);
        } else {
          analogWriteFrequency(SpeakerPinL, RightCueFreq);
          analogWrite(SpeakerPinL, 128);
          analogWriteFrequency(SpeakerPinR, RightCueFreq);
          analogWrite(SpeakerPinR, 128);
        }
        digitalWrite(SpeakerTTLPin, HIGH);
        digitalWrite(RightCueIndicatorPin, HIGH);
      }
    }

    // what to do after a lick (only penalise if CuePenaltyDuration > 0)
    if (CuePenaltyDuration > 0 && (LeftLickOccuring || RightLickOccuring)) {
      digitalWrite(LeftCueIndicatorPin, LOW);
      digitalWrite(RightCueIndicatorPin, LOW);
      analogWrite(SpeakerPinL, 0);
      analogWrite(SpeakerPinR, 0);
      digitalWrite(SpeakerTTLPin, LOW);

      // stop cue opto - animal made a lick
      if (OptoActiveDuringCue == true) {
        optoCueLick = true;  // potential lick during cue opto
      }

      // check if we need to stimulate Cue again this trial
      if ((AllowMultiOptoStimCue == true) && (optoTriggeredCue == true)) {
        optoTriggeredCue = true;  // was already true
      } else {
        optoTriggeredCue = false;  // avoid another stim
      }

      NextState = CUE_PENALTY;
    }

    // end of cue period
    if (millis() - CueStartTime >= ToneDuration) {
      digitalWrite(LeftCueIndicatorPin, LOW);
      digitalWrite(RightCueIndicatorPin, LOW);
      analogWrite(SpeakerPinL, 0);
      analogWrite(SpeakerPinR, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      firstTrial = false;        // for sure not first trial anymore
      optoTriggeredITI = false;  // end of ITI and cue, don't repeat opto next trial
      optoTriggeredCue = false;  // end of trial, don't repeat opto next trial
      if (taskPavlovian) {
        // Pavlovian: treat as correct, reward according to RewardProb, skip animal response
        // Note: giveReward was already drawn at trial start in incrementTrial() and reported
        // to the MATLAB GUI as "Reward prob trial". Do NOT redraw here, otherwise the GUI's
        // reward-plotting state will disagree with what the animal actually receives.
        correctSelection = true;
        CorrectTrialCount++;
        lastTrial = true;
        serialOutBuffer += "Reward in block";
        serialOutBuffer += '\t';
        serialOutBuffer += CorrectTrialCount;
        serialOutBuffer += '\n';
        NextState = REWARD_DELAY;
      } else if (taskDelayedResponse) {
        // DR task: insert DELAY (and START_CUE) between sample cue and selection
        NextState = DELAY;
      } else {
        NextState = SELECTION;
      }
    }

    //   ----------------------------   CUE_PENALTY   ----------------------------
    // - wait CuePenaltyDuration, then enter ITI
  } else if (CurrentState == CUE_PENALTY) {

    // new state
    if (enteredNewState) {

      // keep track of cue penalty time
      PenaltyStartTime = millis();
    }

    // go back to ITI and potentially stimulate again
    if (millis() - PenaltyStartTime >= CuePenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        optoCueLick = false;  // turn off optoCueLick so we might be able to stimulate again
        NextState = ITI;
      }
    }

    //   ----------------------------   DELAY (DR task)   ----------------------------
    // - silent waiting period between sample cue and go cue
    // - licks recorded; if DelayPenaltyDuration > 0, lick triggers DELAY_PENALTY -> ITI
    // - opto runs through full duration; lick (with penalty) terminates opto via optoCueLick
  } else if (CurrentState == DELAY) {

    // new state
    if (enteredNewState) {
      DelayStartTime = millis();
      digitalWrite(DelayIndicatorPin, HIGH);

      // check for opto stim during the delay
      if (optoTriggeredDelay == true) {
        // Delay penalty so repeat Delay opto
        startOptoStim();
        outputEvent("Opto_delay_p", millis(), OptoDuration);
      } else if ((OptoActiveDuringDelay == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTriggeredDelay = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_delay", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          // We've had the target number of correct trials, so activate optoStim (with appropriate prob.)
          if (random(100) < OptoStimProb) {
            optoTriggeredDelay = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_delay", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }
    }

    // what to do after a lick during DELAY (only penalise if DelayPenaltyDuration > 0)
    if (DelayPenaltyDuration > 0 && (LeftLickOccuring || RightLickOccuring)) {

      // stop delay opto - animal made a lick (mirrors cue opto behavior)
      if (OptoActiveDuringDelay == true) {
        optoCueLick = true;  // shared "stop opto on lick" flag
      }

      // check if we need to stimulate Delay again this trial
      if ((AllowMultiOptoStimDelay == true) && (optoTriggeredDelay == true)) {
        optoTriggeredDelay = true;  // was already true
      } else {
        optoTriggeredDelay = false;  // avoid another stim
      }

      digitalWrite(DelayIndicatorPin, LOW);
      NextState = DELAY_PENALTY;
    }

    // end of delay period
    if (millis() - DelayStartTime >= DelayDuration) {
      optoTriggeredDelay = false;  // end of delay successfully, don't repeat opto next trial
      digitalWrite(DelayIndicatorPin, LOW);
      NextState = START_CUE;
    }

    //   ----------------------------   DELAY_PENALTY (DR task)   ----------------------------
    // - wait DelayPenaltyDuration, then re-enter ITI (full trial restart)
  } else if (CurrentState == DELAY_PENALTY) {

    // new state
    if (enteredNewState) {
      PenaltyStartTime = millis();
    }

    // go back to ITI and potentially stimulate again
    if (millis() - PenaltyStartTime >= DelayPenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        optoCueLick = false;  // reset so we can opto again on next opportunity
        NextState = ITI;
      }
    }

    //   ----------------------------   START_CUE (DR task)   ----------------------------
    // - play go-cue tone on dedicated StartCueSpeakerPin / StartCueIndicatorPin
    // - lick during START_CUE with StartCuePenaltyDuration > 0 -> STARTCUE_PENALTY -> ITI
    // - opto mimics current cue opto: lick terminates opto via optoCueLick
  } else if (CurrentState == START_CUE) {

    // new state
    if (enteredNewState) {
      StartCueStartTime = millis();

      // check for opto stim during the start cue
      if (optoTriggeredStartCue == true) {
        // StartCue penalty so repeat StartCue opto
        startOptoStim();
        outputEvent("Opto_startCue_p", millis(), OptoDuration);
      } else if ((OptoActiveDuringStartCue == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTriggeredStartCue = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_startCue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTriggeredStartCue = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_startCue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }

      // play start/go cue tone on its dedicated speaker
      analogWriteFrequency(StartCueSpeakerPin, StartCueFreq);
      analogWrite(StartCueSpeakerPin, 128);
      digitalWrite(StartCueIndicatorPin, HIGH);
    }

    // what to do after a lick during START_CUE (only penalise if StartCuePenaltyDuration > 0)
    if (StartCuePenaltyDuration > 0 && (LeftLickOccuring || RightLickOccuring)) {
      analogWrite(StartCueSpeakerPin, 0);
      digitalWrite(StartCueIndicatorPin, LOW);

      // stop start-cue opto - animal made a lick
      if (OptoActiveDuringStartCue == true) {
        optoCueLick = true;
      }

      // check if we need to stimulate StartCue again this trial
      if ((AllowMultiOptoStimStartCue == true) && (optoTriggeredStartCue == true)) {
        optoTriggeredStartCue = true;
      } else {
        optoTriggeredStartCue = false;
      }

      NextState = STARTCUE_PENALTY;
    }

    // end of start cue period
    if (millis() - StartCueStartTime >= StartCueDuration) {
      analogWrite(StartCueSpeakerPin, 0);
      digitalWrite(StartCueIndicatorPin, LOW);
      optoTriggeredStartCue = false;  // end of start cue successfully, don't repeat opto next trial
      NextState = SELECTION;
    }

    //   ----------------------------   STARTCUE_PENALTY (DR task)   ----------------------------
    // - wait StartCuePenaltyDuration, then re-enter ITI (full trial restart)
  } else if (CurrentState == STARTCUE_PENALTY) {

    // new state
    if (enteredNewState) {
      PenaltyStartTime = millis();
    }

    // go back to ITI and potentially stimulate again
    if (millis() - PenaltyStartTime >= StartCuePenaltyDuration) {
      if (!LeftLickOccuring && !RightLickOccuring) {
        optoCueLick = false;
        NextState = ITI;
      }
    }

    //   ----------------------------   REWARD_DELAY   ----------------------------
    // - wait RewardDelay ms, then deliver reward; licks are ignored (no penalty)
  } else if (CurrentState == REWARD_DELAY) {

    if (enteredNewState) {
      PenaltyStartTime = millis();
    }

    if (millis() - PenaltyStartTime >= RewardDelay) {
      NextState = PRE_REWARD;
    }

    //   ----------------------------   SELECTION   ----------------------------
    // - wait for lick or timeout
    // - initiate reward delivery if correct lick
    // - move on to CONSUMPTION upon lick or timeout
  } else if (CurrentState == SELECTION) {

    // new state
    if (enteredNewState) {
      SelectionStartTime = millis();
    }

    // reset if trial was correct or not
    correctSelection = false;

    // // check for opto stim in next ITI independent of trial outcome (OptoActiveDuringITI==true and numCorrectTrialsBeforeOpto==0)
    // if ((OptoActiveDuringITI == true) && (numCorrectTrialsBeforeOpto == 0) && (optoBLockDone == false)) {
    //   // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
    //   if (random(100) < OptoStimProb) {
    //     optoTriggeredITI = true;
    //     if (!AllowMultiOptoStimBlock) {
    //       optoBlockDone = true;
    //     }
    //   }
    // }

    // licks occurred
    if (LeftLickOccuring && BlockType == LEFT_BLOCK) {
      correctSelection = true;
      outputEvent("#_CORRECT_LEFT", millis(), 0);
    } else if (RightLickOccuring && BlockType == RIGHT_BLOCK) {
      correctSelection = true;
      outputEvent("#_CORRECT_RIGHT", millis(), 0);
    }

    // trial outcome
    if (correctSelection) {
      // CORRECT TRIAL
      failCounter = 0;
      CorrectTrialCount = CorrectTrialCount + 1;

      // block switch: 2 rewards & block switch = winSwitch
      if (lastTrial && TrialsUntilFirstReward == 0) {
        winSwitchCounter++;
        // win repeat if previous trials was not a block switch
      } else if (lastTrial) {
        winStayCounter++;
        //
      } else {
        loseSwitchCounter++;
      }
      lastTrial = true;

      // write data to matlab gui - rewards in block
      serialOutBuffer += "Reward in block";
      serialOutBuffer += '\t';  // add a tab to separate values
      serialOutBuffer += CorrectTrialCount;
      serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

      // check for opto stim during correct selection (reward)
      if ((OptoActiveDuringConsumption == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          // just stimulate whenever possible
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredConsumption = true;
            // outputEvent("Opto Reward", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          // We've had the target number of corect trials, so activate optoStim (with appropiate prob.)
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredConsumption = true;
            // outputEvent("Opto Reward", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) {
              optoBlockDone = true;
            }
          }
        }
      }

      // any-outcome opto (correct trial counts as an outcome)
      if ((OptoActiveDuringAnyOutcome == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }

      // go to reward delay, then pre reward
      NextState = REWARD_DELAY;

    } else if (ReactionTime > 0 && millis() - SelectionStartTime >= ReactionTime) {
      // TIMEOUT
      outputEvent("#_TIMEOUT", millis(), 0);
      failCounter += 1;  // Count omission as a fail
      Num_Omissions += 1;
      if (TrialsUntilFirstReward != 0) {
        badTrialsCounter++;
      }

      // any-outcome opto (omission counts as an outcome)
      if ((OptoActiveDuringAnyOutcome == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }

      // go to consumption directly (omission)
      NextState = CONSUMPTION;  // NEW_TRIAL

    } else if (LeftLickOccuring && BlockType != LEFT_BLOCK) {
      // INCORRECT SELECTION
      outputEvent("#_INCORRECT_LEFT", millis(), 0);
      failCounter += 1;  // Count incorrect lick as a fail
      incorrectCounter += 1;
      if (lastTrial) {
        if (TrialsUntilFirstReward == 0) {
          winStayCounter++;
        } else {
          winSwitchCounter++;
          badTrialsCounter++;
        }
      } else {
        loseStayCounter++;
        if (TrialsUntilFirstReward != 0) {
          badTrialsCounter++;
        }
      }
      lastTrial = false;

      // error opto
      if ((OptoActiveDuringError == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredError = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredError = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }
      // any-outcome opto
      if ((OptoActiveDuringAnyOutcome == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }
      // error light
      if (ErrorLightDuration > 0) {
        digitalWrite(ErrorLightPin, HIGH);
        errorLightActive = true;
        errorLightStartTime = millis();
      }

      // go to reward delay (PRE_REWARD will decide REWARD vs NO_REWARD via giveRewardIncorrect)
      NextState = REWARD_DELAY;

    } else if (RightLickOccuring) {
      // INCORRECT SELECTION
      outputEvent("#_INCORRECT_RIGHT", millis(), 0);
      failCounter += 1;  // Count incorrect lick as a fail
      incorrectCounter += 1;
      if (lastTrial) {
        if (TrialsUntilFirstReward == 0) {
          winStayCounter++;
        } else {
          winSwitchCounter++;
          badTrialsCounter++;
        }
      } else {
        loseStayCounter++;
        if (TrialsUntilFirstReward != 0) {
          badTrialsCounter++;
        }
      }
      lastTrial = false;

      // error opto
      if ((OptoActiveDuringError == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredError = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredError = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }
      // any-outcome opto
      if ((OptoActiveDuringAnyOutcome == true) && (optoBlockDone == false) && (optoTrialDone == false)) {
        if (numCorrectTrialsBeforeOpto == 0) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        } else if (CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTrialDone = true;
            optoTriggeredAnyOutcome = true;
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }
      // error light
      if (ErrorLightDuration > 0) {
        digitalWrite(ErrorLightPin, HIGH);
        errorLightActive = true;
        errorLightStartTime = millis();
      }

      // go to reward delay (PRE_REWARD will decide REWARD vs NO_REWARD via giveRewardIncorrect)
      NextState = REWARD_DELAY;
    }


    //   ----------------------------   PRE_REWARD   ----------------------------
    // - wait for lick to end before starting reward delivery
  } else if (CurrentState == PRE_REWARD) {

    // new state
    if (enteredNewState) {

      // stop cue opto - animal made a lick
      if (OptoActiveDuringCue == true) {
        optoCueLick = true;  // potential lick during cue opto
      }

      // reset: set to true only if REWARD is actually entered this trial (Pavlovian lick-to-start)
      rewardDeliveredThisTrial = false;
      correctLickAfterReward = false;

      // define next state: use giveReward for correct, giveRewardIncorrect for incorrect spout
      bool shouldReward = correctSelection ? giveReward : giveRewardIncorrect;
      if (shouldReward) {
        NextState = REWARD;
        BlockRewardCounter += 1;
      } else {
        NextState = NO_REWARD;
        BlockNoRewardCounter += 1;
      }
    }

    //   ----------------------------  NO REWARD   ----------------------------
    // - if incorrect response (reward probability = 1.00) OR
    //   correct lick made but no reward given (reward probability < 1.00) OR
    //   incorrect lick 
  } else if (CurrentState == NO_REWARD) {

    // no reward
    if (BlockType == LEFT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardLeftCurrentSize) {
        digitalWrite(LeftRewardPin, LOW);
        digitalWrite(LeftRewardIndicatorPin, LOW);

        // unfortunately no reward, go to consumption
        NextState = CONSUMPTION;
      }
    } else if (BlockType == RIGHT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardRightCurrentSize) {
        digitalWrite(RightRewardPin, LOW);
        digitalWrite(RightRewardIndicatorPin, LOW);

        // unfortunately no reward, go to consumption
        NextState = CONSUMPTION;
      }
    }

    //   ----------------------------  FREE REWARD   ----------------------------
    // - open L or R reward valve for correct duration
  } else if (CurrentState == FREE_REWARD) {

    // new state
    if (enteredNewState) {
      // provide free reward
      if (BlockType == LEFT_BLOCK) {
        digitalWrite(LeftRewardPin, HIGH);
        digitalWrite(LeftRewardIndicatorPin, HIGH);
      } else {
        digitalWrite(RightRewardPin, HIGH);
        digitalWrite(RightRewardIndicatorPin, HIGH);
      }
      RewardStartTime = millis();
    }

    // stop free reward
    if (BlockType == LEFT_BLOCK && (millis() - RewardStartTime) > RewardLeftCurrentSize) {
      digitalWrite(LeftRewardPin, LOW);
      digitalWrite(LeftRewardIndicatorPin, LOW);

      // back to consumption for the animal to consume reward
      NextState = CONSUMPTION;
    } else if (BlockType == RIGHT_BLOCK && (millis() - RewardStartTime) > RewardRightCurrentSize) {
      digitalWrite(RightRewardPin, LOW);
      digitalWrite(RightRewardIndicatorPin, LOW);

      // back to consumption for the animal to consume reward
      NextState = CONSUMPTION;
    }

    //   ----------------------------   REWARD   ----------------------------
  } else if (CurrentState == REWARD) {

    // new state
    if (enteredNewState) {
      // give reward
      if (BlockType == LEFT_BLOCK) {
        digitalWrite(LeftRewardPin, HIGH);
        digitalWrite(LeftRewardIndicatorPin, HIGH);
        Num_Reward = Num_Reward + 1;
      } else {
        digitalWrite(RightRewardPin, HIGH);
        digitalWrite(RightRewardIndicatorPin, HIGH);
        Num_Reward = Num_Reward + 1;
      }
      RewardStartTime = millis();
      rewardDeliveredThisTrial = true;  // Pavlovian lick-to-start: a reward was delivered this trial
      if (TrialsUntilFirstReward == 0) {
        TrialsUntilFirstReward = TrialInBlock;
        TrialsUntilFirstRewardHistory[BlockNum - 1] = TrialsUntilFirstReward;
      }
    }

    // stop reward
    if (BlockType == LEFT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardLeftCurrentSize) {
        // stop left reward
        digitalWrite(LeftRewardPin, LOW);
        digitalWrite(LeftRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
        // write data to matlab gui - total rewards
        serialOutBuffer += "Total Rewards";
        serialOutBuffer += '\t';  // add a tab to separate values
        serialOutBuffer += Num_Reward;
        serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
      }
    } else if (BlockType == RIGHT_BLOCK) {
      if ((millis() - RewardStartTime) > RewardRightCurrentSize) {
        // stop right reward
        digitalWrite(RightRewardPin, LOW);
        digitalWrite(RightRewardIndicatorPin, LOW);
        NextState = CONSUMPTION;
        // write data to matlab gui - total rewards
        serialOutBuffer += "Total Rewards";
        serialOutBuffer += '\t';  // add a tab to separate values
        serialOutBuffer += Num_Reward;
        serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
      }
    }

    // start consumption opto if needed
    if (optoTriggeredConsumption && OptoActiveDuringConsumption) {
      optoTriggeredConsumption = false;
      startOptoStim();
      outputEvent("Opto_consumption", millis(), OptoDuration);
    }

    //   ----------------------------   CONSUMPTION   ----------------------------
    // - wait for ConsumptionDuration
    // - turn off reward if necessary
  } else if (CurrentState == CONSUMPTION) {

    // new state
    if (enteredNewState) {

      // stop cue opto - animal made a lick
      if (OptoActiveDuringCue == true) {
        optoCueLick = true;  // potential lick during cue opto
      }

      // check how long we should stay in consumption state
      if (correctSelection) {
        ConsumptionDuration = CorrectDuration;
      } else {
        ConsumptionDuration = IncorrectDuration;
      }

      // start timer
      ConsumptionStartTime = millis();

      // start error opto if needed (incorrect lick trials)
      if (optoTriggeredError && OptoActiveDuringError) {
        optoTriggeredError = false;
        startOptoStim();
        outputEvent("Opto_error", millis(), OptoDuration);
      }
      // start any-outcome opto if needed (reward, error, or omission)
      if (optoTriggeredAnyOutcome && OptoActiveDuringAnyOutcome) {
        optoTriggeredAnyOutcome = false;
        startOptoStim();
        outputEvent("Opto_anyOutcome", millis(), OptoDuration);
      }
    }

    // Pavlovian lick-to-start: track correct (block-spout) lick during consumption of a delivered reward
    trackCorrectLickAfterReward();

    // check if we need to give a free reward
    if (millis() - ConsumptionStartTime >= ConsumptionDuration) {
      if (fails2reward > 0 && failCounter >= fails2reward) {  // Give free reward if threshold is met
        // give a free reward
        NextState = FREE_REWARD;
        failCounter = 0;
        freeRewardCounter += 1;
        freeRewardBlockCounter += 1;
        // write data to matlab gui - free rewards
        serialOutBuffer += "Free Rewards";
        serialOutBuffer += '\t';  // add a tab to separate values
        serialOutBuffer += freeRewardCounter;
        serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

      } else {

        // start new trial
        // Pavlovian lick-to-start: after a DELIVERED reward the animal must have licked the block spout
        // during CONSUMPTION; if not, wait for that lick in WAIT_LICK.
        // unrewarded trials (RewardProb < 100) go straight to the next trial
        if (taskPavlovian && PavlovianLickToStart && rewardDeliveredThisTrial && !correctLickAfterReward) {
          NextState = WAIT_LICK;
        } else {
          rewardDeliveredThisTrial = false;
          correctLickAfterReward = false;
          NextState = NEW_TRIAL;
        }
      }
    }

    //   ----------------------------   END of STATES   ----------------------------
  } else {

    // should never get here
    NextState = IDLE;
    digitalWrite(CameraTriggerPin, LOW);
  }

  // turn off error light after ErrorLightDuration (runs every cycle regardless of state)
  if (errorLightActive && (millis() - errorLightStartTime >= ErrorLightDuration)) {
    digitalWrite(ErrorLightPin, LOW);
    errorLightActive = false;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------------- Functions that are task settings ----------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Fill block lengths
void fillBlockLengths() {  // Fills BlockLengthsLeft and Right with all values in between min and max

  // fill blocks for LEFT
  numBlockLengthsLeft = 0;
  for (int i = minBlockLeft; i <= maxBlockLeft; i++) {
    BlockLengthsLeft[numBlockLengthsLeft] = i;
    numBlockLengthsLeft++;
  }

  // fill blocks for RIGHT
  numBlockLengthsRight = 0;
  for (int i = minBlockRight; i <= maxBlockRight; i++) {
    BlockLengthsRight[numBlockLengthsRight] = i;
    numBlockLengthsRight++;
  }
}

// generate ITI list
void fillITIDuration() {
  unsigned long currentValue = minITI;
  NumITIDurations = 0;
  for (int i = 0; i < 100; i++) {
    // Break if we exceed maxITI
    if (currentValue > maxITI) {
      break;
    }
    ITIDurationList[i] = currentValue;
    currentValue += ITIstep;
    NumITIDurations++;
  }
}
//unsigned long ITIDurationList[5] = {1000, 1250, 1500, 1750, 2000};

// Fill reward probability list
void fillRewardProbList() {
  for (int i = 0; i < RewardProb; i++) {
    RewardProbList[i] = 1;
  }
  for (int i = RewardProb; i < 100; i++) {
    RewardProbList[i] = 0;
  }
}

// Assign BlockType based on random number
void randomBlock() {
  int randomNum = random(0, 2);  // generates either 0 or 1
  if (randomNum == 0) {
    BlockType = LEFT_BLOCK;
  } else if (randomNum == 1) {
    BlockType = RIGHT_BLOCK;
  }
}

// Fill reward sizes left
void fillRewardLeftRange() {
  int currentValue = RewardLeftMin;
  numRewardLeftRange = 0;

  for (int i = 0; i < 100; i++) {
    if (currentValue > RewardLeftMax) {
      break;
    }

    RewardLeftRange[i] = currentValue;
    currentValue += RewardLeftStepSize;
    numRewardLeftRange++;
  }
}

// Fill reward sizes right
void fillRewardRightRange() {
  int currentValue = RewardRightMin;
  numRewardRightRange = 0;

  for (int i = 0; i < 100; i++) {
    if (currentValue > RewardRightMax) {
      break;
    }

    RewardRightRange[i] = currentValue;
    currentValue += RewardRightStepSize;
    numRewardRightRange++;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------- Functions that are related to EEPROM storage ----------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Save parameters to EEPROM. Can only be done 100,000 times
void saveParams() {
  int address = 0;

  // Save all int parameters
  for (int i = 0; i < numIntParams; i++) {
    EEPROM.put(address, *intParams[i]);  // update() compares the new and old values and only re-writes if they are different, this avoids excessive overwrites
    address += sizeof(int);
  }

  // Save all boolean parameters
  for (int i = 0; i < numBoolParams; i++) {
    EEPROM.update(address, *boolParams[i]);
    address += sizeof(bool);
  }

  // Save all unsigned long parameters
  for (int i = 0; i < numUlongParams; i++) {
    byte* pointer = (byte*)&(*ulongParams[i]);
    for (unsigned long j = 0; j < sizeof(unsigned long); j++) {
      EEPROM.update(address++, pointer[j]);
    }
  }
}

// Get parameters from EEPROM. No limit on how many times you can call this function
void getParams() {
  int address = 0;

  // Get all int parameters
  for (int i = 0; i < numIntParams; i++) {
    // load values
    //    *intParams[i] = EEPROM.read(address);
    EEPROM.get(address, *intParams[i]);
    // check if EEPROM is empty
    if (*intParams[i] == -1) {             // default uninitialized int
      *intParams[i] = 1;                   // set default value
      EEPROM.put(address, *intParams[i]);  // store in EEPROM
    }
    // update address value
    address += sizeof(int);
  }

  // Get all boolean parameters
  for (int i = 0; i < numBoolParams; i++) {
    // load values
    uint8_t raw = EEPROM.read(address);
    // check if EEPROM is empty
    if (raw == 0xFF) {                      // Uninitialized EEPROM
      *boolParams[i] = false;               // default to false
      EEPROM.put(address, *boolParams[i]);  // store in EEPROM
    } else {
      *boolParams[i] = raw != 0;  // treat non-zero as true
    }
    // update address value
    address += sizeof(bool);
  }

  // Get all unsigned long parameters
  for (int i = 0; i < numUlongParams; i++) {
    // load values
    EEPROM.get(address, *ulongParams[i]);
    // check if EEPROM is empty
    if (*ulongParams[i] == 4294967295UL) {   // max unsigned long (0xFFFFFFFF)
      *ulongParams[i] = 1;                   // set default value
      EEPROM.put(address, *ulongParams[i]);  // store in EEPROM
    }
    // update address value
    address += sizeof(unsigned long);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// -------------------------- Functions that are part of the task ---------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// turn off manual auditory cues because you cannot use "while" if combined with a reward delivery
void turnOffManualCue() {

  // Turn off auditory cues
  if ((millis() - CueStartTime) >= ToneDuration && (digitalRead(LeftCueIndicatorPin) == HIGH)) {
    analogWrite(SpeakerPinL, 0);
    analogWrite(SpeakerPinR, 0);
    digitalWrite(SpeakerTTLPin, LOW);
    digitalWrite(LeftCueIndicatorPin, LOW);
  }

  // Turn off auditory cues
  if ((millis() - CueStartTime) >= ToneDuration && (digitalRead(RightCueIndicatorPin) == HIGH)) {
    analogWrite(SpeakerPinL, 0);
    analogWrite(SpeakerPinR, 0);
    digitalWrite(SpeakerTTLPin, LOW);
    digitalWrite(RightCueIndicatorPin, LOW);
  }

  // Turn off manual start-cue test (DR task) after StartCueDuration has elapsed
  if (manualStartCueActive && (millis() - ManualStartCueTime) >= StartCueDuration) {
    analogWrite(StartCueSpeakerPin, 0);
    digitalWrite(StartCueIndicatorPin, LOW);
    manualStartCueActive = false;
  }
}

// send trial start TTL
void sendTrialStartPulse() {

  // start pulse
  if (sendTrialStartTTL && !trialStartPulseActive) {

    // turn ON
    trialStartPulseActive = true;
    TrialStartTimer = millis();
    digitalWrite(TrialStartPin, HIGH);

    // write data to matlab gui
    outputEvent("TrialStart_ON", millis(), TrialStartTTLDuration);
  }

  // stop pulse
  if (trialStartPulseActive) {
    if (millis() - TrialStartTimer >= TrialStartTTLDuration) {

      // turn OFF
      digitalWrite(TrialStartPin, LOW);
      trialStartPulseActive = false;
      sendTrialStartTTL = false;  // reset trigger

      // write data to matlab gui
      outputEvent("TrialStart_OFF", millis(), 0);
    }
  }
}

// detect all licks
void lickDetection() {
  // Spouts or buttons
  bool leftLickDetected;
  bool rightLickDetected;
  if (SpoutNotButton == true) {
    leftLickDetected = (digitalRead(LeftLickPin) == LOW);    // LOW for spouts, HIGH for buttons
    rightLickDetected = (digitalRead(RightLickPin) == LOW);  // LOW for spouts, HIGH for buttons
  } else {
    leftLickDetected = (digitalRead(LeftLickPin) == HIGH);    // LOW for spouts, HIGH for buttons
    rightLickDetected = (digitalRead(RightLickPin) == HIGH);  // LOW for spouts, HIGH for buttons
  }

  // LEFT
  if (leftLickDetected && !LeftLickOccuring) {
    // If start of new L lick detected:
    LeftLickOccuring = true;
    LeftLickStartTime = millis();
  } else if (!leftLickDetected && LeftLickOccuring) {
    // If end of L lick detected:
    LeftLickOccuring = false;
    unsigned long Lick_Duration = millis() - LeftLickStartTime;

    outputEvent("LickLeft", LeftLickStartTime, Lick_Duration);
    leftLickCounter += 1;
  }

  // RIGHT
  if (rightLickDetected && !RightLickOccuring) {
    // If start of new L lick detected:
    RightLickOccuring = true;
    RightLickStartTime = millis();
  } else if (!rightLickDetected && RightLickOccuring) {
    // If end of L lick detected:
    RightLickOccuring = false;
    unsigned long Lick_Duration = millis() - RightLickStartTime;

    outputEvent("LickRight", RightLickStartTime, Lick_Duration);
    rightLickCounter += 1;
  }
  // Lick Detection END //
}

// Pavlovian lick-to-start: during CONSUMPTION / WAIT_LICK of a rewarded trial, register the first lick
// on the current block's spout (left block -> left spout, right block -> right spout).
// A lick that is still ongoing when CONSUMPTION starts (e.g. started during REWARD) also counts.
void trackCorrectLickAfterReward() {
  if (!(taskPavlovian && PavlovianLickToStart && rewardDeliveredThisTrial) || correctLickAfterReward) {
    return;
  }
  bool blockSpoutLicking = (BlockType == LEFT_BLOCK) ? LeftLickOccuring : RightLickOccuring;
  if (blockSpoutLicking) {
    correctLickAfterReward = true;
    unsigned long lickOnset = (BlockType == LEFT_BLOCK) ? LeftLickStartTime : RightLickStartTime;
    outputEvent("CORRECT_LICK", lickOnset, 0);
  }
}

// Increment trial count and update block
void incrementTrial() {
  // Update trial number
  optoTrialDone = false;
  optoTriggeredConsumption = false;  // clear reward opto flag on new trial (prevents leak via NO_REWARD path)
  optoTriggeredError = false;        // clear error opto flag on new trial
  optoTriggeredAnyOutcome = false;   // clear any-outcome opto flag on new trial
  optoTriggeredDelay = false;        // DR: clear delay opto flag on new trial
  optoTriggeredStartCue = false;     // DR: clear startcue opto flag on new trial
  TrialNum = TrialNum + 1;
  TrialInBlock = TrialInBlock + 1;
  optoCueLick = false;  // reset potential lick during cue opto

  // find left and right reward size
  int LeftRewardIdx = random(numRewardLeftRange);
  RewardLeftCurrentSize = RewardLeftRange[LeftRewardIdx];
  int RightRewardIdx = random(numRewardRightRange);
  RewardRightCurrentSize = RewardRightRange[RightRewardIdx];

  // randomly select reward or not
  int randomRewardIdx = random(100);
  int RewardMaybe = RewardProbList[randomRewardIdx];
  if (RewardMaybe == 1) {
    giveReward = true;
    giveRewardIncorrect = false;
  } else {
    giveReward = false;
    giveRewardIncorrect = true;
  }

  // Check if we should switch to new block (or starting task) - both normal LLLR and 2ABT tasks
  if (task2ABT) {  // 2ABT: Markov reversal with refractory period
    int currentMin = (BlockType == LEFT_BLOCK) ? minBlockLeft : minBlockRight;
    int currentMax = (BlockType == LEFT_BLOCK) ? maxBlockLeft : maxBlockRight;
    if (currentMax == 0) {
      // Pure Markov mode (block length = 0): no refractory period, no forced reversal
      // 2% reversal probability applies from the very first trial
      if (random(100) < 2) {
        switchBlock();
      }
    } else if (TrialInBlock > currentMax) {
      // forced reversal: max refractory period exceeded
      switchBlock();
    } else if (TrialInBlock > currentMin) {
      // Markov zone: 2% reversal probability per trial
      if (random(100) < 2) {
        switchBlock();
      }
    }
    // else: still in refractory period, no reversal possible
  } else if (TrialNum == 1) {  // first block: always initialize
    switchBlock();
  } else if (BlockLength > 0) {
    // BlockSwitchAfterCorrectTrials == false (default):
    //   Switch when TrialInBlock > BlockLength (count all trials).
    // BlockSwitchAfterCorrectTrials == true:
    //   Switch when CorrectTrialCount >= BlockLength (count correct trials only).
    //   BlockLength == 0 means never switch blocks.
    bool useCorrectCount = BlockSwitchAfterCorrectTrials;
    bool shouldSwitch = useCorrectCount
                          ? (CorrectTrialCount >= BlockLength)
                          : (TrialInBlock > BlockLength);
    if (shouldSwitch) switchBlock();
  }

  // write data to matlab gui - current trial
  serialOutBuffer += "Reward prob trial";
  serialOutBuffer += '\t';  // add a tab to separate values
  serialOutBuffer += RewardMaybe;
  serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

  // write data to matlab gui - current trial
  if (BlockType == LEFT_BLOCK) {
    // write data to matlab gui - left reward size
    serialOutBuffer += "Reward size trial";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardLeftCurrentSize;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
  } else if (BlockType == RIGHT_BLOCK) {
    // write data to matlab gui - right reward size
    serialOutBuffer += "Reward size trial";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardRightCurrentSize;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
  }
}

// Actually switch blocks
void switchBlock() {

  //  if ((CorrectTrialCount >= BlockLength) || (TrialNum == 1)) {
  // Print stats for previous block
  if (TrialNum > 1) {
    printEndOfBlockStats();
    optoBlockFirstTrial = true;  // opto first trial in block
  }

  // update vars
  BlockNum = BlockNum + 1;
  TrialInBlock = 1;
  TrialsUntilFirstReward = 0;
  TrialsUntilFirstRewardHistory[BlockNum - 1] = 0;
  CorrectTrialCount = 0;
  incorrectCounter = 0;
  Num_ENLPenalty = 0;
  Num_Omissions = 0;
  freeRewardBlockCounter = 0;
  BlockRewardCounter = 0;
  BlockNoRewardCounter = 0;
  leftLickCounter = 0;
  rightLickCounter = 0;
  winStayCounter = 0;
  loseSwitchCounter = 0;
  winSwitchCounter = 0;
  loseStayCounter = 0;
  badTrialsCounter = 0;
  lastTrial = true;
  optoBlockDone = false;  // reset opto in block

  // switch BlockType
  if (BlockType == RIGHT_BLOCK) {
    BlockType = LEFT_BLOCK;
  } else if (BlockType == LEFT_BLOCK) {
    BlockType = RIGHT_BLOCK;
  }

  // Decide block length for left or right
  if (BlockType == LEFT_BLOCK) {
    int BlockLengthIdx = random(numBlockLengthsLeft);
    BlockLength = BlockLengthsLeft[BlockLengthIdx];
  } else if (BlockType == RIGHT_BLOCK) {
    int BlockLengthIdx = random(numBlockLengthsRight);
    BlockLength = BlockLengthsRight[BlockLengthIdx];
  }

  // write data to matlab gui - block switch
  if (!task2ABT) {  // not 2ABT as switching is a Markov process independent of BlockLength
    serialOutBuffer += "Out of";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += BlockLength;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
  }
}

// Generate sync pulses (100 ms pulses with random inter-pulse-interval)
void updateSyncPulses() {
  if (sessionStart == true) {
    if (millis() - TimerSync > SyncPulseInterval) {
      TimerSync = millis();
      if (SyncPulseStatus == 1) {
        // stop pulse
        digitalWrite(SyncPin, LOW);
        SyncPulseStatus = 0;
        // random low
        SyncPulseInterval = 100 + 10 * random(1, 50);  // random sync pulse interval between 110~590 ms in steps of 10 ms = 49 possibilities 
        // write data to matlab gui
        serialOutBuffer += "Sync_off";
        serialOutBuffer += '\t';  // add a tab to separate values
        serialOutBuffer += millis();
        serialOutBuffer += '\t';
        serialOutBuffer += SyncPulseInterval;
        serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

      } else {
        // start pulse
        digitalWrite(SyncPin, HIGH);
        SyncPulseStatus = 1;
        // keep high for fixed length
        SyncPulseInterval = 100;  // sync pulse HIGH for 100 ms
        // write data to matlab gui
        serialOutBuffer += "Sync_on";
        serialOutBuffer += '\t';  // add a tab to separate values
        serialOutBuffer += millis();
        serialOutBuffer += '\t';
        serialOutBuffer += SyncPulseInterval;
        serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
      }
    }
  } else if (sessionEnd == true) {
    // force camera ttl low at end of session
    digitalWrite(SyncPin, LOW);
    // write data to matlab gui
    serialOutBuffer += "Sync_off";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += millis();
    serialOutBuffer += '\t';
    serialOutBuffer += SyncPulseInterval;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

    // stop if statement
    sessionEnd = false;
  }
}

// Detect rising/falling edges on the external sync pin (INPUT, 0/3V TTL).
// Only emit edge events to MATLAB - no continuous sampling - to keep serial traffic low.
// Format mirrors the internal sync pulse stream (3 tab-separated fields) so the existing
// length==3 routing in MATLAB can handle it; first field disambiguates the source.
//
// Field semantics for the third (duration) column:
//   - ExtSync_on  -> inter-pulse interval (gap since previous falling edge).
//                    For the first pulse of the session: time since '8' was received.
//   - ExtSync_off -> pulse width (this rise -> this fall).
void updateExtSyncDetection() {
  if (sessionStart == true) {
    int currentRead = digitalRead(ExtSyncPin);

    if (currentRead == HIGH && ExtSyncState == 0) {
      // rising edge - external pulse started
      ExtSyncState = 1;
      ExtSyncRiseTime = millis();
      // compute inter-pulse interval (or time since session start if this is the first pulse)
      unsigned long ExtSyncITI;
      if (firstExtSync) {
        ExtSyncITI = ExtSyncRiseTime - SessionStartTime;
        firstExtSync = false;
      } else {
        ExtSyncITI = ExtSyncRiseTime - ExtSyncLastFallTime;
      }
      // write data to matlab gui
      serialOutBuffer += "ExtSync_on";
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncRiseTime;
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncITI;  // inter-pulse interval (ms)
      serialOutBuffer += '\n';
      // report so ext sync pulse pin
      digitalWrite(ExtSyncReportPin, HIGH);
      
    } else if (currentRead == LOW && ExtSyncState == 1) {
      // falling edge - external pulse ended
      ExtSyncState = 0;
      unsigned long ExtSyncFallTime = millis();
      unsigned long ExtSyncDuration = ExtSyncFallTime - ExtSyncRiseTime;
      // write data to matlab gui
      serialOutBuffer += "ExtSync_off";
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncFallTime;
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncDuration;  // pulse width (ms)
      serialOutBuffer += '\n';
      // remember this fall time for the next rising edge's ITI calculation
      ExtSyncLastFallTime = ExtSyncFallTime;
      // report so ext sync pulse pin
      digitalWrite(ExtSyncReportPin, LOW);

    }
  } else {
    // outside session: keep state in sync with the line so we don't fire a stale edge
    // the moment a new session starts
    ExtSyncState = digitalRead(ExtSyncPin);
    
    // turn ext sync pulse pin off in case we stop task during pulse
    digitalWrite(ExtSyncReportPin, LOW);
  }
}

// trigger camera
void updateCameraTrigger() {
  // 2ms pulses at 50 Hz
  unsigned long Now = millis();
  if (CurrentState != IDLE) {
    if (((Now - CameraTrigStartTime) % 20) == 0 || ((Now - CameraTrigStartTime) % 20) == 1) {
      digitalWrite(CameraTriggerPin, HIGH);
    } else {
      digitalWrite(CameraTriggerPin, LOW);
    }
  }
}

// check buttons
void checkButtons() {
  // TODO
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ----------------------- Functions that are related to printing ---------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// update state name
void outputTrialState() {
  if (CurrentState == PRE_REWARD) {
    // no need to indicate pre-reward state
    return;
  }
  const char* stateName = "";
  unsigned long duration = 0;
  if (CurrentState == IDLE) {
    stateName = "IDLE";
  } else if (CurrentState == NEW_TRIAL) {
    stateName = "NEW_TRIAL";
  } else if (CurrentState == ITI) {
    if (BlockType == RIGHT_BLOCK) {
      stateName = "ITI_RIGHT";
    } else {
      stateName = "ITI_LEFT";
    }
    // stateName = "ITI";
    duration = ITIDuration;
  } else if (CurrentState == ENL_PENALTY) {
    stateName = "ENL_PENALTY";
    duration = ENLPenaltyDuration;
  } else if (CurrentState == CUE_PENALTY) {
    stateName = "CUE_PENALTY";
    duration = CuePenaltyDuration;
  } else if (CurrentState == WAIT_LICK) {
    stateName = "WAIT_LICK";
    duration = 0;  // open-ended: waits until the animal licks
  } else if (CurrentState == DELAY) {
    stateName = "DELAY";
    duration = DelayDuration;
  } else if (CurrentState == DELAY_PENALTY) {
    stateName = "DELAY_PENALTY";
    duration = DelayPenaltyDuration;
  } else if (CurrentState == START_CUE) {
    stateName = "START_CUE";
    duration = StartCueDuration;
  } else if (CurrentState == STARTCUE_PENALTY) {
    stateName = "STARTCUE_PENALTY";
    duration = StartCuePenaltyDuration;
  } else if (CurrentState == CUE) {
    if (BlockType == RIGHT_BLOCK) {
      stateName = "CUE_RIGHT";
    } else {
      stateName = "CUE_LEFT";
    }
    duration = ToneDuration;
  } else if (CurrentState == CUE_PENALTY) {
    stateName = "CUE_PENALTY";
    duration = CuePenaltyDuration;
  } else if (CurrentState == SELECTION) {
    stateName = "SELECTION";
    duration = ReactionTime;
  } else if (CurrentState == REWARD) {
    stateName = "REWARD";
    if (BlockType == RIGHT_BLOCK) {
      duration = RewardRightCurrentSize;
    } else {
      duration = RewardLeftCurrentSize;
    }
  } else if (CurrentState == NO_REWARD) {
    stateName = "NO_REWARD";
    duration = 0;
  } else if (CurrentState == FREE_REWARD) {
    stateName = "FREE_REWARD";
  } else if (CurrentState == REWARD_DELAY) {
    stateName = "REWARD_DELAY";
    duration = RewardDelay;
  } else if (CurrentState == CONSUMPTION) {
    stateName = "CONSUMPTION";
    duration = ConsumptionDuration;
  }
  outputEvent(stateName, millis(), duration);
}

// Print trial Statements at setup, send to matlab GUI
void printDataHeader() {
  Serial.println("TrialNum\tBlockNum\tTrialInBlock\tCorrectTrialCount\tEvent\tEventTime\tLickDuration");
}

// output event values - 7 long trialnum/blocknum/trialinblock/correcttrialcount/eventnm/starttime/duration
void outputEvent(const char* eventName, unsigned long startTime, unsigned long duration) {
  pausePrint = true;
  // unsigned long pTime = micros();
  serialOutBuffer += TrialNum;
  serialOutBuffer += '\t';  // add a tab to separate values
  serialOutBuffer += BlockNum;
  serialOutBuffer += '\t';
  serialOutBuffer += TrialInBlock;
  serialOutBuffer += '\t';
  serialOutBuffer += CorrectTrialCount;
  serialOutBuffer += '\t';
  serialOutBuffer += eventName;
  serialOutBuffer += '\t';
  serialOutBuffer += startTime;
  serialOutBuffer += '\t';
  serialOutBuffer += duration;
  serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
  // Serial.print(tmpserialOutBuffer.c_str());
  // Serial.println(micros() - pTime);

  // TIMING tested by OM 2022-10-04
  // - 250-450us  building up the string
  // - ~250us     printing the string to Serial
}

// Helper - emit a Block Stats block (used by both end-of-block and end-of-session)
void emitBlockStats() {
  serialOutBuffer += "\nBlock Stats\t";  // start on new line to make sure user can read
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\n";
  serialOutBuffer += "Block Num\t";
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\t";
  serialOutBuffer += "Block\t";
  if (BlockType == RIGHT_BLOCK) {
    serialOutBuffer += "RIGHT\t";
  } else {
    serialOutBuffer += "LEFT\t";
  }
  serialOutBuffer += "Length\t";
  serialOutBuffer += BlockLength;
  serialOutBuffer += '\t';
  serialOutBuffer += "Trials\t";
  serialOutBuffer += (TrialInBlock - 1);
  serialOutBuffer += '\t';  // NOTE I did trials - 1. It gets incremented in new trial, which is too early
  serialOutBuffer += "Correct\t";
  serialOutBuffer += CorrectTrialCount;
  serialOutBuffer += '\t';
  serialOutBuffer += "Incorrect\t";
  serialOutBuffer += incorrectCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "Rewards\t";
  serialOutBuffer += BlockRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "No Rewards\t";
  serialOutBuffer += BlockNoRewardCounter;
  serialOutBuffer += '\t';
  serialOutBuffer += "ENL Penalties\t";
  serialOutBuffer += Num_ENLPenalty;
  serialOutBuffer += "\t";
  serialOutBuffer += "Omissions\t";
  serialOutBuffer += Num_Omissions;
  serialOutBuffer += "\t";
  serialOutBuffer += "Free Rewards\t";
  serialOutBuffer += freeRewardBlockCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Left licks\t";
  serialOutBuffer += leftLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Right licks\t";
  serialOutBuffer += rightLickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Stays\t";
  serialOutBuffer += winStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Switches\t";
  serialOutBuffer += loseSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Win Switches\t";
  serialOutBuffer += winSwitchCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Lose Stays\t";
  serialOutBuffer += loseStayCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Bad Trials Post Reward\t";
  serialOutBuffer += badTrialsCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Trials To 1st Reward\t";
  serialOutBuffer += TrialsUntilFirstReward;
  serialOutBuffer += "\n\n";  // 2 new lines -> 1st line for updateSerialOutput to know when to cut, 2nd for user reading serial monitor
  // serialOutBuffer += "Tot. rewards: "; serialOutBuffer += Num_Reward; serialOutBuffer += "\n\n\n";
  // Serial.print(statsString.c_str());
  // TIMING (tested by OM 2022-10-04)
  // ~3 ms
}

// print end of block data - starts with Block Stats, plots every block end
void printEndOfBlockStats() {
  pausePrint = true;
  emitBlockStats();
}

// print end of session data - again starts with Block Stats, only plotted at the end
void printEndOfSessionStats() {
  pausePrint = true;
  emitBlockStats();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------- Functions that are related to optogenetic stimulation ----------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// initiate a new opto stimulation
void startOptoStim() {
  if (OptoEnabled) {
    // keep track of start of opto
    optoStartTime = millis();
    duringOptoStim = true;
    duringOptoStimDelay = true;
    outputEvent("Opto_START", millis(), OptoDuration);
  }
}

// This should be called on a regular basis by loop().
// Turns opto on/off as specified by the delay and duration paramters.
void updateOptoStim() {

  // stimulate pulsed or continuous
  if (duringOptoStim) {

    // keep track of time in opto stim
    timeInStim = millis() - optoStartTime;

    if (duringOptoStimDelay && (timeInStim > OptoDelay)) {
      // if we've past delay period, turn on opto
      digitalWrite(OptoPin, HIGH);
      digitalWrite(OptoTTLPin, HIGH);
      duringOptoStimDelay = false;
      optoStartPulse = millis();
      optoPulseActive = true;
      outputEvent("Opto_ON", millis(), OptoDuration);

    } else if ((timeInStim > (OptoDelay + OptoDuration)) || optoCueLick == true) {
      // if we've reached the end of the opto stim duration, turn off opto
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
      duringOptoStim = false;
      optoPulseActive = false;
      optoIPIActive = false;
      optoTaperActive = false;
      outputEvent("Opto_OFF", millis(), OptoDuration);

    } else if (optoContinuous == false && optoPulseActive == true) {
      // keep track of IPI duration when frequency modulated
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > OptoPulseDuration) {
        // turn opto off after pulse duration has passed
        digitalWrite(OptoPin, LOW);
        digitalWrite(OptoTTLPin, LOW);
        optoPulseActive = false;
        optoIPIActive = true;
        optoStartPulse = millis();
      }

    } else if (optoContinuous == false && optoIPIActive == true) {
      // calculate inter-pulse-interval
      optoIPI = (1000 / OptoFrequency) - OptoPulseDuration;  // keep track of IPI
      // keep track of pulse high when frequency modulated
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > optoIPI) {
        // turn opto ON after IPI has passed
        digitalWrite(OptoPin, HIGH);
        digitalWrite(OptoTTLPin, HIGH);
        optoIPIActive = false;
        optoPulseActive = true;
        optoStartPulse = millis();
      }
    }
  }

  // apply AOM or driver modulation
  if (manualOptoTestActive) {
    // manual test is running — the "bt" handler already wrote the correct value;
    // skip the main-loop management so it cannot clobber OptoModPin
  } else if (OptoEnabled) {  // this is an opto session

    if (OptoModContinuous) {  // continuous AOM or led during opto
      // AOM mode: always drive the modulation pin at OptoMod level
      // (existing behavior — keep pre-armed between trials, taper during stim)
      if (!duringOptoStim) {
        analogWrite(OptoModPin, OptoMod * 2.56);
      } else if (duringOptoStim) {
        // if we are stimulating, start keeping an eye on tapering off
        if (duringOptoStim && !optoTaperActive && ((long)timeInStim > (long)(OptoDelay + OptoDuration - OptoTaperOff))) {
          // state 2) first time we start tapering off, we only get here once per opto stim
          optoTaperStartTime = millis();
          optoTaperActive = true;

        } else if (optoTaperActive) {
          // state 3) taper off stimulation if needed
          // how long are we in taper off mode?
          timeInTaper = millis() - optoTaperStartTime;
          // calculate what the current opto modulation should be
          tmpOptoMod = round((1 - ((float)timeInTaper / OptoTaperOff)) * OptoMod);
          // write that value
          analogWrite(OptoModPin, tmpOptoMod * 2.56);  // user sets percentage - convert to 0-256 to drive 0-5 V -  round(256 * (OptoMod / 100))
        }
      }

    } else {
      // LED driver mode: only drive the pin DURING opto stim
      if (duringOptoStim) {
        if (optoTaperActive) {
          // taper logic unchanged...
          analogWrite(OptoModPin, tmpOptoMod * 2.56);
        } else {
          analogWrite(OptoModPin, OptoMod * 2.56);
        }
      } else {
        // NOT during stim → keep pin at 0
        analogWrite(OptoModPin, 0);
      }
    }

  } else if (!OptoEnabled) {
    // turn off opto modulation
    analogWrite(OptoModPin, 0);
  }
}

// initiate random opto distraction light
void randomLightFunction() {
  // check if we want opto random distraction light and the task is running
  if ((ApplyOptoRandomLight) && (OptoEnabled) && (sessionStart)) {
    // apply random distraction light
    if (millis() - RandomLightTimer >= RandomLightInterval) {
      if (RandomLightON == 1) {
        // stop light - random duration off
        RandomLightTimer = millis();
        digitalWrite(RandomLightPin, LOW);
        RandomLightON = 0;
        RandomLightInterval = 100 + random(1, 900);  // random opto interval between 100-1000 ms
      } else {
        // start light - fixed duration on
        RandomLightTimer = millis();
        digitalWrite(RandomLightPin, HIGH);
        RandomLightON = 1;
        RandomLightInterval = 500;  // random opto light on for 500 ms
      }
    }
  } else if (sessionEnd == false) {
    digitalWrite(RandomLightPin, LOW);  // make sure to turn off opto random light
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------- Functions that are related to serial communication ----------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Higher MAX_CHARS_PER_LOOP + faster baud (115200) drains the serial buffer much faster,
// which is what removes the visible lag in the MATLAB GUI ("Event:" label / new trial dot).
// Keep this <= ~256 so a single loop still completes well under 1 ms in the worst case.
const unsigned int MAX_CHARS_PER_LOOP = 128;  // used to be 24 characters (emprically found to be fast and we don't write many characters per cycle anyway)
char tempBuffer[MAX_CHARS_PER_LOOP + 10];    // make a bigger tmpBuffer
void updateSerialOutput() {
  // // FOR DEBUGGING
  // Serial.print(serialOutBuffer.c_str());
  // serialOutBuffer = "";
  // return;

  // Spread Serial output over multiple loop iterations by printing
  // no more than MAX_CHARS_PER_LOOP characters at a time.
  if (pausePrint) {
    // don't print anything on iterations where pausePrint flag is set
    // (should be set when long string is written to serialOutBuffer)
    pausePrint = false;
    return;
  }

  if (serialOutBuffer.length() > 0) {
    unsigned int numCharToPrint = min(MAX_CHARS_PER_LOOP, serialOutBuffer.length());
    serialOutBuffer.toCharArray(tempBuffer, numCharToPrint + 1);  // add 1 more to know when to stop
    Serial.print(tempBuffer);                                     // just print!
    // Serial.print("["); // for debugging
    // Serial.print(numCharToPrint); // for debugging
    // Serial.print("]"); // for debugging
    serialOutBuffer.remove(0, numCharToPrint);  // remove the chars we just printed
  }
}

// manually trigger events (can go through serial command & matlab)
void checkSerialInput() {

  if (Serial.available() > 0) {
    // read the incoming byte
    SerialInput = Serial.read();

    // Handle inputs 1, 2, 8, and 9
    // give a manual left reward = 1
    if (SerialInput == '1' && LeftRewardButtonStatus == 0) {
      CueStartTime = millis();
      LeftRewardButtonStatus = 1;
      digitalWrite(LeftRewardPin, HIGH);
      digitalWrite(LeftRewardIndicatorPin, HIGH);
      LeftRewardTimer = millis();

      // write serial for matlab gui
      outputEvent("MANUAL_LEFT_REWARD", CueStartTime, RewardLeftCurrentSize);
    }

    // give a manual right reward = 2
    else if (SerialInput == '2' && RightRewardButtonStatus == 0) {
      CueStartTime = millis();
      RightRewardButtonStatus = 1;
      digitalWrite(RightRewardPin, HIGH);
      digitalWrite(RightRewardIndicatorPin, HIGH);
      RightRewardTimer = millis();

      // write seral for matlab gui
      outputEvent("MANUAL_RIGHT_REWARD", CueStartTime, RewardRightCurrentSize);
    }

    // task/session start = 8
    else if ((SerialInput == '8') || (buttonsAreWiredUp && (digitalRead(StartStopButtonPin) == HIGH))) {
      sessionStart = true;   // variable to keep track of task running
      TimerSync = millis();  // start sync pulse
      digitalWrite(CameraTriggerPin, LOW);
      CameraTrigStartTime = millis();
      digitalWrite(StartStopIndicatorPin, HIGH);

      // session-start timing: used by ExtSync (first-pulse ITI) and START_DELAY state
      SessionStartTime = millis();
      StartDelayTimer = millis();
      ExtSyncLastFallTime = 0;
      ExtSyncRiseTime = SessionStartTime;       // seed so a stray fall before any rise reports a sane bounded duration
      firstExtSync = true;
      ExtSyncState = digitalRead(ExtSyncPin);   // re-baseline so a line already-high doesn't fire a fake edge

      // write data to matlab gui - task start
      serialOutBuffer += "TASK STARTED AT";
      serialOutBuffer += '\t';  // add a tab to separate values
      serialOutBuffer += millis();
      serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

      // identify first block direction
      if (spoutStart == 0) {
        randomBlock();
      } else if (spoutStart == 1) {  // start on left block
        BlockType = RIGHT_BLOCK;
      } else if (spoutStart == 2) {  // start on right block
        BlockType = LEFT_BLOCK;
      }

      // fill variables
      fillBlockLengths();
      fillITIDuration();
      fillRewardLeftRange();
      fillRewardRightRange();
      fillRewardProbList();
      NextState = START_DELAY;  // wait TaskStartDelay seconds before the first NEW_TRIAL
    }

    // task stop = 9
    else if (SerialInput == '9' && sessionStart == true) {
      sessionStart = false;                       // variable to keep track of task running
      sessionEnd = true;                          // make syncPulse low
      digitalWrite(StartStopIndicatorPin, HIGH);  // indicate end of session
      NextState = IDLE;
      // write data to matlab gui - task stop
      serialOutBuffer += "TASK ENDED AT";
      serialOutBuffer += '\t';  // add a tab to separate values
      serialOutBuffer += millis();
      serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut

      // print block details if animal did more than 1 block
      if (TrialInBlock > 1) {
        printEndOfSessionStats();
      }
    }

    // This code adapted from Ofer's readFromUSB function
    // Will accept messages coming as inputs that can be parsed later
    else {
      static String usbMessage = "";  // initialize usbMessage to empty string

      // Start building the message from the current input
      usbMessage += SerialInput;

      // Continue reading characters until end of message
      while (Serial.available() > 0) {
        char inByte = Serial.read();
        if ((inByte == '\n') || (inByte == ';')) {
          // Message is complete, interpret it
          interpretUSBMessage(usbMessage);
          usbMessage = "";  // clear message buffer
        } else {
          // Append character to message buffer
          usbMessage += inByte;
        }
      }
    }

    // if (SerialInput == 'xxx') {
    //   Serial.println("Entered 3: ShutterBlue on");
    //   digitalWrite(OptoPin, HIGH);
    //   //delay(2000);
    //   //digitalWrite(ShutterBlue, LOW);
    // }

    //  if (SerialInput == 'xxx') {
    //   Serial.println("Entered 4: ShutterBlue off");
    //   digitalWrite(OptoPin, LOW);
    //   //delay(2000);
    //   //digitalWrite(ShutterBlue, LOW);
    // }

    // if (SerialInput == '5') {
    //   Serial.println("Entered 5: OptoStim Enabled");
    //   OptoEnabled = true;
    // }

    // if (SerialInput == '6') {
    //   Serial.println("Entered 6: OptoStim Disabled");
    //   OptoEnabled = false;
    // }

    //if (SerialInput == '3') {
    //  Serial.println("");
    //  Serial.println("** Loop Timing **");
    //  Serial.print(" Mean Loop: ");
    //  Serial.print(cumulativeLoopTime_us / loopCounter);
    //  Serial.println(" us");
    //  Serial.print(" Max Loop: ");
    //  Serial.print(maxLoopTime_us);
    //  Serial.println(" us");
    //  Serial.println("");
    //  maxLoopTime_us = 0;
    //  cumulativeLoopTime_us = 0;
    //  loopCounter = 0;
    //  loopStartTime_us = micros();  // cancel out this loop
    //}

    //if (SerialInput == '4') {
    //  outputTrialState();
    //}
  }

  // stop a manual left reward
  if ((millis() - LeftRewardTimer) > RewardLeftCurrentSize && LeftRewardButtonStatus == 1) {
    digitalWrite(LeftRewardPin, LOW);
    digitalWrite(LeftRewardIndicatorPin, LOW);
    LeftRewardButtonStatus = 0;
  }

  // stop a manual right reward
  if ((millis() - RightRewardTimer) > RewardRightCurrentSize && RightRewardButtonStatus == 1) {
    digitalWrite(RightRewardPin, LOW);
    digitalWrite(RightRewardIndicatorPin, LOW);
    RightRewardButtonStatus = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------- USB/GUI command parser --------------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// allow Matlab to define task variables through our GUI
void interpretUSBMessage(String message) {

  // 1) Parse message into "<command> and <arg1>"
  // Where <command> is a double character, and <arg1> is a positive integer

  message.trim();  // remove leading and trailing white space
  int len = message.length();
  if (len == 0) {
    Serial.println("#");  // "#" means error, message does not exist
    return;
  }
  String command = message.substring(0, 2);  // the command is the first 2 chars of a message
  String parameters = message.substring(2);
  parameters.trim();

  // Handshake - matlab GUI knows we're connected
  if (command == "^^") {
    Serial.println("^^");
    return;
  }

  String intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) {
    intString += parameters[0];
    parameters.remove(0, 1);
  }
  long arg1 = intString.toInt();

  // 2) Act on the command
  // free letters:
  if (command == "aa") {  // Set min number of trials per block for all blocks
    minBlockLeft = arg1;
    minBlockRight = arg1;
    minBlock = arg1;
    fillBlockLengths();
    // write data to matlab gui - update min block length left/right
    serialOutBuffer += "Min Block Left";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += minBlockLeft;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Min Block Right";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += minBlockRight;
    serialOutBuffer += '\n';     // 1 new line -> for updateSerialOutput to know when to cut
  } else if (command == "ab") {  // Set max number of trials per block for all blocks
    maxBlockLeft = arg1;
    maxBlockRight = arg1;
    maxBlock = arg1;
    fillBlockLengths();
    // write data to matlab gui - update max block length left/right
    serialOutBuffer += "Max Block Left";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += maxBlockLeft;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Max Block Right";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += maxBlockRight;
    serialOutBuffer += '\n';     // 1 new line -> for updateSerialOutput to know when to cut
  } else if (command == "am") {  // set min number of trials per block for LEFT
    minBlockLeft = arg1;
    fillBlockLengths();
  } else if (command == "an") {  // set max number of trials per block for LEFT
    maxBlockLeft = arg1;
    fillBlockLengths();
  } else if (command == "ap") {  // set min number of trials per block for RIGHT
    minBlockRight = arg1;
    fillBlockLengths();
  } else if (command == "aq") {  // set max number of trials per block for RIGHT
    maxBlockRight = arg1;
    fillBlockLengths();
  } else if (command == "ac") {  // set fails until reward
    fails2reward = arg1;
  } else if (command == "ad") {  // set selection time
    ReactionTime = arg1;
  } else if (command == "ae") {  // manual auditory cues
    if (arg1 == 0) {             // Left cue
      CueStartTime = millis();
      if (cuesInstructed) {
        // two-speaker mode: left cue on left speaker only
        analogWriteFrequency(SpeakerPinL, LeftCueFreq);
        analogWrite(SpeakerPinL, 128);
        analogWrite(SpeakerPinR, 0);
      } else {
        // uninstructed cue mode: play on both pins for manual testing
        analogWriteFrequency(SpeakerPinL, LeftCueFreq);
        analogWrite(SpeakerPinL, 128);
        analogWriteFrequency(SpeakerPinR, LeftCueFreq);
        analogWrite(SpeakerPinR, 128);
      }
      digitalWrite(SpeakerTTLPin, HIGH);
      digitalWrite(LeftCueIndicatorPin, HIGH);
      outputEvent("MANUAL_LEFT_CUE", CueStartTime, ToneDuration);
    } else if (arg1 == 1) {  // Right cue
      CueStartTime = millis();
      if (cuesInstructed) {
        // two-speaker mode: right cue on right speaker only
        analogWrite(SpeakerPinL, 0);
        analogWriteFrequency(SpeakerPinR, RightCueFreq);
        analogWrite(SpeakerPinR, 128);
      } else {
        // uninstructed cue mode: play on both pins for manual testing
        analogWriteFrequency(SpeakerPinL, RightCueFreq);
        analogWrite(SpeakerPinL, 128);
        analogWriteFrequency(SpeakerPinR, RightCueFreq);
        analogWrite(SpeakerPinR, 128);
      }
      digitalWrite(SpeakerTTLPin, HIGH);
      digitalWrite(RightCueIndicatorPin, HIGH);
      outputEvent("MANUAL_RIGHT_CUE", CueStartTime, ToneDuration);
    }
  } else if (command == "ag") {  // Change left calibration size
    RewardLeftMin = arg1;
    RewardLeftMax = arg1;
    RewardLeftStepSize = 1;
    RewardLeftCurrentSize = arg1;
    fillRewardLeftRange();
    // write data to matlab gui - update min/max left reward
    serialOutBuffer += "Min Left";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardLeftMin;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Max Left";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardLeftMax;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Left Step";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardLeftStepSize;
    serialOutBuffer += '\n';     // 1 new line -> for updateSerialOutput to know when to cut
  } else if (command == "ah") {  // Change right calibration size
    RewardRightMin = arg1;
    RewardRightMax = arg1;
    RewardRightStepSize = 1;
    RewardRightCurrentSize = arg1;
    fillRewardRightRange();
    // write data to matlab gui - update min/max right reward
    serialOutBuffer += "Min Right";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardRightMin;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Max Right";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardRightMax;
    serialOutBuffer += '\n';  // 1 new line -> for updateSerialOutput to know when to cut
    serialOutBuffer += "Right Step";
    serialOutBuffer += '\t';  // add a tab to separate values
    serialOutBuffer += RewardRightStepSize;
    serialOutBuffer += '\n';     // 1 new line -> for updateSerialOutput to know when to cut
  } else if (command == "ai") {  // Set min ITI
    minITI = arg1;
    fillITIDuration();
  } else if (command == "aj") {  // Set max ITI
    maxITI = arg1;
    fillITIDuration();
  } else if (command == "ak") {  // Set ITI step
    ITIstep = arg1;
    fillITIDuration();
  } else if (command == "al") {  // Turn opto ON/OFF
    if (arg1 == 0) {             // OFF
      OptoEnabled = false;
    } else if (arg1 == 1) {  // ON
      OptoEnabled = true;
    }
  } else if (command == "ao") {  // spout start
    if (arg1 == 0) {             // random
      spoutStart = 0;
    } else if (arg1 == 1) {  // left
      spoutStart = 1;
    } else if (arg1 == 2) {  // right
      spoutStart = 2;
    }
  } else if (command == "ar") {  // min left reward size
    RewardLeftMin = arg1;
    fillRewardLeftRange();
  } else if (command == "as") {  // max left reward size
    RewardLeftMax = arg1;
    fillRewardLeftRange();
  } else if (command == "at") {  // left reward step
    RewardLeftStepSize = arg1;
    fillRewardLeftRange();
  } else if (command == "au") {  // min right reward size
    RewardRightMin = arg1;
    fillRewardRightRange();
  } else if (command == "av") {  // max right reward size
    RewardRightMax = arg1;
    fillRewardRightRange();
  } else if (command == "aw") {  // right reward step
    RewardRightStepSize = arg1;
    fillRewardRightRange();
  } else if (command == "ax") {  // reward probability
    RewardProb = arg1;
    fillRewardProbList();
  } else if (command == "ba") {  // opto continuous?
    if (arg1 == 1) {
      optoContinuous = true;
    } else {
      optoContinuous = false;
    }
  } else if (command == "bb") {  // opto probability
    OptoStimProb = arg1;
  } else if (command == "bc") {  // correct trials before opto
    numCorrectTrialsBeforeOpto = arg1;
  } else if (command == "bd") {  // opto delay
    OptoDelay = arg1;
  } else if (command == "be") {  // opto duration
    OptoDuration = arg1;
  } else if (command == "bf") {  // pulse duration
    OptoPulseDuration = arg1;
  } else if (command == "bg") {  // opto frequency
    OptoFrequency = arg1;
  } else if (command == "bh") {  // opto during block switch
    if (arg1 == 1) {
      OptoActiveDuringBlockSwitch = true;
    } else {
      OptoActiveDuringBlockSwitch = false;
    }
  } else if (command == "bi") {  // opto during ITI
    if (arg1 == 1) {
      OptoActiveDuringITI = true;
    } else {
      OptoActiveDuringITI = false;
    }
  } else if (command == "bj") {  // opto during cue
    if (arg1 == 1) {
      OptoActiveDuringCue = true;
    } else {
      OptoActiveDuringCue = false;
    }
  } else if (command == "bk") {  // opto during consumption
    if (arg1 == 1) {
      OptoActiveDuringConsumption = true;
    } else {
      OptoActiveDuringConsumption = false;
    }
  } else if (command == "af") {  // change opto AOM modulation
    OptoMod = arg1;
    if (manualOptoTestActive) {  // immediately update pin if manual test is running
      analogWrite(OptoModPin, OptoMod * 2.56);
    }
  } else if (command == "by") {  // change opto taper off
    OptoTaperOff = arg1;
  } else if (command == "bl") {  // allow multi-stim in ITI
    if (arg1 == 1) {
      AllowMultiOptoStimITI = true;
    } else {
      AllowMultiOptoStimITI = false;
    }
  } else if (command == "bx") {  // allow multi-stim in cue
    if (arg1 == 1) {
      AllowMultiOptoStimCue = true;
    } else {
      AllowMultiOptoStimCue = false;
    }
  } else if (command == "bm") {  // allow multi-stim in block
    if (arg1 == 1) {
      AllowMultiOptoStimBlock = true;
    } else {
      AllowMultiOptoStimBlock = false;
    }
  } else if (command == "bn") {  // set cue penalty duration
    CuePenaltyDuration = arg1;
  } else if (command == "bo") {  // set left cue frequency
    LeftCueFreq = arg1;
  } else if (command == "bp") {  // set right cue frequency
    RightCueFreq = arg1;
  } else if (command == "bq") {  // set outcome duration for Reward
    CorrectDuration = arg1;
  } else if (command == "bz") {  // set outcome duration for NoReward
    IncorrectDuration = arg1;
  } else if (command == "br") {  // set cue duration
    ToneDuration = arg1;
  } else if (command == "bs") {  // set opto random light
    if (arg1 == 1) {
      ApplyOptoRandomLight = true;
    } else {
      ApplyOptoRandomLight = false;
    }
  } else if (command == "bt") {  // turn opto ON and OFF for testing
    if (arg1 == 1) {
      manualOptoTestActive = true;
      if (!OptoModContinuous) {  // led modulation during opto
        analogWrite(OptoModPin, OptoMod * 2.56);
      }
      digitalWrite(OptoPin, HIGH);
      digitalWrite(OptoTTLPin, HIGH);
    } else {
      manualOptoTestActive = false;
      if (!OptoModContinuous) {  // turn off led modulation
        analogWrite(OptoModPin, 0);
      }
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
    }
  } else if (command == "bu") {  // task: 2ABT on (1) or off (0)
    if (arg1 == 1) { task2ABT = true;  taskLLLR = false; taskPavlovian = false; taskDelayedResponse = false; }
    else if (taskLLLR || taskPavlovian || taskDelayedResponse) { task2ABT = false; }
  } else if (command == "cj") {  // task: LLLR on (1) or off (0)
    if (arg1 == 1) { taskLLLR = true;  task2ABT = false; taskPavlovian = false; taskDelayedResponse = false; }
    else if (task2ABT || taskPavlovian || taskDelayedResponse) { taskLLLR = false; }
  } else if (command == "bv") {  // max number of consecutive omissions before session end - Matlab GUI determines
    maxOmissions = arg1;
  } else if (command == "bw") {  // max duration (min) before session end - Matlab GUI determines
    maxTimeout = arg1;
  } else if (command == "cc") {  // max total trials before session end - Matlab GUI determines
    maxTrials = arg1;
  } else if (command == "cd") {  // max total rewards before session end - Matlab GUI determines
    maxRewards = arg1;
  } else if (command == "cg") {  // ENL enabled (1) or disabled (plain ITI) (0)
    if (arg1 == 1) {
      ENLEnabled = true;
    } else {
      ENLEnabled = false;
    }
  } else if (command == "ce") {  // task: Pavlovian on (1) or off (0)
    if (arg1 == 1) { taskPavlovian = true;  taskLLLR = false; task2ABT = false; taskDelayedResponse = false; }
    else if (taskLLLR || task2ABT || taskDelayedResponse) { taskPavlovian = false; }
  } else if (command == "cf") {  // Pavlovian pause duration (ms)
    RewardDelay = arg1;
  } else if (command == "cm") {  // ENL penalty duration (ms)
    ENLPenaltyDuration = arg1;
  } else if (command == "ca") {  // set one-speaker (1) or two-speaker (0) mode
    if (arg1 == 1) {
      cuesInstructed = true;
    } else {
      cuesInstructed = false;
    }
  } else if (command == "cb") {  // have opto modulation continuously on or only during opto
    if (arg1 == 1) {
      OptoModContinuous = true;
    } else {
      OptoModContinuous = false;
    }
  } else if (command == "ch") {  // opto during error (incorrect lick) trials
    if (arg1 == 1) {
      OptoActiveDuringError = true;
    } else {
      OptoActiveDuringError = false;
    }
  } else if (command == "ci") {  // opto during any outcome (reward, error, or omission)
    if (arg1 == 1) {
      OptoActiveDuringAnyOutcome = true;
    } else {
      OptoActiveDuringAnyOutcome = false;
    }
  } else if (command == "ck") {  // error light duration (ms)
    ErrorLightDuration = arg1;
  } else if (command == "cl") {  // task start delay (s) - wait between '8' and first NEW_TRIAL
    TaskStartDelay = arg1;
  } else if (command == "cs") { 
    if (arg1 == 1) {
      BlockSwitchAfterCorrectTrials = true;
    } else {
      BlockSwitchAfterCorrectTrials = false;
    }
  } else if (command == "cn") {  // DR: manual start-cue test (plays start cue tone for StartCueDuration on StartCueSpeakerPin)
    ManualStartCueTime = millis();
    manualStartCueActive = true;
    analogWriteFrequency(StartCueSpeakerPin, StartCueFreq);
    analogWrite(StartCueSpeakerPin, 128);
    digitalWrite(StartCueIndicatorPin, HIGH);
    outputEvent("MANUAL_START_CUE", ManualStartCueTime, StartCueDuration);
  } else if (command == "co") {  // task: Delayed-Response on (1) or off (0)
    if (arg1 == 1) { taskDelayedResponse = true; taskLLLR = false; task2ABT = false; taskPavlovian = false; }
    else if (taskLLLR || task2ABT || taskPavlovian) { taskDelayedResponse = false; }
  } else if (command == "cp") {  // DR: DELAY duration (ms)
    DelayDuration = arg1;
  } else if (command == "cq") {  // DR: START_CUE duration (ms)
    StartCueDuration = arg1;
  } else if (command == "cr") {  // DR: start cue frequency (Hz)
    StartCueFreq = arg1;
  } else if (command == "ct") {  // DR: opto during DELAY
    if (arg1 == 1) {
      OptoActiveDuringDelay = true;
    } else {
      OptoActiveDuringDelay = false;
    }
  } else if (command == "cu") {  // DR: opto during START_CUE
    if (arg1 == 1) {
      OptoActiveDuringStartCue = true;
    } else {
      OptoActiveDuringStartCue = false;
    }
  } else if (command == "cv") {  // DR: delay penalty duration (ms; 0 disables)
    DelayPenaltyDuration = arg1;
  } else if (command == "cw") {  // DR: start cue penalty duration (ms; 0 disables)
    StartCuePenaltyDuration = arg1;
  } else if (command == "cx") {  // DR: allow multi-stim in Delay (re-arm on delay penalty)
    if (arg1 == 1) {
      AllowMultiOptoStimDelay = true;
    } else {
      AllowMultiOptoStimDelay = false;
    }
  } else if (command == "cy") {  // DR: allow multi-stim in StartCue (re-arm on startcue penalty)
    if (arg1 == 1) {
      AllowMultiOptoStimStartCue = true;
    } else {
      AllowMultiOptoStimStartCue = false;
    }
  } else if (command == "cz") {  // Pavlovian lick-to-start: on (1) or off (0)
    if (arg1 == 1) {
      PavlovianLickToStart = true;
    } else {
      PavlovianLickToStart = false;
    }
  } else if (command == "ay") {  // to save memory, do not call until needed
    saveParams();                // will update EEPROM with current values for variables
  } else if (command == "az") {  // Send current parameters -> setup command for matlab gui to read correct values
    Serial.print("Task\t"); Serial.println("twoSpouts task");               // identity: lets MATLAB verify correct script
    Serial.print("Max Omissions\t");
    Serial.println(maxOmissions);
    Serial.print("Max Timeout\t");
    Serial.println(maxTimeout);
    Serial.print("Max Trials\t");
    Serial.println(maxTrials);
    Serial.print("Max Rewards\t");
    Serial.println(maxRewards);
    Serial.print("Task LLLR\t");
    Serial.println(taskLLLR);
    Serial.print("Task 2ABT\t");
    Serial.println(task2ABT);
    Serial.print("Task Pavlovian\t");
    Serial.println(taskPavlovian);
    Serial.print("Min Block\t");
    Serial.println(minBlock);
    Serial.print("Max Block\t");
    Serial.println(maxBlock);
    Serial.print("Min Block Left\t");
    Serial.println(minBlockLeft);
    Serial.print("Max Block Left\t");
    Serial.println(maxBlockLeft);
    Serial.print("Min Block Right\t");
    Serial.println(minBlockRight);
    Serial.print("Max Block Right\t");
    Serial.println(maxBlockRight);
    Serial.print("Reaction Time\t");
    Serial.println(ReactionTime);
    // Serial.print("Outcome Time\t");
    // Serial.println(ConsumptionDuration);
    Serial.print("Correct Time\t");
    Serial.println(CorrectDuration);
    Serial.print("Incorrect Time\t");
    Serial.println(IncorrectDuration);
    Serial.print("Cue Time\t");
    Serial.println(ToneDuration);
    Serial.print("Cue Penalty\t");
    Serial.println(CuePenaltyDuration);
    Serial.print("Fails Until Reward\t");
    Serial.println(fails2reward);
    Serial.print("Left Calibration\t");
    Serial.println(RewardLeftMax);
    Serial.print("Right Calibration\t");
    Serial.println(RewardRightMax);
    Serial.print("Min ITI\t");
    Serial.println(minITI);
    Serial.print("Max ITI\t");
    Serial.println(maxITI);
    Serial.print("ITI Step\t");
    Serial.println(ITIstep);
    Serial.print("Min Left\t");
    Serial.println(RewardLeftMin);
    Serial.print("Max Left\t");
    Serial.println(RewardLeftMax);
    Serial.print("Left Step\t");
    Serial.println(RewardLeftStepSize);
    Serial.print("Min Right\t");
    Serial.println(RewardRightMin);
    Serial.print("Max Right\t");
    Serial.println(RewardRightMax);
    Serial.print("Right Step\t");
    Serial.println(RewardRightStepSize);
    Serial.print("Reward Probability\t");
    Serial.println(RewardProb);
    Serial.print("Reward Delay\t");
    Serial.println(RewardDelay);
    Serial.print("Spout Start\t");
    Serial.println(spoutStart);
    Serial.print("Left Cue Frequency\t");
    Serial.println(LeftCueFreq);
    Serial.print("Right Cue Frequency\t");
    Serial.println(RightCueFreq);
    Serial.print("Opto Enabled\t");
    Serial.println(OptoEnabled);
    Serial.print("Opto Continuous\t");
    Serial.println(optoContinuous);
    Serial.print("Opto Probability\t");
    Serial.println(OptoStimProb);
    Serial.print("Correct Trials Before Opto\t");
    Serial.println(numCorrectTrialsBeforeOpto);
    Serial.print("Opto Delay\t");
    Serial.println(OptoDelay);
    Serial.print("Opto Duration\t");
    Serial.println(OptoDuration);
    Serial.print("Pulse Duration\t");
    Serial.println(OptoPulseDuration);
    Serial.print("Opto Frequency\t");
    Serial.println(OptoFrequency);
    Serial.print("Opto Modulation\t");
    Serial.println(OptoMod);
    Serial.print("Opto Mod Continuously\t");
    Serial.println(OptoModContinuous);
    Serial.print("Opto Taper Off\t");
    Serial.println(OptoTaperOff);
    Serial.print("Opto At Block Switch\t");
    Serial.println(OptoActiveDuringBlockSwitch);
    Serial.print("Opto At ITI\t");
    Serial.println(OptoActiveDuringITI);
    Serial.print("Opto At Cue\t");
    Serial.println(OptoActiveDuringCue);
    Serial.print("Opto At Consumption\t");
    Serial.println(OptoActiveDuringConsumption);
    Serial.print("Opto At Error\t");
    Serial.println(OptoActiveDuringError);
    Serial.print("Opto At Outcome\t");
    Serial.println(OptoActiveDuringAnyOutcome);
    Serial.print("Error Light Duration\t");
    Serial.println(ErrorLightDuration);
    Serial.print("Multi-stim In ITI\t");
    Serial.println(AllowMultiOptoStimITI);
    Serial.print("Multi-stim In Cue\t");
    Serial.println(AllowMultiOptoStimCue);
    Serial.print("Multi-stim In Block\t");
    Serial.println(AllowMultiOptoStimBlock);
    Serial.print("Opto Random Light\t");
    Serial.println(ApplyOptoRandomLight);
    Serial.print("Cues Instructed\t");
    Serial.println(cuesInstructed);
    Serial.print("Pavlovian Mode\t");
    Serial.println(taskPavlovian);
    Serial.print("Reward Delay\t");
    Serial.println(RewardDelay);
    Serial.print("ENL Enabled\t");
    Serial.println(ENLEnabled);
    Serial.print("ENL Penalty Duration\t");
    Serial.println(ENLPenaltyDuration);
    Serial.print("Task Start Delay\t");
    Serial.println(TaskStartDelay);
    // Delayed-Response (DR) task parameters
    Serial.print("Task DelayedResponse\t");
    Serial.println(taskDelayedResponse);
    Serial.print("Delay Duration\t");
    Serial.println(DelayDuration);
    Serial.print("Start Cue Duration\t");
    Serial.println(StartCueDuration);
    Serial.print("Start Cue Frequency\t");
    Serial.println(StartCueFreq);
    Serial.print("Delay Penalty\t");
    Serial.println(DelayPenaltyDuration);
    Serial.print("Start Cue Penalty\t");
    Serial.println(StartCuePenaltyDuration);
    Serial.print("Opto At Delay\t");
    Serial.println(OptoActiveDuringDelay);
    Serial.print("Opto At StartCue\t");
    Serial.println(OptoActiveDuringStartCue);
    Serial.print("Multi-stim In Delay\t");
    Serial.println(AllowMultiOptoStimDelay);
    Serial.print("Multi-stim In StartCue\t");
    Serial.println(AllowMultiOptoStimStartCue);
    Serial.print("Pavlovian Lick To Start\t");
    Serial.println(PavlovianLickToStart);
    Serial.print("EEPROM settings\t");
    Serial.println(1);
  }
}