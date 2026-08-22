// This is a general one-spout state machine to run 1-spout tasks (Go-Nogo / Stop / Ratio schedules / Pavlovian)
// Sketch runs experiment and communicates with Bastijn's Matlab GUI (oneSpout.mlapp)
//
// State machine is build off teensy2spouts.ino:
//  - Single spout (the LEFT-side hardware on the breakout board) - all reward/lick logic uses one valve
//  - Trial-type semantics renamed LEFT/RIGHT -> A/B because there is no physical left/right
//      Go-Nogo:      A = Go trial (lick to reward)
//                    B = NoGo trial (withhold to reward)
//      Pavlovian:    A = CS+ (rewarded with prob)
//                    B = CS- (never rewarded)
//      Ratio:        A = FR or PR
//                    B = not in use
//      Stop-signal:  A = Go cue (# licks + selection time for reward)
//                    B = Stop (no licks after cue)
//  - Four mutually exclusive task modes: taskGoNogo (default), taskStop, taskRatio, taskPavlCond
//  - Same hardware pins / serial protocol style / EEPROM scheme / opto / sync as 2-spout

// Pavel's breakout board for Teensy 4
// PIN2 (PWM) - SpeakerPinA
// PIN3 (PWM) - SyncPin
// PIN4_SLN1 - copy SLN1
// PIN5_SLN2 - copy SLN2
// ANALOG_A1 - SpeakerPinB
// ANALOG_A0 - LickPin
// DAC1 (PIN41) - UnusedLickPin (kept defined for hardware compat with 2-spout)
// DAC0 (PIN40) - OptoPin
// PIN9  - OptoModPin
// PIN21 - RandomLightPin

// Serial-out buffering: same scheme as 2-spout. We append to serialOutBuffer (String)
// and updateSerialOutput() flushes up to MAX_CHARS_PER_LOOP characters per loop iteration.

#include <EEPROM.h>

// hardware setting for testing
const bool SpoutNotButton = false;  // true = spouts (pull down), false = buttons (pull up)

///////////////////////////////////////////////////////////////////////////////////////////////////////
// --------------------------- Modify pin numbers if needed ---------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Input/output pins - hardware identical to 2-spout, only one spout actually used
const byte SyncPin = 3;
const byte ExtSyncPin = 22;
const byte SpeakerPinA = 2;      // (was SpeakerPinL)
const byte SpeakerPinB = 15;     // (was SpeakerPinR)
const byte LickPin = 14;         // (was LeftLickPin) - the active spout's lick detector
const byte UnusedLickPin = 41;   // (was RightLickPin) - kept defined, not used
const byte RewardPin = 5;        // (was LeftRewardPin) - the active spout's solenoid
const byte UnusedRewardPin = 4;  // (was RightRewardPin) - kept defined, not used

// TTL pins
const byte CueAIndicatorPin = 17;          // (was LeftCueIndicatorPin) - HIGH during cue A
const byte CueBIndicatorPin = 18;          // (was RightCueIndicatorPin) - HIGH during cue B
const byte SpeakerTTLPin = 6;              // both speakers ttl
const byte RewardIndicatorPin = 19;        // (was LeftRewardIndicatorPin) - HIGH during reward
const byte UnusedRewardIndicatorPin = 20;  // (was RightRewardIndicatorPin) - kept defined, not used
const byte ITIIndicatorPin = 8;            // pulse that signals ITI period
const byte StartStopIndicatorPin = 10;     // start-stop session output pin
const byte OptoPin = 40;                   // opto control
const byte OptoTTLPin = 1;                 // opto reporter LED
const byte OptoModPin = 9;                 // opto modulation pin (0-5V via PWM + RC)
const byte RandomLightPin = 21;            // random light as background
const byte TrialStartPin = 7;              // TTL when trial starts
const byte CameraTriggerPin = 66;          // camera trigger (50Hz, 2ms pulses)
const byte ErrorLightPin = 66;             // also used as NO_REWARD signal
const byte ExtSyncReportPin = 12;          // report external sync on a pin

// Not currently in use
const bool buttonsAreWiredUp = false;
const byte StartStopButtonPin = 66;
const byte RewardButtonPin = 66;


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------ Task related variables (preserved by EEPROM) --------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// REWARD VARIABLES
unsigned long RewardDelay;        // delay between correct lick (or cue end in Pavlovian) and reward delivery (ms)
unsigned long RewardCurrentSize;  // duration that reward solenoid is left open (ms)
int RewardMin;                    // min reward size
int RewardMax;                    // max reward size
int RewardStepSize;               // reward size step (in ms)
int RewardRange[100];             // range of possible rewards from min to max
int numRewardRange;               // number of items in RewardRange

// Reward probability
int RewardProb;                    // probability of reward being dispensed if correct
int RewardProbList[100];           // reward probability list in steps of 1
bool giveReward = false;           // will we give reward this trial (correct outcome)
bool giveRewardIncorrect = false;  // will we give reward if incorrect outcome
bool correctSelection = false;     // did the animal do the expected behavior this trial

// Free Reward
int failCounter;        // tracks consecutive failures
int fails2reward;       // number of failures before triggering free reward
int freeRewardCounter;  // total free rewards this session

// Spout / first block start
int trialStart;  // 0 = random, 1 = block A first, 2 = block B first

// TASK MODE (mutually exclusive)
bool taskGoNogo;    // Go-Nogo task
bool taskStop;      // Stop task
bool taskPavlCond;  // Pavlovian conditioning (CS+/CS-)
bool taskRatio;     // Ratio Schedule task (Fixed or Progressive Ratio)
bool ratioFixed;    // true = Fixed Ratio, false = Progressive Ratio

// Pavlovian random reward
int PavlRandomRewardProb;          // % of Pavlovian trials that are silent random rewards (0 = off)
bool isRandomRewardTrial = false;  // set per-trial in incrementTrial(), read in CUE state

// ITI mode
bool ENLEnabled;  // if true, ITI becomes an enforced non-lick period

// Trial time parameters (ms)
unsigned long ReactionTime;         // selection window (Go in Go-Nogo, lick window in Stop)
unsigned long NogoHoldDuration;     // NoGo: lick-withhold window (starts at cue offset)
unsigned long ConsumptionDuration;  // consumption duration (set per trial)
unsigned long CorrectDuration;      // consumption period after correct/rewarded outcome
unsigned long IncorrectDuration;    // no-reward / incorrect period
unsigned long minITI;
unsigned long maxITI;
unsigned long ITIDurationList[100] = { 0 };
int NumITIDurations;
unsigned long ITIstep;
unsigned long TaskStartDelay;  // delay (s) between '8' and first NEW_TRIAL

// Lick thresholds
int GoLickThreshold;       // # licks during SELECTION needed for Go-hit
int StopLickThreshold;     // # licks during SELECTION needed for Stop-hit
int FRNumberLicks;         // # licks needed per trial in Fixed Ratio mode
int PRNumberStartLicks;    // # licks required on trial 1 in Progressive Ratio mode
int licksInSelection = 0;  // counts new licks since SELECTION entry (reset on entry)

// SSRT stop-signal task (block B stop cue)
bool SSDStaircase;           // true = diverging staircase, false = fixed-grid sampling
int SSDmin;                  // min stop-signal delay (ms)
int SSDmax;                  // max stop-signal delay (ms); staircase starts here
int SSDstep;                 // staircase / grid step (ms)
int SSDCurrent = 0;          // this trial's SSD (ms from SELECTION onset to stop cue)
bool stopCuePlayed = false;  // stop cue presented this trial?
bool stopCueActive = false;  // stop cue tone currently sounding?
int licksAtStopCue = 0;      // licksInSelection captured at stop-cue onset
unsigned long StopCueStartTime = 0;

// Ratio Schedule runtime (not EEPROM - reset every session)
int PRTrialNum = 0;      // which PR trial we're on (increments each trial)
int PRCurrentLicks = 0;  // computed lick threshold for current PR trial

// Block length (A and B trial types)
int BlockLengthsA[1000];
int BlockLengthsB[1000];
int minBlock;
int maxBlock;
int minBlockA;
int maxBlockA;
int minBlockB;
int maxBlockB;
int numBlockLengthsA;
int numBlockLengthsB;

// Block-switch mode toggle.
//   false (default) -> end the current block after BlockLength TRIALS (legacy behavior)
//   true            -> end the current block after BlockLength CORRECT TRIALS
//                      (CorrectTrialCount >= BlockLength). Counted non-consecutively:
//                      #_INCORRECT and #_TIMEOUT trials extend the block but don't
//                      contribute toward the count.
//
// Why correct trials and not rewards: in Stop block B the reward probability is the
// inverse of RewardProb (typically ~10%), so counting rewards would extend B blocks
// near-indefinitely. Stop B trials still emit #_CORRECT when the animal hits
// StopLickThreshold (regardless of whether the dice roll delivered reward), so counting
// correct trials gives a sensible block end on both A and B.
//
// Special case: Pavlovian CS- (BLOCK_B) emits #_CORRECT but does NOT increment
// CorrectTrialCount (CS- is never rewarded and not a behavioral success), so
// CorrectTrialCount stays 0 the whole block. When the animal is in a Pavl CS- block
// we silently fall back to the trial-count rule or the block would never end.
bool BlockSwitchAfterCorrectTrials;

// OPTOGENETIC VARIABLES
bool OptoActiveDuringITI;
bool OptoActiveDuringCue;
bool OptoActiveDuringConsumption;
bool OptoActiveDuringBlockSwitch;
bool OptoActiveDuringError;
bool OptoActiveDuringAnyOutcome;

unsigned long OptoStimProb;
unsigned long OptoDelay;
unsigned long OptoDuration;
bool optoContinuous;
unsigned long OptoPulseDuration;
unsigned long OptoFrequency;
unsigned long OptoMod;
unsigned long OptoTaperOff;
bool OptoModContinuous;

int numCorrectTrialsBeforeOpto;
bool AllowMultiOptoStimBlock;
bool AllowMultiOptoStimITI;
bool AllowMultiOptoStimCue;
bool ApplyOptoRandomLight;
unsigned long RandomLightTimer;
unsigned long RandomLightInterval;
int RandomLightON;

// Cue variables
unsigned long CueAFreq;  // (was LeftCueFreq)  Go-Nogo: Go cue,  Pavl: CS+
unsigned long CueBFreq;  // (was RightCueFreq) Go-Nogo: NoGo cue, Pavl: CS-
unsigned long ToneDuration;

// MISC
unsigned long ErrorLightDuration;
unsigned long ENLPenaltyDuration;
unsigned long CuePenaltyDuration;

bool cuesInstructed;  // false = both speakers play same cue, true = A on speakerA / B on speakerB

bool sessionStart;
bool sessionEnd;

int maxOmissions;
int maxTimeout;
int maxTrials;
int maxRewards;

bool sendTrialStartTTL = false;
bool trialStartPulseActive = false;
unsigned long TrialStartTTLDuration = 500;


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Define counters -------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Sync
unsigned long TimerSync = 0;
unsigned long SyncPulseInterval = 1000;
int SyncPulseStatus = 0;
unsigned long TrialStartTimer = 0;

// External sync
int ExtSyncState = 0;
unsigned long ExtSyncRiseTime = 0;
unsigned long ExtSyncLastFallTime = 0;
bool firstExtSync = true;

unsigned long SessionStartTime = 0;
unsigned long StartDelayTimer = 0;

// Trial stats
int TrialNum = 0;
int BlockNum = 0;
int TrialInBlock = 0;
int Num_Reward = 0;
int CorrectTrialCount = 0;
int incorrectCounter = 0;

const int BLOCK_A = 0;
const int BLOCK_B = 1;
int BlockType = BLOCK_A;

// Block counters (kept generic - no LLLR-specific stats)
int Num_ENLPenalty = 0;
int Num_Omissions = 0;
int freeRewardBlockCounter = 0;
int BlockRewardCounter = 0;
int BlockNoRewardCounter = 0;
int LickCounter = 0;
bool firstTrial = true;

// "Quality of block" counters - same semantics as teensy2spouts:
//   TrialsUntilFirstReward = TrialInBlock at the moment the first REWARD entry occurs
//                            in the current block (0 means no reward yet this block).
//   badTrialsCounter       = number of #_INCORRECT or #_TIMEOUT trials AFTER the first
//                            reward in the current block (i.e. mistakes made by an animal
//                            that has already demonstrated it understands the contingency).
const int MAX_BLOCKS = 1024;
int TrialsUntilFirstReward = 0;
int TrialsUntilFirstRewardHistory[MAX_BLOCKS];
int badTrialsCounter = 0;

// Trial structure
int BlockLength = 0;
unsigned long ITIDuration = 0;

// Opto bookkeeping
bool OptoEnabled = false;
bool optoTriggeredITI = false;
bool optoTriggeredCue = false;
bool optoTriggeredConsumption = false;
bool optoTriggeredBlockSwitch = false;
bool optoTriggeredError = false;
bool optoTriggeredAnyOutcome = false;
bool duringOptoStim = false;
bool duringOptoStimDelay = false;
unsigned long optoStartTime = 0;
bool optoPulseActive = false;
bool optoIPIActive = false;
bool optoBlockDone = false;
bool optoTrialDone = false;
bool optoCueLick = false;
unsigned long optoStartPulse = 0;
unsigned long optoTimeInPulse = 0;
bool optoBlockFirstTrial = false;
unsigned long optoIPI = 0;
unsigned long timeInStim = 0;
unsigned long optoTaperStartTime = 0;
bool optoTaperActive = false;
unsigned long timeInTaper = 0;
unsigned long tmpOptoMod = 0;
bool manualOptoTestActive = false;

// Error/no-reward light bookkeeping
bool errorLightActive = false;
unsigned long errorLightStartTime = 0;

// Lick detection (single spout)
bool LickOccuring = false;
unsigned long LickStartTime = 0;

// Manual reward
int RewardButtonStatus = 0;
unsigned long RewardTimer = 0;

// Timestamps
unsigned long ITIStartTime = 0;
unsigned long PenaltyStartTime = 0;
unsigned long CueStartTime = 0;
unsigned long CueOffTime = 0;
unsigned long SelectionStartTime = 0;
unsigned long RewardStartTime = 0;
unsigned long RewardOffTime = 0;
unsigned long ConsumptionStartTime = 0;
unsigned long CameraTrigStartTime = 0;

// Serial buffer
String serialOutBuffer = "";
bool pausePrint = false;

// Loop timing
unsigned long maxLoopTime_us = 0;
unsigned long cumulativeLoopTime_us = 0;
unsigned long loopCounter = 0;
unsigned long loopStartTime_us = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Define states ---------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

enum state {
  IDLE,
  START_DELAY,
  NEW_TRIAL,
  ITI,
  ENL_PENALTY,
  CUE,
  CUE_PENALTY,
  SELECTION,
  REWARD_DELAY,
  PRE_REWARD,
  FREE_REWARD,
  NO_REWARD,
  REWARD,
  CONSUMPTION
};
typedef enum state OneSpoutState;

char SerialInput = '0';
OneSpoutState CurrentState = IDLE;
OneSpoutState NextState = IDLE;
int StartStopButtonStatus = 1;


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ----------------------------------- EEPROM settings --------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Append-only positional storage (same scheme as 2-spout). If you add a parameter, append it
// to the relevant array - existing addresses must not move.
int* intParams[] = {
  &minBlock, &maxBlock,
  &minBlockA, &maxBlockA, &minBlockB, &maxBlockB,
  &RewardMin, &RewardMax, &RewardStepSize,
  &fails2reward, &RewardProb, &trialStart,
  &numCorrectTrialsBeforeOpto,
  &maxTimeout, &maxOmissions, &maxTrials, &maxRewards,
  &GoLickThreshold,
  &StopLickThreshold,
  &PavlRandomRewardProb,
  &FRNumberLicks,
  &PRNumberStartLicks,
  &SSDmin,
  &SSDmax,
  &SSDstep
};

bool* boolParams[] = {
  &taskGoNogo,
  &taskStop,
  &taskRatio,
  &taskPavlCond,
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
  &OptoActiveDuringError,
  &OptoActiveDuringAnyOutcome,
  &BlockSwitchAfterCorrectTrials,
  &ratioFixed,
  &SSDStaircase
};

unsigned long* ulongParams[] = {
  &minITI, &maxITI, &ITIstep,
  &CueAFreq, &CueBFreq,
  &ReactionTime, &CuePenaltyDuration,
  &RewardCurrentSize,
  &OptoStimProb, &OptoDelay, &OptoDuration,
  &OptoPulseDuration, &OptoFrequency, &OptoMod, &OptoTaperOff,
  &CorrectDuration, &IncorrectDuration,
  &ToneDuration,
  &ErrorLightDuration,
  &TaskStartDelay,
  &NogoHoldDuration,
  &RewardDelay,
  &ENLPenaltyDuration
};

const int numIntParams = sizeof(intParams) / sizeof(intParams[0]);
const int numBoolParams = sizeof(boolParams) / sizeof(boolParams[0]);
const int numUlongParams = sizeof(ulongParams) / sizeof(ulongParams[0]);


///////////////////////////////////////////////////////////////////////////////////////////////////////
// --------------------------- Setup and loop ------------------------------------------------------ //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);      // used to be 38400
  analogWriteResolution(8);  // 8-bit resolution for analogWrite (0-255)

  // get EERPOM variables
  getParams();

  // Ensure exactly one task mode is active (default to Go-Nogo on first boot)
  if (!(taskGoNogo || taskStop || taskPavlCond || taskRatio)) {
    taskGoNogo = true;
    taskStop = false;
    taskPavlCond = false;
    taskRatio = false;
    ratioFixed = false;
  }

  // Pin modes
  pinMode(SyncPin, OUTPUT);
  pinMode(ExtSyncPin, INPUT_PULLDOWN);
  pinMode(RewardPin, OUTPUT);
  pinMode(UnusedRewardPin, OUTPUT);
  pinMode(RewardIndicatorPin, OUTPUT);
  pinMode(UnusedRewardIndicatorPin, OUTPUT);
  pinMode(LickPin, INPUT_PULLDOWN);
  pinMode(UnusedLickPin, INPUT_PULLDOWN);
  pinMode(SpeakerPinA, OUTPUT);
  pinMode(SpeakerPinB, OUTPUT);
  pinMode(SpeakerTTLPin, OUTPUT);
  pinMode(CueAIndicatorPin, OUTPUT);
  pinMode(CueBIndicatorPin, OUTPUT);
  pinMode(CameraTriggerPin, OUTPUT);
  pinMode(StartStopIndicatorPin, OUTPUT);
  pinMode(OptoPin, OUTPUT);
  pinMode(OptoTTLPin, OUTPUT);
  pinMode(OptoModPin, OUTPUT);
  pinMode(ITIIndicatorPin, OUTPUT);
  pinMode(TrialStartPin, OUTPUT);
  pinMode(ErrorLightPin, OUTPUT);
  if (buttonsAreWiredUp) {
    pinMode(StartStopButtonPin, INPUT);
    pinMode(RewardButtonPin, INPUT);
  }
  pinMode(ExtSyncReportPin, OUTPUT);

  CameraTrigStartTime = millis();
  CurrentState = IDLE;
  SyncPulseStatus = 0;

  digitalWrite(SyncPin, LOW);
  digitalWrite(RewardPin, LOW);
  digitalWrite(UnusedRewardPin, LOW);
  digitalWrite(RewardIndicatorPin, LOW);
  digitalWrite(UnusedRewardIndicatorPin, LOW);
  digitalWrite(ITIIndicatorPin, LOW);
  analogWrite(SpeakerPinA, 0);
  analogWrite(SpeakerPinB, 0);
  digitalWrite(SpeakerTTLPin, LOW);
  digitalWrite(CameraTriggerPin, LOW);
  digitalWrite(StartStopIndicatorPin, LOW);
  digitalWrite(OptoPin, LOW);
  digitalWrite(OptoTTLPin, LOW);
  digitalWrite(OptoModPin, LOW);
  digitalWrite(TrialStartPin, LOW);
  digitalWrite(ErrorLightPin, LOW);
  digitalWrite(ExtSyncReportPin, LOW);
  randomSeed(analogRead(3));

  Serial.println("-----------------------------------------------------------------");
  Serial.println("Manual check: 1 -> reward;");
  Serial.println("Trial start/stop: 8 -> start; 9 -> end");
  Serial.println("Tasks: Go-Nogo (default) | Stop | Pavlovian CS+/CS-");
  Serial.println("-----------------------------------------------------------------");
  Serial.println("");
  Serial.println("");
  printDataHeader();
  serialOutBuffer.reserve(1024);
  loopStartTime_us = micros();

  fillRewardProbList();
}


void loop() {
  checkSerialInput();
  updateSyncPulses();
  updateExtSyncDetection();
  lickDetection();
  updateStateMachine();
  updateCameraTrigger();
  updateOptoStim();
  updateSerialOutput();
  randomLightFunction();
  turnOffManualCue();
  sendTrialStartPulse();

  unsigned long now_us = micros();
  loopCounter++;
  cumulativeLoopTime_us += (now_us - loopStartTime_us);
  maxLoopTime_us = max(maxLoopTime_us, now_us - loopStartTime_us);
  loopStartTime_us = now_us;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------- State machine (Go-Nogo / Stop / Pavlovian CS+/CS-) ----------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void updateStateMachine() {
  bool enteredNewState = false;
  if (NextState != CurrentState) {
    enteredNewState = true;
    CurrentState = NextState;
    outputTrialState();
  }

  //   ----------------------------   IDLE   ----------------------------
  if (CurrentState == IDLE) {
    if (enteredNewState) {
      TrialNum = 0;
      BlockNum = 0;
      Num_Reward = 0;
      TrialInBlock = 0;
      CorrectTrialCount = 0;
      incorrectCounter = 0;
      Num_ENLPenalty = 0;
      Num_Omissions = 0;
      failCounter = 0;
      freeRewardCounter = 0;
      freeRewardBlockCounter = 0;
      BlockRewardCounter = 0;
      BlockNoRewardCounter = 0;
      LickCounter = 0;
      licksInSelection = 0;
      firstTrial = true;
      TrialsUntilFirstReward = 0;
      badTrialsCounter = 0;

      BlockLength = 0;
      PRTrialNum = 0;
      PRCurrentLicks = 0;
      digitalWrite(RewardPin, LOW);
      digitalWrite(RewardIndicatorPin, LOW);
      digitalWrite(ITIIndicatorPin, LOW);
      analogWrite(SpeakerPinA, 0);
      analogWrite(SpeakerPinB, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      digitalWrite(CameraTriggerPin, LOW);
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
      digitalWrite(ErrorLightPin, LOW);
      errorLightActive = false;
    }

    //   ----------------------------   START_DELAY   ----------------------------
  } else if (CurrentState == START_DELAY) {
    if (millis() - StartDelayTimer >= TaskStartDelay * 1000UL) {
      NextState = NEW_TRIAL;
    }

    //   ----------------------------   NEW_TRIAL   ----------------------------
  } else if (CurrentState == NEW_TRIAL) {
    sendTrialStartTTL = true;

    if (OptoActiveDuringCue) optoCueLick = true;

    if (fails2reward > 0 && failCounter >= fails2reward) {
      NextState = FREE_REWARD;
      failCounter = 0;
      freeRewardCounter += 1;
      freeRewardBlockCounter += 1;
      serialOutBuffer += "Free Rewards";
      serialOutBuffer += '\t';
      serialOutBuffer += freeRewardCounter;
      serialOutBuffer += '\n';
    } else {
      digitalWrite(RewardPin, LOW);
      digitalWrite(RewardIndicatorPin, LOW);
      analogWrite(SpeakerPinA, 0);
      analogWrite(SpeakerPinB, 0);
      digitalWrite(SpeakerTTLPin, LOW);

      ITIStartTime = millis();
      digitalWrite(StartStopIndicatorPin, LOW);
      digitalWrite(ITIIndicatorPin, HIGH);

      int randomIdx = random(0, NumITIDurations);
      ITIDuration = ITIDurationList[randomIdx];

      incrementTrial();
      NextState = ITI;
    }

    //   ----------------------------   ITI   ----------------------------
  } else if (CurrentState == ITI) {
    if (enteredNewState) {
      ITIStartTime = millis();
      digitalWrite(ITIIndicatorPin, HIGH);

      // block-switch opto
      if (optoTriggeredBlockSwitch) {
        startOptoStim();
        outputEvent("Opto_blockSwitch_p", millis(), OptoDuration);
      } else if (OptoActiveDuringBlockSwitch && optoBlockFirstTrial && !optoTrialDone) {
        if (random(100) < OptoStimProb) {
          optoTriggeredBlockSwitch = true;
          optoBlockFirstTrial = false;
          optoTrialDone = true;
          startOptoStim();
          outputEvent("Opto_blockSwitch", millis(), OptoDuration);
          if (!AllowMultiOptoStimBlock) optoBlockDone = true;
        }
      }

      // ITI opto
      if (optoTriggeredITI) {
        startOptoStim();
        outputEvent("Opto_ENL_p", millis(), OptoDuration);
      } else if (OptoActiveDuringITI && !optoBlockDone && !firstTrial && !optoTrialDone) {
        if (numCorrectTrialsBeforeOpto == 0 || CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTriggeredITI = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_ITI", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }
    }

    if (ENLEnabled && LickOccuring) {
      digitalWrite(ITIIndicatorPin, LOW);
      if (!(AllowMultiOptoStimITI && optoTriggeredITI)) optoTriggeredITI = false;
      if (!(AllowMultiOptoStimBlock && optoTriggeredBlockSwitch)) optoTriggeredBlockSwitch = false;
      NextState = ENL_PENALTY;
    }

    if (millis() - ITIStartTime >= ITIDuration) {
      digitalWrite(ITIIndicatorPin, LOW);
      optoTriggeredBlockSwitch = false;
      optoBlockFirstTrial = false;
      NextState = CUE;
    }

    //   ----------------------------   ENL_PENALTY   ----------------------------
  } else if (CurrentState == ENL_PENALTY) {
    optoTaperActive = false;
    if (enteredNewState) {
      PenaltyStartTime = millis();
      Num_ENLPenalty += 1;
    }
    if (millis() - PenaltyStartTime >= ENLPenaltyDuration) {
      if (!LickOccuring) NextState = ITI;
    }

    //   ----------------------------   CUE   ----------------------------
  } else if (CurrentState == CUE) {
    if (enteredNewState) {
      CueStartTime = millis();

      // Cue opto
      if (optoTriggeredCue) {
        startOptoStim();
        outputEvent("Opto_cue_p", millis(), OptoDuration);
      } else if (OptoActiveDuringCue && !optoBlockDone && !optoTrialDone) {
        if (numCorrectTrialsBeforeOpto == 0 || CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
          if (random(100) < OptoStimProb) {
            optoTriggeredCue = true;
            optoTrialDone = true;
            startOptoStim();
            outputEvent("Opto_cue", millis(), OptoDuration);
            if (!AllowMultiOptoStimBlock) optoBlockDone = true;
          }
        }
      }

      // Present cue (A or B based on BlockType). Skipped entirely for random-reward trials.
      if (!isRandomRewardTrial) {
        if (BlockType == BLOCK_A || taskStop) {
          if (cuesInstructed) {
            analogWriteFrequency(SpeakerPinA, CueAFreq);
            analogWrite(SpeakerPinA, 128);
            analogWrite(SpeakerPinB, 0);
          } else {
            analogWriteFrequency(SpeakerPinA, CueAFreq);
            analogWrite(SpeakerPinA, 128);
            analogWriteFrequency(SpeakerPinB, CueAFreq);
            analogWrite(SpeakerPinB, 128);
          }
          digitalWrite(SpeakerTTLPin, HIGH);
          digitalWrite(CueAIndicatorPin, HIGH);
        } else {
          if (cuesInstructed) {
            analogWrite(SpeakerPinA, 0);
            analogWriteFrequency(SpeakerPinB, CueBFreq);
            analogWrite(SpeakerPinB, 128);
          } else {
            analogWriteFrequency(SpeakerPinA, CueBFreq);
            analogWrite(SpeakerPinA, 128);
            analogWriteFrequency(SpeakerPinB, CueBFreq);
            analogWrite(SpeakerPinB, 128);
          }
          digitalWrite(SpeakerTTLPin, HIGH);
          digitalWrite(CueBIndicatorPin, HIGH);
        }
      }
    }

    // cue penalty (lick during cue) — not applicable for random-reward trials (no cue played)
    if (CuePenaltyDuration > 0 && LickOccuring && !isRandomRewardTrial) {
      digitalWrite(CueAIndicatorPin, LOW);
      digitalWrite(CueBIndicatorPin, LOW);
      analogWrite(SpeakerPinA, 0);
      analogWrite(SpeakerPinB, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      if (OptoActiveDuringCue) optoCueLick = true;
      if (!(AllowMultiOptoStimCue && optoTriggeredCue)) optoTriggeredCue = false;
      NextState = CUE_PENALTY;
    }

    // end of cue
    if (millis() - CueStartTime >= ToneDuration) {
      CueOffTime = millis();
      digitalWrite(CueAIndicatorPin, LOW);
      digitalWrite(CueBIndicatorPin, LOW);
      analogWrite(SpeakerPinA, 0);
      analogWrite(SpeakerPinB, 0);
      digitalWrite(SpeakerTTLPin, LOW);
      firstTrial = false;
      optoTriggeredITI = false;
      optoTriggeredCue = false;

      if (taskPavlCond) {
        // Pavlovian: no behavioral choice. All trial types emit #_CORRECT (neither CS- nor
        // random-reward is a mistake by the animal). Random-reward trials additionally use
        // the special #_RANDOM_REWARD label so the GUI can distinguish them.
        //   Random reward -> #_RANDOM_REWARD, correctSelection=true  (always rewarded)
        //   CS+ (BLOCK_A) -> #_CORRECT,       correctSelection=true  (rewarded per RewardProb)
        //   CS- (BLOCK_B) -> #_CORRECT,       correctSelection=false (never rewarded; uses IncorrectDuration)
        // giveReward / giveRewardIncorrect were set once in incrementTrial() — not touched here.
        // CONSUMPTION duration follows correctSelection (CorrectDuration vs IncorrectDuration).
        if (isRandomRewardTrial) {
          correctSelection = true;
          outputEvent("#_RANDOM_REWARD", millis(), 0);
          // not CS+/CS-, do not increment CorrectTrialCount
        } else if (BlockType == BLOCK_A) {
          correctSelection = true;
          outputEvent("#_CORRECT", millis(), 0);
          CorrectTrialCount++;
        } else {
          correctSelection = false;
          outputEvent("#_CORRECT", millis(), 0);
          // CS- is not an animal mistake; badTrialsCounter and incorrectCounter not incremented
        }
        if (correctSelection) handleCorrectOpto();
        handleAnyOutcomeOpto();
        NextState = REWARD_DELAY;
      } else {
        NextState = SELECTION;
      }
    }

    //   ----------------------------   CUE_PENALTY   ----------------------------
  } else if (CurrentState == CUE_PENALTY) {
    if (enteredNewState) PenaltyStartTime = millis();
    if (millis() - PenaltyStartTime >= CuePenaltyDuration) {
      if (!LickOccuring) {
        optoCueLick = false;
        NextState = ITI;
      }
    }

    //   ----------------------------   REWARD_DELAY   ----------------------------
  } else if (CurrentState == REWARD_DELAY) {
    if (enteredNewState) PenaltyStartTime = millis();
    if (millis() - PenaltyStartTime >= RewardDelay) NextState = PRE_REWARD;

    //   ----------------------------   SELECTION   ----------------------------
  } else if (CurrentState == SELECTION) {
    if (enteredNewState) {
      SelectionStartTime = millis();
      licksInSelection = 0;
      stopCuePlayed = false;  // SSRT: reset stop-cue state each SELECTION
      stopCueActive = false;
      licksAtStopCue = 0;
    }

    correctSelection = false;

    if (taskGoNogo) {
      // Go/Nogo task: Go trials animals needs to lick, Nogo trials animal needs to withold licking
      if (BlockType == BLOCK_A) {
        // Go trial:
        //   licks >= GoLickThreshold within ReactionTime  -> #_CORRECT (reward per RewardProb)
        //   ReactionTime expired with some licks but < threshold -> #_INCORRECT (no reward)
        //   ReactionTime expired with zero licks                 -> #_TIMEOUT  (no reward)
        if (licksInSelection >= GoLickThreshold) {
          correctSelection = true;
          outputEvent("#_CORRECT", millis(), 0);
          failCounter = 0;
          CorrectTrialCount++;
          handleCorrectOpto();
          handleAnyOutcomeOpto();
          NextState = REWARD_DELAY;
        } else if (ReactionTime > 0 && millis() - SelectionStartTime >= ReactionTime) {
          if (licksInSelection > 0) {
            // some licks but not enough -> incorrect
            outputEvent("#_INCORRECT", millis(), 0);
            failCounter += 1;
            incorrectCounter += 1;
            if (TrialsUntilFirstReward != 0) badTrialsCounter++;
            handleErrorOpto();
            handleAnyOutcomeOpto();
            fireErrorLight();
          } else {
            // zero licks -> omission (treated as an error for opto purposes:
            // for a Go trial the animal failed to respond, so OptoActiveDuringError applies)
            outputEvent("#_TIMEOUT", millis(), 0);
            failCounter += 1;
            Num_Omissions += 1;
            if (TrialsUntilFirstReward != 0) badTrialsCounter++;
            handleErrorOpto();
            handleAnyOutcomeOpto();
            fireErrorLight();
          }
          NextState = CONSUMPTION;  // no reward path
        }
      } else {
        // NoGo trial:
        //   any lick during NogoHoldDuration -> #_INCORRECT (no reward)
        //   held the full NogoHoldDuration   -> #_CORRECT (reward per RewardProb)
        // We check raw LickOccuring (matches teensy2spouts choice-detection pattern):
        // a lick that started during the cue and is still in progress when SELECTION
        // begins also counts as a Nogo failure.
        if (LickOccuring) {
          outputEvent("#_INCORRECT", millis(), 0);
          failCounter += 1;
          incorrectCounter += 1;
          if (TrialsUntilFirstReward != 0) badTrialsCounter++;
          handleErrorOpto();
          handleAnyOutcomeOpto();
          fireErrorLight();
          NextState = REWARD_DELAY;  // PRE_REWARD will route to NO_REWARD via giveRewardIncorrect=false
        } else if (millis() - SelectionStartTime >= NogoHoldDuration) {
          correctSelection = true;
          outputEvent("#_CORRECT", millis(), 0);
          failCounter = 0;
          CorrectTrialCount++;
          handleCorrectOpto();
          handleAnyOutcomeOpto();
          NextState = REWARD_DELAY;
        }
      }
    } else if (taskStop) {
      // STOP-SIGNAL TASK
      //  Block A (Reward/go): reward needs >= StopLickThreshold licks AND the full
      //    ReactionTime window to elapse.  -> #_CORRECT / #_INCORRECT / #_TIMEOUT
      //  Block B (Stop): go cue already played in CUE; present the stop cue (tone B)
      //    at SSDCurrent after SELECTION onset, then require NO new lick from stop-cue
      //    onset to the end of ReactionTime.  -> #_STOP_SUCCESS / #_STOP_FAILURE
      //  Reward on correct outcomes follows RewardProb in both blocks (set in incrementTrial()).
      if (BlockType == BLOCK_A) {
        if (ReactionTime > 0 && millis() - SelectionStartTime >= ReactionTime) {
          if (licksInSelection >= StopLickThreshold) {
            correctSelection = true;
            outputEvent("#_CORRECT", millis(), 0);
            failCounter = 0;
            CorrectTrialCount++;
            handleCorrectOpto();
            handleAnyOutcomeOpto();
            NextState = REWARD_DELAY;
          } else if (licksInSelection > 0) {
            outputEvent("#_INCORRECT", millis(), 0);
            failCounter += 1;
            incorrectCounter += 1;
            if (TrialsUntilFirstReward != 0) badTrialsCounter++;
            handleErrorOpto();
            handleAnyOutcomeOpto();
            fireErrorLight();
            NextState = CONSUMPTION;
          } else {
            outputEvent("#_TIMEOUT", millis(), 0);
            failCounter += 1;
            Num_Omissions += 1;
            if (TrialsUntilFirstReward != 0) badTrialsCounter++;
            handleErrorOpto();
            handleAnyOutcomeOpto();
            fireErrorLight();
            NextState = CONSUMPTION;
          }
        }
      } else {
        // Block B: stop trial.
        // 1) present the stop cue once, at the (clamped) SSD after SELECTION onset.
        //    Clamp so the cue always has >= ToneDuration to sound before the window ends;
        //    otherwise a large SSD fires the cue and the window-end block silences it in the
        //    same pass -> #_STOP_CUE is logged but nothing is heard.
        int effSSD = SSDCurrent;
        int maxSSD = (int)ReactionTime - (int)ToneDuration;
        if (maxSSD < 0) maxSSD = 0;
        if (effSSD > maxSSD) effSSD = maxSSD;
        if (!stopCuePlayed && millis() - SelectionStartTime >= (unsigned long)effSSD) {
          stopCuePlayed = true;
          stopCueActive = true;
          StopCueStartTime = millis();
          CueStartTime = millis();   // makes turnOffManualCue() hold cue B for ToneDuration (like cue A)
          licksAtStopCue = licksInSelection;  // baseline: any lick beyond this = failed stop
          if (cuesInstructed) {
            analogWrite(SpeakerPinA, 0);
            analogWriteFrequency(SpeakerPinB, CueBFreq);
            analogWrite(SpeakerPinB, 128);
          } else {
            analogWriteFrequency(SpeakerPinA, CueBFreq);
            analogWrite(SpeakerPinA, 128);
            analogWriteFrequency(SpeakerPinB, CueBFreq);
            analogWrite(SpeakerPinB, 128);
          }
          digitalWrite(SpeakerTTLPin, HIGH);
          digitalWrite(CueBIndicatorPin, HIGH);
          outputEvent("CUE_B", millis(), ToneDuration);   // cue B played for the cue duration
          outputEvent("#_STOP_CUE", millis(), effSSD);    // stop-signal delay actually used (inhibition duration);
        }
        // 2) turn the stop cue off after ToneDuration
        if (stopCueActive && millis() - StopCueStartTime >= ToneDuration) {
          stopCueActive = false;
          analogWrite(SpeakerPinA, 0);
          analogWrite(SpeakerPinB, 0);
          digitalWrite(SpeakerTTLPin, LOW);
          digitalWrite(CueBIndicatorPin, LOW);
        }
        // 3) score at end of the selection window
        if (ReactionTime > 0 && millis() - SelectionStartTime >= ReactionTime) {
          analogWrite(SpeakerPinA, 0);
          analogWrite(SpeakerPinB, 0);
          digitalWrite(SpeakerTTLPin, LOW);
          digitalWrite(CueBIndicatorPin, LOW);
          stopCueActive = false;
          bool lickedAfterStop = stopCuePlayed && (licksInSelection > licksAtStopCue);
          if (lickedAfterStop) {
            outputEvent("#_STOP_FAILURE", millis(), 0);
            failCounter += 1;
            incorrectCounter += 1;
            if (TrialsUntilFirstReward != 0) badTrialsCounter++;
            handleErrorOpto();
            handleAnyOutcomeOpto();
            fireErrorLight();
            updateSSDStaircase(false);
            NextState = CONSUMPTION;
          } else {
            correctSelection = true;
            outputEvent("#_STOP_SUCCESS", millis(), 0);
            failCounter = 0;
            CorrectTrialCount++;
            handleCorrectOpto();
            handleAnyOutcomeOpto();
            updateSSDStaircase(true);
            NextState = REWARD_DELAY;
          }
        }
      }

    } else if (taskRatio) {
      // Ratio Schedule: animal needs PRCurrentLicks licks within ReactionTime.
      // PRCurrentLicks is set each trial in incrementTrial():
      //   Fixed Ratio:       constant FRNumberLicks every trial
      //   Progressive Ratio: Richardson & Roberts (1996) exponential escalation, k=0.2
      if (licksInSelection >= PRCurrentLicks) {
        correctSelection = true;
        outputEvent("#_CORRECT", millis(), 0);
        failCounter = 0;
        CorrectTrialCount++;
        handleCorrectOpto();
        handleAnyOutcomeOpto();
        NextState = REWARD_DELAY;
      } else if (ReactionTime > 0 && millis() - SelectionStartTime >= ReactionTime) {
        if (licksInSelection > 0) {
          outputEvent("#_INCORRECT", millis(), 0);
          failCounter += 1;
          incorrectCounter += 1;
          if (TrialsUntilFirstReward != 0) badTrialsCounter++;
          handleErrorOpto();
          handleAnyOutcomeOpto();
          fireErrorLight();
        } else {
          // zero licks -> omission (treated as an error for opto purposes:
          // Ratio task requires PRCurrentLicks licks, so 0 licks is a failure
          // to perform the required action)
          outputEvent("#_TIMEOUT", millis(), 0);
          failCounter += 1;
          Num_Omissions += 1;
          if (TrialsUntilFirstReward != 0) badTrialsCounter++;
          handleErrorOpto();
          handleAnyOutcomeOpto();
          fireErrorLight();
        }
        NextState = CONSUMPTION;
      }
    }
    // taskPavlCond never enters SELECTION

    //   ----------------------------   PRE_REWARD   ----------------------------
  } else if (CurrentState == PRE_REWARD) {
    if (enteredNewState) {
      if (OptoActiveDuringCue) optoCueLick = true;

      bool shouldReward = correctSelection ? giveReward : giveRewardIncorrect;
      if (shouldReward) {
        NextState = REWARD;
        BlockRewardCounter += 1;
      } else {
        NextState = NO_REWARD;
        BlockNoRewardCounter += 1;
      }
    }

    //   ----------------------------   NO_REWARD   ----------------------------
  } else if (CurrentState == NO_REWARD) {
    if (enteredNewState) {
      // NOTE: ErrorLightPin is intentionally NOT fired here. It only fires on:
      //   - Go-Nogo Go #_INCORRECT or #_TIMEOUT                       (SELECTION)
      //   - Go-Nogo NoGo #_INCORRECT (animal failed to hold)           (SELECTION)
      //   - Stop A/B   #_INCORRECT or #_TIMEOUT                       (SELECTION)
      // It does NOT fire for probabilistic no-reward outcomes after a #_CORRECT choice,
      // and never fires for Pavlovian (CS- is not an "error").
      //
      // RewardStartTime is intentionally not refreshed here, so this state passes through
      // immediately - the actual no-reward "duration" lives in CONSUMPTION (IncorrectDuration).
    }
    if ((millis() - RewardStartTime) > RewardCurrentSize) {
      digitalWrite(RewardPin, LOW);
      digitalWrite(RewardIndicatorPin, LOW);
      NextState = CONSUMPTION;
    }

    //   ----------------------------   FREE_REWARD   ----------------------------
  } else if (CurrentState == FREE_REWARD) {
    if (enteredNewState) {
      digitalWrite(RewardPin, HIGH);
      digitalWrite(RewardIndicatorPin, HIGH);
      RewardStartTime = millis();
    }
    if ((millis() - RewardStartTime) > RewardCurrentSize) {
      digitalWrite(RewardPin, LOW);
      digitalWrite(RewardIndicatorPin, LOW);
      NextState = CONSUMPTION;
    }

    //   ----------------------------   REWARD   ----------------------------
  } else if (CurrentState == REWARD) {
    if (enteredNewState) {
      digitalWrite(RewardPin, HIGH);
      digitalWrite(RewardIndicatorPin, HIGH);
      Num_Reward += 1;
      RewardStartTime = millis();
      // record the trial number on which the first reward of this block was delivered
      // (used by the GUI / CSV to flag "good" blocks). Subsequent rewards within the
      // same block don't update this value, but DO mean later mistakes count as
      // badTrialsCounter increments (see #_INCORRECT / #_TIMEOUT handlers).
      if (TrialsUntilFirstReward == 0) {
        TrialsUntilFirstReward = TrialInBlock;
        if (BlockNum >= 1 && BlockNum <= MAX_BLOCKS) {
          TrialsUntilFirstRewardHistory[BlockNum - 1] = TrialsUntilFirstReward;
        }
      }
      // Reward log line: emit once per actual reward delivered (was previously logged
      // on every correct outcome, even unrewarded ones - matches teensy2spouts now).
      serialOutBuffer += "Reward in block";
      serialOutBuffer += '\t';
      serialOutBuffer += BlockRewardCounter;
      serialOutBuffer += '\n';
    }
    if ((millis() - RewardStartTime) > RewardCurrentSize) {
      digitalWrite(RewardPin, LOW);
      digitalWrite(RewardIndicatorPin, LOW);
      NextState = CONSUMPTION;
      serialOutBuffer += "Total Rewards";
      serialOutBuffer += '\t';
      serialOutBuffer += Num_Reward;
      serialOutBuffer += '\n';
    }

    if (optoTriggeredConsumption && OptoActiveDuringConsumption) {
      optoTriggeredConsumption = false;
      startOptoStim();
      outputEvent("Opto_consumption", millis(), OptoDuration);
    }

    //   ----------------------------   CONSUMPTION   ----------------------------
  } else if (CurrentState == CONSUMPTION) {
    if (enteredNewState) {
      if (OptoActiveDuringCue) optoCueLick = true;

      // CorrectDuration for correct/CS+ trials, IncorrectDuration for incorrect/CS- trials
      ConsumptionDuration = correctSelection ? CorrectDuration : IncorrectDuration;
      ConsumptionStartTime = millis();

      if (optoTriggeredError && OptoActiveDuringError) {
        optoTriggeredError = false;
        startOptoStim();
        outputEvent("Opto_error", millis(), OptoDuration);
      }
      if (optoTriggeredAnyOutcome && OptoActiveDuringAnyOutcome) {
        optoTriggeredAnyOutcome = false;
        startOptoStim();
        outputEvent("Opto_anyOutcome", millis(), OptoDuration);
      }
    }

    if (millis() - ConsumptionStartTime >= ConsumptionDuration) {
      if (fails2reward > 0 && failCounter >= fails2reward) {
        NextState = FREE_REWARD;
        failCounter = 0;
        freeRewardCounter += 1;
        freeRewardBlockCounter += 1;
        serialOutBuffer += "Free Rewards";
        serialOutBuffer += '\t';
        serialOutBuffer += freeRewardCounter;
        serialOutBuffer += '\n';
      } else {
        NextState = NEW_TRIAL;
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
// ---------------------------- Functions: task settings -------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Fill block lengths
void fillBlockLengths() {
  numBlockLengthsA = 0;
  for (int i = minBlockA; i <= maxBlockA; i++) {
    BlockLengthsA[numBlockLengthsA++] = i;
  }
  numBlockLengthsB = 0;
  for (int i = minBlockB; i <= maxBlockB; i++) {
    BlockLengthsB[numBlockLengthsB++] = i;
  }
}

// generate ITI list
void fillITIDuration() {
  unsigned long currentValue = minITI;
  NumITIDurations = 0;
  for (int i = 0; i < 100; i++) {
    if (currentValue > maxITI) break;
    ITIDurationList[i] = currentValue;
    currentValue += ITIstep;
    NumITIDurations++;
  }
}

// Fill reward probability list
void fillRewardProbList() {
  for (int i = 0; i < RewardProb; i++) RewardProbList[i] = 1;
  for (int i = RewardProb; i < 100; i++) RewardProbList[i] = 0;
}

// Assign BlockType based on random number
void randomBlock() {
  int randomNum = random(0, 2);
  BlockType = (randomNum == 0) ? BLOCK_A : BLOCK_B;
}

// Fill reward sizes
void fillRewardRange() {
  int currentValue = RewardMin;
  numRewardRange = 0;
  for (int i = 0; i < 100; i++) {
    if (currentValue > RewardMax) break;
    RewardRange[i] = currentValue;
    currentValue += RewardStepSize;
    numRewardRange++;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------- Functions: EEPROM storage ------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Save parameters to EEPROM. Can only be done 100,000 times
void saveParams() {
  int address = 0;

  // Save all int parameters
  for (int i = 0; i < numIntParams; i++) {
    EEPROM.put(address, *intParams[i]);
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
    EEPROM.get(address, *intParams[i]);
    if (*intParams[i] == -1) {
      *intParams[i] = 1;
      EEPROM.put(address, *intParams[i]);
    }
    address += sizeof(int);
  }

  // Get all boolean parameters
  for (int i = 0; i < numBoolParams; i++) {
    uint8_t raw = EEPROM.read(address);
    if (raw == 0xFF) {
      *boolParams[i] = false;
      EEPROM.put(address, *boolParams[i]);
    } else {
      *boolParams[i] = raw != 0;
    }
    address += sizeof(bool);
  }

  // Get all unsigned long parameters
  for (int i = 0; i < numUlongParams; i++) {
    EEPROM.get(address, *ulongParams[i]);
    if (*ulongParams[i] == 4294967295UL) {
      *ulongParams[i] = 1;
      EEPROM.put(address, *ulongParams[i]);
    }
    address += sizeof(unsigned long);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// -------------------------- Functions: task helpers ----------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// turn off manual auditory cues because you cannot use "while" if combined with a reward delivery
void turnOffManualCue() {
  if ((millis() - CueStartTime) >= ToneDuration && (digitalRead(CueAIndicatorPin) == HIGH)) {
    analogWrite(SpeakerPinA, 0);
    analogWrite(SpeakerPinB, 0);
    digitalWrite(SpeakerTTLPin, LOW);
    digitalWrite(CueAIndicatorPin, LOW);
  }
  if ((millis() - CueStartTime) >= ToneDuration && (digitalRead(CueBIndicatorPin) == HIGH)) {
    analogWrite(SpeakerPinA, 0);
    analogWrite(SpeakerPinB, 0);
    digitalWrite(SpeakerTTLPin, LOW);
    digitalWrite(CueBIndicatorPin, LOW);
  }
}

// send trial start TTL
void sendTrialStartPulse() {
  if (sendTrialStartTTL && !trialStartPulseActive) {
    trialStartPulseActive = true;
    TrialStartTimer = millis();
    digitalWrite(TrialStartPin, HIGH);
    outputEvent("TrialStart_ON", millis(), TrialStartTTLDuration);
  }
  if (trialStartPulseActive) {
    if (millis() - TrialStartTimer >= TrialStartTTLDuration) {
      digitalWrite(TrialStartPin, LOW);
      trialStartPulseActive = false;
      sendTrialStartTTL = false;
      outputEvent("TrialStart_OFF", millis(), 0);
    }
  }
}

// Single-spout lick detection. Mirrors the teensy2spouts pattern: rising edge sets
// LickOccuring + LickStartTime, falling edge emits a "Lick" event with the start
// timestamp and duration. Runs unconditionally so licks remain visible in the GUI
// even when no task is running (useful for spout calibration / hardware checks).
//
// In addition to the 2-spout pattern, this also increments licksInSelection on each
// new lick that begins inside the SELECTION state - needed for the Go (GoLickThreshold)
// and Stop (StopLickThreshold) lick-count comparisons.
void lickDetection() {
  bool lickDetected;
  if (SpoutNotButton) {
    lickDetected = (digitalRead(LickPin) == LOW);
  } else {
    lickDetected = (digitalRead(LickPin) == HIGH);
  }

  if (lickDetected && !LickOccuring) {
    LickOccuring = true;
    LickStartTime = millis();
    if (CurrentState == SELECTION) licksInSelection += 1;
  } else if (!lickDetected && LickOccuring) {
    LickOccuring = false;
    unsigned long Lick_Duration = millis() - LickStartTime;
    outputEvent("Lick", LickStartTime, Lick_Duration);
    LickCounter += 1;
  }
}

// Schedule a "consumption" (correct trial reward) opto event
void handleCorrectOpto() {
  if (OptoActiveDuringConsumption && !optoBlockDone && !optoTrialDone) {
    if (numCorrectTrialsBeforeOpto == 0 || CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
      if (random(100) < OptoStimProb) {
        optoTrialDone = true;
        optoTriggeredConsumption = true;
        if (!AllowMultiOptoStimBlock) optoBlockDone = true;
      }
    }
  }
}

// Schedule an "error" opto event (incorrect lick / NoGo false alarm)
void handleErrorOpto() {
  if (OptoActiveDuringError && !optoBlockDone && !optoTrialDone) {
    if (numCorrectTrialsBeforeOpto == 0 || CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
      if (random(100) < OptoStimProb) {
        optoTrialDone = true;
        optoTriggeredError = true;
        if (!AllowMultiOptoStimBlock) optoBlockDone = true;
      }
    }
  }
}

// Schedule an "any outcome" opto event
void handleAnyOutcomeOpto() {
  if (OptoActiveDuringAnyOutcome && !optoBlockDone && !optoTrialDone) {
    if (numCorrectTrialsBeforeOpto == 0 || CorrectTrialCount >= numCorrectTrialsBeforeOpto) {
      if (random(100) < OptoStimProb) {
        optoTrialDone = true;
        optoTriggeredAnyOutcome = true;
        if (!AllowMultiOptoStimBlock) optoBlockDone = true;
      }
    }
  }
}

// Fire ErrorLightPin (used as no-reward signal). Idempotent within one ErrorLightDuration.
// Duration = 0 means disabled; any value > 0 enables the light for that many ms.
void fireErrorLight() {
  if (ErrorLightDuration > 0 && !errorLightActive) {
    digitalWrite(ErrorLightPin, HIGH);
    errorLightActive = true;
    errorLightStartTime = millis();
  }
}

// Diverging SSD staircase: correct stop -> shorter SSD (easier); failed stop -> longer SSD (harder).
void updateSSDStaircase(bool success) {
  if (!SSDStaircase) return;
  SSDCurrent += success ? -SSDstep : SSDstep;
  if (SSDCurrent < SSDmin) SSDCurrent = SSDmin;
  if (SSDCurrent > SSDmax) SSDCurrent = SSDmax;
}

// Fixed-grid SSD sampling from [SSDmin, SSDmax] in SSDstep increments.
int sampleSSDGrid() {
  if (SSDstep <= 0 || SSDmax <= SSDmin) return SSDmin;
  int n = (SSDmax - SSDmin) / SSDstep + 1;
  return SSDmin + SSDstep * (int)random(n);
}

// Increment trial count, set per-trial parameters, and switch blocks if needed.
void incrementTrial() {
  optoTrialDone = false;
  optoTriggeredConsumption = false;
  optoTriggeredError = false;
  optoTriggeredAnyOutcome = false;
  TrialNum += 1;
  TrialInBlock += 1;
  optoCueLick = false;

  // Per-trial reward size (single valve)
  int rewardIdx = random(numRewardRange);
  RewardCurrentSize = RewardRange[rewardIdx];

  // Block switching FIRST so the per-trial reward decision below sees the correct BlockType.
  //
  //   BlockSwitchAfterCorrectTrials == false (default):
  //     Switch when TrialInBlock > BlockLength (legacy behavior - count all trials).
  //
  //   BlockSwitchAfterCorrectTrials == true:
  //     Switch when CorrectTrialCount >= BlockLength (count #_CORRECT trials only).
  //     The block extends through any number of #_INCORRECT / #_TIMEOUT trials and ends
  //     only when BlockLength correct trials have accumulated. EXCEPTION: Pavl CS-
  //     (BLOCK_B) emits #_CORRECT but does NOT increment CorrectTrialCount (stays 0), so we
  //     fall back to trial count there or the block would never end.
  if (TrialNum == 1) {
    switchBlock();  // initialize first block (always runs; for taskRatio this locks in BLOCK_A)
  } else if (BlockLength > 0 && !taskRatio) {
    // Ratio Schedule has no block switching - animal stays in BLOCK_A for the whole session
    bool useCorrectCount = BlockSwitchAfterCorrectTrials && !(taskPavlCond && BlockType == BLOCK_B);
    bool shouldSwitch = useCorrectCount
                          ? (CorrectTrialCount >= BlockLength)
                          : (TrialInBlock > BlockLength);
    if (shouldSwitch) switchBlock();
  }

  // Per-trial reward decision (probabilistic). Made ONCE here at trial start - no other
  // state may overwrite giveReward/giveRewardIncorrect during the trial. The "Reward
  // prob trial" log line below tells the GUI what was decided so it can render the
  // outcome dot correctly (rewarded vs probabilistic-no-reward) without re-guessing.
  //
  //   default:                                giveReward = 1 with prob RewardProb%
  //   Pavl  BLOCK_B (CS-):                    giveReward = 0 always (CS- never rewards)

  // --- SSRT stop-signal delay (taskStop only); SSDCurrent = SELECTION onset -> stop cue ---
  if (taskStop) {
    if (SSDStaircase) {
      if (TrialNum == 1) SSDCurrent = (SSDmin + SSDmax) / 2;  // staircase starts in the middle or range
      // else SSDCurrent persists; nudged in updateSSDStaircase() at each stop outcome
    } else {
      SSDCurrent = sampleSSDGrid();  // fixed sampling from [SSDmin..SSDmax]
    }
  }

  int randomRewardIdx = random(100);
  int RewardMaybe = RewardProbList[randomRewardIdx];
  if (taskPavlCond && BlockType == BLOCK_B) {
    RewardMaybe = 0;  // CS- never rewards
  }
  if (RewardMaybe == 1) {
    giveReward = true;
    giveRewardIncorrect = false;
  } else {
    giveReward = false;
    giveRewardIncorrect = false;  // 1-spout: incorrect choices never get reward
  }

  // Random Reward override (Pavlovian only) — must come after the normal reward-probability
  // block above so giveReward is always set to a consistent final value.
  // isRandomRewardTrial suppresses the cue in the CUE state and emits #_RANDOM_REWARD.
  isRandomRewardTrial = false;
  if (taskPavlCond && PavlRandomRewardProb > 0) {
    if (random(100) < (uint32_t)PavlRandomRewardProb) {
      isRandomRewardTrial = true;
      giveReward = true;
      giveRewardIncorrect = false;
    }
  }

  // Ratio Schedule: compute lick threshold for this trial
  // Fixed Ratio: constant FRNumberLicks each trial
  // Progressive Ratio: Richardson & Roberts (1996) exponential escalation, k=0.2
  //   PRCurrentLicks = round(PRNumberStartLicks * exp(0.2 * (PRTrialNum - 1)))
  //   gives: trial 1 = PRNumberStartLicks, then ~22% more each trial
  if (taskRatio) {
    PRTrialNum++;
    if (ratioFixed) {
      PRCurrentLicks = FRNumberLicks;
    } else {
      PRCurrentLicks = (int)round(PRNumberStartLicks * exp(0.2 * (PRTrialNum - 1)));
      outputEvent("PR_licks_required", millis(), PRCurrentLicks);
    }
  }

  // Logging
  serialOutBuffer += "Reward prob trial";
  serialOutBuffer += '\t';
  serialOutBuffer += RewardMaybe;
  serialOutBuffer += '\n';
  serialOutBuffer += "Reward size trial";
  serialOutBuffer += '\t';
  serialOutBuffer += RewardCurrentSize;
  serialOutBuffer += '\n';
}

// Actually switch blocks
void switchBlock() {
  // Print stats for previous block (skipped on the very first switch)
  if (TrialNum > 1) {
    printEndOfBlockStats();
    optoBlockFirstTrial = true;
  }

  // reset per-block counters
  BlockNum += 1;
  TrialInBlock = 1;
  CorrectTrialCount = 0;
  incorrectCounter = 0;
  Num_ENLPenalty = 0;
  Num_Omissions = 0;
  freeRewardBlockCounter = 0;
  BlockRewardCounter = 0;
  BlockNoRewardCounter = 0;
  LickCounter = 0;
  TrialsUntilFirstReward = 0;
  if (BlockNum >= 1 && BlockNum <= MAX_BLOCKS) {
    TrialsUntilFirstRewardHistory[BlockNum - 1] = 0;
  }
  badTrialsCounter = 0;
  optoBlockDone = false;

  // pick new BlockType and block length
  // Ratio Schedule has no block structure - stays BLOCK_A, BlockLength = 0
  if (taskRatio) {
    BlockLength = 0;
    // BlockType intentionally NOT flipped - always BLOCK_A for ratio task
  } else {
    BlockType = (BlockType == BLOCK_A) ? BLOCK_B : BLOCK_A;
    if (BlockType == BLOCK_A) {
      int idx = random(numBlockLengthsA);
      BlockLength = BlockLengthsA[idx];
    } else {
      int idx = random(numBlockLengthsB);
      BlockLength = BlockLengthsB[idx];
    }
  }

  serialOutBuffer += "Out of";
  serialOutBuffer += '\t';
  serialOutBuffer += BlockLength;
  serialOutBuffer += '\n';
}

// Generate sync pulses (100 ms pulses with random inter-pulse-interval)
void updateSyncPulses() {
  if (sessionStart) {
    if (millis() - TimerSync > SyncPulseInterval) {
      TimerSync = millis();
      if (SyncPulseStatus == 1) {
        digitalWrite(SyncPin, LOW);
        SyncPulseStatus = 0;
        SyncPulseInterval = 100 + 10 * random(1, 50);  // random sync pulse interval between 110~590 ms in steps of 10 ms = 49 possibilities
        serialOutBuffer += "Sync_off";
        serialOutBuffer += '\t';
        serialOutBuffer += millis();
        serialOutBuffer += '\t';
        serialOutBuffer += SyncPulseInterval;
        serialOutBuffer += '\n';
      } else {
        digitalWrite(SyncPin, HIGH);
        SyncPulseStatus = 1;
        SyncPulseInterval = 100;
        serialOutBuffer += "Sync_on";
        serialOutBuffer += '\t';
        serialOutBuffer += millis();
        serialOutBuffer += '\t';
        serialOutBuffer += SyncPulseInterval;
        serialOutBuffer += '\n';
      }
    }
  } else if (sessionEnd) {
    digitalWrite(SyncPin, LOW);
    serialOutBuffer += "Sync_off";
    serialOutBuffer += '\t';
    serialOutBuffer += millis();
    serialOutBuffer += '\t';
    serialOutBuffer += SyncPulseInterval;
    serialOutBuffer += '\n';
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
  if (sessionStart) {
    int currentRead = digitalRead(ExtSyncPin);
    if (currentRead == HIGH && ExtSyncState == 0) {
      ExtSyncState = 1;
      ExtSyncRiseTime = millis();
      unsigned long ExtSyncITI;
      if (firstExtSync) {
        ExtSyncITI = ExtSyncRiseTime - SessionStartTime;
        firstExtSync = false;
      } else {
        ExtSyncITI = ExtSyncRiseTime - ExtSyncLastFallTime;
      }
      serialOutBuffer += "ExtSync_on";
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncRiseTime;
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncITI;
      serialOutBuffer += '\n';
      digitalWrite(ExtSyncReportPin, HIGH);
    } else if (currentRead == LOW && ExtSyncState == 1) {
      ExtSyncState = 0;
      unsigned long ExtSyncFallTime = millis();
      unsigned long ExtSyncDuration = ExtSyncFallTime - ExtSyncRiseTime;
      serialOutBuffer += "ExtSync_off";
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncFallTime;
      serialOutBuffer += '\t';
      serialOutBuffer += ExtSyncDuration;
      serialOutBuffer += '\n';
      ExtSyncLastFallTime = ExtSyncFallTime;
      digitalWrite(ExtSyncReportPin, LOW);
    }
  } else {
    ExtSyncState = digitalRead(ExtSyncPin);
    digitalWrite(ExtSyncReportPin, LOW);
  }
}

// trigger camera
void updateCameraTrigger() {
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
// ----------------------- Functions: printing / serial out ---------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// update state name
void outputTrialState() {
  if (CurrentState == PRE_REWARD) return;
  const char* stateName = "";
  unsigned long duration = 0;
  if (CurrentState == IDLE) stateName = "IDLE";
  else if (CurrentState == NEW_TRIAL) stateName = "NEW_TRIAL";
  else if (CurrentState == ITI) {
    if (isRandomRewardTrial) stateName = "ITI_RR";
    else stateName = (BlockType == BLOCK_B) ? "ITI_B" : "ITI_A";
    duration = ITIDuration;
  } else if (CurrentState == ENL_PENALTY) {
    stateName = "ENL_PENALTY";
    duration = ENLPenaltyDuration;
  } else if (CurrentState == CUE_PENALTY) {
    stateName = "CUE_PENALTY";
    duration = CuePenaltyDuration;
  } else if (CurrentState == CUE) {
    if (isRandomRewardTrial) stateName = "CUE_RR";
    else if (taskStop) stateName = "CUE_A";  // stop task: block B also opens with the go cue
    else stateName = (BlockType == BLOCK_B) ? "CUE_B" : "CUE_A";
    duration = ToneDuration;
  } else if (CurrentState == SELECTION) {
    stateName = "SELECTION";
    if (taskGoNogo && BlockType == BLOCK_B) duration = NogoHoldDuration;
    else duration = ReactionTime;
  } else if (CurrentState == REWARD) {
    stateName = "REWARD";
    duration = RewardCurrentSize;
  } else if (CurrentState == NO_REWARD) {
    stateName = "NO_REWARD";
    duration = 0;
  } else if (CurrentState == FREE_REWARD) {
    stateName = "FREE_REWARD";
    duration = RewardCurrentSize;
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
  serialOutBuffer += TrialNum;
  serialOutBuffer += '\t';
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
  serialOutBuffer += '\n';
}

// Helper - emit a Block Stats block (used by both end-of-block and end-of-session)
void emitBlockStats() {
  serialOutBuffer += "\nBlock Stats\t";
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\n";
  serialOutBuffer += "Block Num\t";
  serialOutBuffer += BlockNum;
  serialOutBuffer += "\t";
  serialOutBuffer += "Block\t";
  serialOutBuffer += (BlockType == BLOCK_B) ? "B\t" : "A\t";
  serialOutBuffer += "Length\t";
  serialOutBuffer += BlockLength;
  serialOutBuffer += '\t';
  serialOutBuffer += "Trials\t";
  serialOutBuffer += (TrialInBlock - 1);
  serialOutBuffer += '\t';
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
  serialOutBuffer += "Licks\t";
  serialOutBuffer += LickCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Bad Trials Post Reward\t";
  serialOutBuffer += badTrialsCounter;
  serialOutBuffer += "\t";
  serialOutBuffer += "Trials To 1st Reward\t";
  serialOutBuffer += TrialsUntilFirstReward;
  serialOutBuffer += "\n\n";
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
// ------------------- Functions: optogenetic stimulation ------------------------------------------ //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// initiate a new opto stimulation
void startOptoStim() {
  if (OptoEnabled) {
    optoStartTime = millis();
    duringOptoStim = true;
    duringOptoStimDelay = true;
    outputEvent("Opto_START", millis(), OptoDuration);
  }
}

// This should be called on a regular basis by loop().
// Turns opto on/off as specified by the delay and duration paramters.
void updateOptoStim() {
  if (duringOptoStim) {
    timeInStim = millis() - optoStartTime;
    if (duringOptoStimDelay && (timeInStim > OptoDelay)) {
      digitalWrite(OptoPin, HIGH);
      digitalWrite(OptoTTLPin, HIGH);
      duringOptoStimDelay = false;
      optoStartPulse = millis();
      optoPulseActive = true;
      outputEvent("Opto_ON", millis(), OptoDuration);
    } else if ((timeInStim > (OptoDelay + OptoDuration)) || optoCueLick == true) {
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
      duringOptoStim = false;
      optoPulseActive = false;
      optoIPIActive = false;
      optoTaperActive = false;
      outputEvent("Opto_OFF", millis(), OptoDuration);
    } else if (optoContinuous == false && optoPulseActive == true) {
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > OptoPulseDuration) {
        digitalWrite(OptoPin, LOW);
        digitalWrite(OptoTTLPin, LOW);
        optoPulseActive = false;
        optoIPIActive = true;
        optoStartPulse = millis();
      }
    } else if (optoContinuous == false && optoIPIActive == true) {
      optoIPI = (1000 / OptoFrequency) - OptoPulseDuration;
      optoTimeInPulse = millis() - optoStartPulse;
      if (optoTimeInPulse > optoIPI) {
        digitalWrite(OptoPin, HIGH);
        digitalWrite(OptoTTLPin, HIGH);
        optoIPIActive = false;
        optoPulseActive = true;
        optoStartPulse = millis();
      }
    }
  }

  // AOM / driver modulation
  if (manualOptoTestActive) {
    // manual test: bt handler manages OptoModPin
  } else if (OptoEnabled) {
    if (OptoModContinuous) {
      if (!duringOptoStim) {
        analogWrite(OptoModPin, OptoMod * 2.56);
      } else {
        if (duringOptoStim && !optoTaperActive && ((long)timeInStim > (long)(OptoDelay + OptoDuration - OptoTaperOff))) {
          optoTaperStartTime = millis();
          optoTaperActive = true;
        } else if (optoTaperActive) {
          timeInTaper = millis() - optoTaperStartTime;
          tmpOptoMod = round((1 - ((float)timeInTaper / OptoTaperOff)) * OptoMod);
          analogWrite(OptoModPin, tmpOptoMod * 2.56);
        }
      }
    } else {
      if (duringOptoStim) {
        if (optoTaperActive) {
          analogWrite(OptoModPin, tmpOptoMod * 2.56);
        } else {
          analogWrite(OptoModPin, OptoMod * 2.56);
        }
      } else {
        analogWrite(OptoModPin, 0);
      }
    }
  } else {
    analogWrite(OptoModPin, 0);
  }
}

// initiate random opto distraction light
void randomLightFunction() {
  if (ApplyOptoRandomLight && OptoEnabled && sessionStart) {
    if (millis() - RandomLightTimer >= RandomLightInterval) {
      if (RandomLightON == 1) {
        RandomLightTimer = millis();
        digitalWrite(RandomLightPin, LOW);
        RandomLightON = 0;
        RandomLightInterval = 100 + random(1, 900);
      } else {
        RandomLightTimer = millis();
        digitalWrite(RandomLightPin, HIGH);
        RandomLightON = 1;
        RandomLightInterval = 500;
      }
    }
  } else if (sessionEnd == false) {
    digitalWrite(RandomLightPin, LOW);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------- Functions: serial communication ------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Higher MAX_CHARS_PER_LOOP + faster baud (115200) drains the serial buffer much faster,
// which is what removes the visible lag in the MATLAB GUI ("Event:" label / new trial dot).
// Keep this <= ~256 so a single loop still completes well under 1 ms in the worst case.
const unsigned int MAX_CHARS_PER_LOOP = 128;  // 24
char tempBuffer[MAX_CHARS_PER_LOOP + 10];
void updateSerialOutput() {
  if (pausePrint) {
    pausePrint = false;
    return;
  }
  if (serialOutBuffer.length() > 0) {
    unsigned int numCharToPrint = min(MAX_CHARS_PER_LOOP, serialOutBuffer.length());
    serialOutBuffer.toCharArray(tempBuffer, numCharToPrint + 1);
    Serial.print(tempBuffer);
    serialOutBuffer.remove(0, numCharToPrint);
  }
}

// manually trigger events (can go through serial command & matlab)
void checkSerialInput() {
  if (Serial.available() > 0) {
    SerialInput = Serial.read();

    // Manual reward (single spout) = 1
    if (SerialInput == '1' && RewardButtonStatus == 0) {
      CueStartTime = millis();
      RewardButtonStatus = 1;
      digitalWrite(RewardPin, HIGH);
      digitalWrite(RewardIndicatorPin, HIGH);
      RewardTimer = millis();
      outputEvent("MANUAL_REWARD", CueStartTime, RewardCurrentSize);
    }

    // Task start = 8
    else if ((SerialInput == '8') || (buttonsAreWiredUp && (digitalRead(StartStopButtonPin) == HIGH))) {
      sessionStart = true;
      TimerSync = millis();
      digitalWrite(CameraTriggerPin, LOW);
      CameraTrigStartTime = millis();
      digitalWrite(StartStopIndicatorPin, HIGH);

      SessionStartTime = millis();
      StartDelayTimer = millis();
      ExtSyncLastFallTime = 0;
      ExtSyncRiseTime = SessionStartTime;
      firstExtSync = true;
      ExtSyncState = digitalRead(ExtSyncPin);

      serialOutBuffer += "TASK STARTED AT";
      serialOutBuffer += '\t';
      serialOutBuffer += millis();
      serialOutBuffer += '\n';

      // first block direction - all 3 task modes honor trialStart now (Stop hybrid uses A and B)
      if (trialStart == 0) {
        randomBlock();
        // pre-emptive flip so switchBlock lands on the picked side
        BlockType = (BlockType == BLOCK_A) ? BLOCK_B : BLOCK_A;
      } else if (trialStart == 1) {
        BlockType = BLOCK_B;  // will flip to A
      } else if (trialStart == 2) {
        BlockType = BLOCK_A;  // will flip to B
      }

      fillBlockLengths();
      fillITIDuration();
      fillRewardRange();
      fillRewardProbList();
      NextState = START_DELAY;
    }

    // Task stop = 9
    else if (SerialInput == '9' && sessionStart == true) {
      sessionStart = false;
      sessionEnd = true;
      digitalWrite(StartStopIndicatorPin, HIGH);
      NextState = IDLE;
      serialOutBuffer += "TASK ENDED AT";
      serialOutBuffer += '\t';
      serialOutBuffer += millis();
      serialOutBuffer += '\n';
      if (TrialInBlock > 1) printEndOfSessionStats();
    }

    // Multi-character GUI command
    else {
      static String usbMessage = "";
      usbMessage += SerialInput;
      while (Serial.available() > 0) {
        char inByte = Serial.read();
        if ((inByte == '\n') || (inByte == ';')) {
          interpretUSBMessage(usbMessage);
          usbMessage = "";
        } else {
          usbMessage += inByte;
        }
      }
    }
  }

  // turn off manual reward valve after RewardCurrentSize ms
  if ((millis() - RewardTimer) > RewardCurrentSize && RewardButtonStatus == 1) {
    digitalWrite(RewardPin, LOW);
    digitalWrite(RewardIndicatorPin, LOW);
    RewardButtonStatus = 0;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------- USB/GUI command parser --------------------------------------------------------- //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// allow Matlab to define task variables through our GUI
void interpretUSBMessage(String message) {
  message.trim();
  int len = message.length();
  if (len == 0) {
    Serial.println("#");
    return;
  }
  String command = message.substring(0, 2);
  String parameters = message.substring(2);
  parameters.trim();

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

  if (command == "aa") {  // min trials per block (both A and B)
    minBlockA = arg1;
    minBlockB = arg1;
    minBlock = arg1;
    fillBlockLengths();
    serialOutBuffer += "Min Block A\t";
    serialOutBuffer += minBlockA;
    serialOutBuffer += '\n';
    serialOutBuffer += "Min Block B\t";
    serialOutBuffer += minBlockB;
    serialOutBuffer += '\n';
  } else if (command == "ab") {  // max trials per block (both A and B)
    maxBlockA = arg1;
    maxBlockB = arg1;
    maxBlock = arg1;
    fillBlockLengths();
    serialOutBuffer += "Max Block A\t";
    serialOutBuffer += maxBlockA;
    serialOutBuffer += '\n';
    serialOutBuffer += "Max Block B\t";
    serialOutBuffer += maxBlockB;
    serialOutBuffer += '\n';
  } else if (command == "am") {
    minBlockA = arg1;
    fillBlockLengths();
  }  // min block A
  else if (command == "an") {
    maxBlockA = arg1;
    fillBlockLengths();
  }  // max block A
  else if (command == "ap") {
    minBlockB = arg1;
    fillBlockLengths();
  }  // min block B
  else if (command == "aq") {
    maxBlockB = arg1;
    fillBlockLengths();
  }                                                   // max block B
  else if (command == "ac") { fails2reward = arg1; }  // fails until free reward
  else if (command == "ad") {
    ReactionTime = arg1;
  }                            // selection / lick window
  else if (command == "ae") {  // manual cue test
    if (arg1 == 0) {           // cue A
      CueStartTime = millis();
      if (cuesInstructed) {
        analogWriteFrequency(SpeakerPinA, CueAFreq);
        analogWrite(SpeakerPinA, 128);
        analogWrite(SpeakerPinB, 0);
      } else {
        analogWriteFrequency(SpeakerPinA, CueAFreq);
        analogWrite(SpeakerPinA, 128);
        analogWriteFrequency(SpeakerPinB, CueAFreq);
        analogWrite(SpeakerPinB, 128);
      }
      digitalWrite(SpeakerTTLPin, HIGH);
      digitalWrite(CueAIndicatorPin, HIGH);
      outputEvent("MANUAL_CUE_A", CueStartTime, ToneDuration);
    } else if (arg1 == 1) {  // cue B
      CueStartTime = millis();
      if (cuesInstructed) {
        analogWrite(SpeakerPinA, 0);
        analogWriteFrequency(SpeakerPinB, CueBFreq);
        analogWrite(SpeakerPinB, 128);
      } else {
        analogWriteFrequency(SpeakerPinA, CueBFreq);
        analogWrite(SpeakerPinA, 128);
        analogWriteFrequency(SpeakerPinB, CueBFreq);
        analogWrite(SpeakerPinB, 128);
      }
      digitalWrite(SpeakerTTLPin, HIGH);
      digitalWrite(CueBIndicatorPin, HIGH);
      outputEvent("MANUAL_CUE_B", CueStartTime, ToneDuration);
    }
  } else if (command == "ag") {  // single reward calibration (sets min=max=size)
    RewardMin = arg1;
    RewardMax = arg1;
    RewardStepSize = 1;
    RewardCurrentSize = arg1;
    fillRewardRange();
    serialOutBuffer += "Min Reward\t";
    serialOutBuffer += RewardMin;
    serialOutBuffer += '\n';
    serialOutBuffer += "Max Reward\t";
    serialOutBuffer += RewardMax;
    serialOutBuffer += '\n';
    serialOutBuffer += "Reward Step\t";
    serialOutBuffer += RewardStepSize;
    serialOutBuffer += '\n';
  } else if (command == "ai") {
    minITI = arg1;
    fillITIDuration();
  } else if (command == "aj") {
    maxITI = arg1;
    fillITIDuration();
  } else if (command == "ak") {
    ITIstep = arg1;
    fillITIDuration();
  } else if (command == "al") {
    OptoEnabled = (arg1 == 1);
  } else if (command == "ao") {  // first block direction
    if (arg1 == 0) trialStart = 0;
    else if (arg1 == 1) trialStart = 1;
    else if (arg1 == 2) trialStart = 2;
  } else if (command == "ar") {
    RewardMin = arg1;
    fillRewardRange();
  }  // min reward size
  else if (command == "as") {
    RewardMax = arg1;
    fillRewardRange();
  }  // max reward size
  else if (command == "at") {
    RewardStepSize = arg1;
    fillRewardRange();
  }  // reward step
  else if (command == "ax") {
    RewardProb = arg1;
    fillRewardProbList();
  } else if (command == "ba") {
    optoContinuous = (arg1 == 1);
  } else if (command == "bb") {
    OptoStimProb = arg1;
  } else if (command == "bc") {
    numCorrectTrialsBeforeOpto = arg1;
  } else if (command == "bd") {
    OptoDelay = arg1;
  } else if (command == "be") {
    OptoDuration = arg1;
  } else if (command == "bf") {
    OptoPulseDuration = arg1;
  } else if (command == "bg") {
    OptoFrequency = arg1;
  } else if (command == "bh") {
    OptoActiveDuringBlockSwitch = (arg1 == 1);
  } else if (command == "bi") {
    OptoActiveDuringITI = (arg1 == 1);
  } else if (command == "bj") {
    OptoActiveDuringCue = (arg1 == 1);
  } else if (command == "bk") {
    OptoActiveDuringConsumption = (arg1 == 1);
  } else if (command == "af") {
    OptoMod = arg1;
    if (manualOptoTestActive) analogWrite(OptoModPin, OptoMod * 2.56);
  } else if (command == "by") {
    OptoTaperOff = arg1;
  } else if (command == "bl") {
    AllowMultiOptoStimITI = (arg1 == 1);
  } else if (command == "bx") {
    AllowMultiOptoStimCue = (arg1 == 1);
  } else if (command == "bm") {
    AllowMultiOptoStimBlock = (arg1 == 1);
  } else if (command == "bn") {
    CuePenaltyDuration = arg1;
  } else if (command == "bo") {
    CueAFreq = arg1;
  }                                               // cue A frequency
  else if (command == "bp") { CueBFreq = arg1; }  // cue B frequency
  else if (command == "bq") {
    CorrectDuration = arg1;
  }                                                        // consumption period
  else if (command == "bz") { IncorrectDuration = arg1; }  // no-reward period
  else if (command == "br") {
    ToneDuration = arg1;
  } else if (command == "bs") {
    ApplyOptoRandomLight = (arg1 == 1);
  } else if (command == "bt") {  // manual opto on/off for testing
    if (arg1 == 1) {
      manualOptoTestActive = true;
      if (!OptoModContinuous) analogWrite(OptoModPin, OptoMod * 2.56);
      digitalWrite(OptoPin, HIGH);
      digitalWrite(OptoTTLPin, HIGH);
    } else {
      manualOptoTestActive = false;
      if (!OptoModContinuous) analogWrite(OptoModPin, 0);
      digitalWrite(OptoPin, LOW);
      digitalWrite(OptoTTLPin, LOW);
    }
  } else if (command == "bv") {
    maxOmissions = arg1;
  } else if (command == "bw") {
    maxTimeout = arg1;
  } else if (command == "cc") {
    maxTrials = arg1;
  } else if (command == "cd") {
    maxRewards = arg1;
  } else if (command == "cg") {
    ENLEnabled = (arg1 == 1);
  } else if (command == "cf") {
    RewardDelay = arg1;
  } else if (command == "ca") {
    cuesInstructed = (arg1 == 1);
  } else if (command == "cb") {
    OptoModContinuous = (arg1 == 1);
  } else if (command == "ch") {
    OptoActiveDuringError = (arg1 == 1);
  } else if (command == "ci") {
    OptoActiveDuringAnyOutcome = (arg1 == 1);
  } else if (command == "ck") {
    ErrorLightDuration = arg1;
  } else if (command == "cl") {
    TaskStartDelay = arg1;
  } else if (command == "cm") {
    ENLPenaltyDuration = arg1;
  }  // ENL penalty duration (ms)
  // === task mode (mutually exclusive) === //
  else if (command == "cu") {  // taskGoNogo
    if (arg1 == 1) {
      taskGoNogo = true;
      taskStop = false;
      taskPavlCond = false;
      taskRatio = false;
    } else if (taskStop || taskPavlCond || taskRatio) {
      taskGoNogo = false;
    }
  } else if (command == "cn") {  // taskStop
    if (arg1 == 1) {
      taskStop = true;
      taskGoNogo = false;
      taskPavlCond = false;
      taskRatio = false;
    } else if (taskGoNogo || taskPavlCond || taskRatio) {
      taskStop = false;
    }
  } else if (command == "co") {  // taskPavlCond
    if (arg1 == 1) {
      taskPavlCond = true;
      taskGoNogo = false;
      taskStop = false;
      taskRatio = false;
    } else if (taskGoNogo || taskStop || taskRatio) {
      taskPavlCond = false;
    }
  } else if (command == "cv") {  // taskRatio
    if (arg1 == 1) {
      taskRatio = true;
      taskGoNogo = false;
      taskStop = false;
      taskPavlCond = false;
    } else if (taskGoNogo || taskStop || taskPavlCond) {
      taskRatio = false;
    }
  } else if (command == "cw") {
    ratioFixed = (arg1 == 1);
  }                                                    // 1 = Fixed Ratio, 0 = Progressive Ratio
  else if (command == "cx") { FRNumberLicks = arg1; }  // # licks per trial in Fixed Ratio mode
  else if (command == "cy") {
    PRNumberStartLicks = arg1;
  }  // # licks on trial 1 in Progressive Ratio mode
  // === new task-specific parameters === //
  else if (command == "cp") { GoLickThreshold = arg1; }  // NEW - # licks for Go-hit
  else if (command == "cq") {
    NogoHoldDuration = arg1;
  }                                                        // NEW - NoGo lick-withhold window
  else if (command == "cr") { StopLickThreshold = arg1; }  // NEW - # licks for Stop-hit
  else if (command == "cs") {
    BlockSwitchAfterCorrectTrials = (arg1 == 1);
  }                                                           // NEW - end block on N correct trials (true) vs N total trials (false)
  else if (command == "ct") { PavlRandomRewardProb = arg1; }  // NEW - % of Pavlovian trials that are silent random rewards (0 = off)

  else if (command == "da") {
    SSDStaircase = (arg1 == 1);
  }                                             // SSRT staircase on/off
  else if (command == "db") { SSDmin = arg1; }  // SSRT min SSD (ms)
  else if (command == "dc") {
    SSDmax = arg1;
  }                                              // SSRT max SSD (ms)
  else if (command == "dd") { SSDstep = arg1; }  // SSRT SSD step (ms)

  else if (command == "ay") {
    saveParams();
  } else if (command == "az") {  // dump current parameters
    Serial.print("Task\t");
    Serial.println("oneSpout task");  // identity: lets MATLAB verify correct script
    Serial.print("Max Omissions\t");
    Serial.println(maxOmissions);
    Serial.print("Max Timeout\t");
    Serial.println(maxTimeout);
    Serial.print("Max Trials\t");
    Serial.println(maxTrials);
    Serial.print("Max Rewards\t");
    Serial.println(maxRewards);
    Serial.print("Task GoNogo\t");
    Serial.println(taskGoNogo);
    Serial.print("Task Stop\t");
    Serial.println(taskStop);
    Serial.print("Task Ratio\t");
    Serial.println(taskRatio);
    Serial.print("Task PavlCond\t");
    Serial.println(taskPavlCond);
    Serial.print("Ratio Fixed\t");
    Serial.println(ratioFixed);
    Serial.print("FR Number Licks\t");
    Serial.println(FRNumberLicks);
    Serial.print("PR Number Start Licks\t");
    Serial.println(PRNumberStartLicks);
    Serial.print("Min Block\t");
    Serial.println(minBlock);
    Serial.print("Max Block\t");
    Serial.println(maxBlock);
    Serial.print("Min Block A\t");
    Serial.println(minBlockA);
    Serial.print("Max Block A\t");
    Serial.println(maxBlockA);
    Serial.print("Min Block B\t");
    Serial.println(minBlockB);
    Serial.print("Max Block B\t");
    Serial.println(maxBlockB);
    Serial.print("Reaction Time\t");
    Serial.println(ReactionTime);
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
    Serial.print("Reward Calibration\t");
    Serial.println(RewardMax);
    Serial.print("Min ITI\t");
    Serial.println(minITI);
    Serial.print("Max ITI\t");
    Serial.println(maxITI);
    Serial.print("ITI Step\t");
    Serial.println(ITIstep);
    Serial.print("Min Reward\t");
    Serial.println(RewardMin);
    Serial.print("Max Reward\t");
    Serial.println(RewardMax);
    Serial.print("Reward Step\t");
    Serial.println(RewardStepSize);
    Serial.print("Reward Probability\t");
    Serial.println(RewardProb);
    Serial.print("Reward Delay\t");
    Serial.println(RewardDelay);
    Serial.print("Trial Start\t");
    Serial.println(trialStart);
    Serial.print("Cue A Frequency\t");
    Serial.println(CueAFreq);
    Serial.print("Cue B Frequency\t");
    Serial.println(CueBFreq);
    Serial.print("Go Lick Threshold\t");
    Serial.println(GoLickThreshold);
    Serial.print("Nogo Hold Duration\t");
    Serial.println(NogoHoldDuration);
    Serial.print("Stop Lick Threshold\t");
    Serial.println(StopLickThreshold);
    Serial.print("SSD Staircase\t");
    Serial.println(SSDStaircase);
    Serial.print("SSD Min\t");
    Serial.println(SSDmin);
    Serial.print("SSD Max\t");
    Serial.println(SSDmax);
    Serial.print("SSD Step\t");
    Serial.println(SSDstep);
    Serial.print("Block Switch After Correct Trials\t");
    Serial.println(BlockSwitchAfterCorrectTrials);
    Serial.print("Pavlovian Random Reward Prob\t");
    Serial.println(PavlRandomRewardProb);
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
    Serial.print("ENL Enabled\t");
    Serial.println(ENLEnabled);
    Serial.print("ENL Penalty Duration\t");
    Serial.println(ENLPenaltyDuration);
    Serial.print("Task Start Delay\t");
    Serial.println(TaskStartDelay);
    Serial.print("EEPROM settings\t");
    Serial.println(1);
  }
}
