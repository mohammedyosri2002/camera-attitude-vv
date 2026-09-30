// ============================================================
// motor_pitch_roll_FINAL.ino
// ------------------------------------------------------------
// FINAL PRODUCTION SKETCH -- IEEE Aerospace pitch/roll campaign.
// UIM5756PM / UIM344 integrated closed-loop NEMA23.
//
// Wiring (UNCHANGED from the yaw campaign):
//   DIR -> D8   PLS -> D9   ENA -> D7   COM -> GND
//   1/128 microstepping, 25600 pulses / revolution
//
// ONE sketch for AXIS_A and AXIS_B. Nothing is edited or
// recompiled between them. The run identity arrives over serial:
//
//     START,<runId 1..20>,<AXIS_A|AXIS_B>
//
// ------------------------------------------------------------
// WHAT IS NEW vs motor_yaw_FINAL.ino
// ------------------------------------------------------------
//  1. SIGNED bidirectional positioning inside one run. The yaw
//     sketch only ever moved in one direction and used
//     "if (delta > 0)". Here moveToPulses() sets DIR from the
//     sign of the delta on every move, so the profile crosses
//     zero and reverses freely.
//  2. Static profile is symmetric about zero and revisits each
//     magnitude from BOTH approach directions, so hysteresis is
//     observable.
//  3. Rate phase runs signed legs between -10 and +10 deg.
//  4. Return to zero is a COMMAND-BASED return. It is not
//     sensor-verified homing and is never called homing.
//  5. Axis name is carried in the metadata and in RUN_BEGIN.
//
// ------------------------------------------------------------
// PROFILE (verified: 483.998 s from START to the end of the
// final baseline, 114 "#S," events)
// ------------------------------------------------------------
//   10 s countdown
//   60 s stationary start baseline
//   STATIC: 17 targets, 10 s hold each, 5 deg/s transits
//           0 +5 +10 +15 +20 +15 +10 +5 0 -5 -10 -15 -20 -15 -10 -5 0
//           16 moves, 5688 pulses, 79.9875 deg of travel, 15.997 s
//           2 DIR reversals (at +20 and at -20)
//   20 s intermission
//   RATE:   position 0 -> -10 deg, then for each rate magnitude
//           RATE_CYCLES full back-and-forth cycles between
//           -10 and +10 deg. Each cycle = 2 legs.
//   command-based return to zero
//   60 s stationary end baseline
//
// EVENT LOG FORMAT (parsed by the logger):
//     #S,<EVENT>,<micros>,<signed_value>[,<signed_extra>]
// Signed physical command values everywhere; internal pulse
// counters are signed positions relative to the established zero.
// ============================================================

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ------------------------------------------------------------
// PINS / DRIVER
// ------------------------------------------------------------
const byte DIR_PIN  = 8;
const byte STEP_PIN = 9;
const byte EN_PIN   = 7;

const bool MOTOR_ENABLE  = HIGH;   // verified on this hardware
const bool DIR_LEVEL_POS = HIGH;   // DIR level that increases the signed position
const unsigned int DIR_SETUP_US = 200;   // driver DIR setup time before the first edge

const unsigned long PULSES_PER_REV = 25600UL;

// ------------------------------------------------------------
// STATIC PROFILE  (signed targets, degrees)
// ------------------------------------------------------------
const int STATIC_TARGETS[] = {
  0, 5, 10, 15, 20, 15, 10, 5, 0,
  -5, -10, -15, -20, -15, -10, -5, 0
};
const byte NUM_TARGETS = sizeof(STATIC_TARGETS) / sizeof(STATIC_TARGETS[0]);

const unsigned long HOLD_TIME_MS   = 10000UL;
const unsigned int  MOVE_RATE_DPS  = 5;

// ------------------------------------------------------------
// RATE PROFILE
// ------------------------------------------------------------
const int RATE_LOW_DEG  = -10;     // leg start
const int RATE_HIGH_DEG =  10;     // leg end

const unsigned int RATES_DPS[] = { 1, 2, 5, 10 };
const byte NUM_RATES = sizeof(RATES_DPS) / sizeof(RATES_DPS[0]);

// Full back-and-forth cycles per rate. One cycle = 2 legs.
// DEFAULT {2,2,2,2} as specified: 4 legs per rate, 144 s of rate motion.
// RECOMMENDED ALTERNATIVE, equal motion time per rate (~40 s each),
// which fixes the thin sample count at 10 deg/s:
//     const byte RATE_CYCLES[] = { 1, 2, 5, 10 };
const byte RATE_CYCLES[] = { 2, 2, 2, 2 };

// Optional extra rate, OFF by default. 15 deg/s over a 20 deg
// travel leaves only 1.33 s of leg and ~1.07 s after trimming,
// which is thin. Enable only deliberately.
const bool  ENABLE_OPTIONAL_15DPS = false;
const unsigned int OPTIONAL_RATE_DPS = 15;
const byte  OPTIONAL_RATE_CYCLES = 4;

// ------------------------------------------------------------
// BASELINES / TIMELINE
// ------------------------------------------------------------
const unsigned long START_BASELINE_MS = 60000UL;
const unsigned long END_BASELINE_MS   = 60000UL;
const unsigned long INTERMISSION_MS   = 20000UL;
const int           START_DELAY_SEC   = 10;

// [FIX v2] settling time held at the pre-position before software zero is
// redefined. Motor stays ENABLED throughout.
const unsigned long PREBIAS_SETTLE_MS = 3000UL;

// ------------------------------------------------------------
// RUN STATE
// ------------------------------------------------------------
int  runId = 0;
char axisName[8] = "AXIS_A";
bool runDone = false;
bool prebiasDone = false;
// [FIX v2] provenance of the commanded pre-position, reported in the header.
// This is a PULSE-DERIVED COMMANDED pre-position, never an encoder-verified
// or measured physical angle.
float prebiasDeg = 0.0f;
long  prebiasPulses = 0;
unsigned long prebiasBeginUs = 0;
unsigned long prebiasDoneUs = 0;

// SIGNED position in pulses relative to the mechanically
// established zero. Positive = DIR_LEVEL_POS direction.
long currentPositionPulses = 0;

char cmdBuf[40];
byte cmdLen = 0;


// ============================================================
// GEOMETRY
// ============================================================
long angleToPulses(float deg)
{
  // magnitude rounded exactly as in the yaw campaign, sign preserved
  float a = fabs(deg);
  long p = lround(a * (float)PULSES_PER_REV / 360.0f);
  return (deg < 0.0f) ? -p : p;
}

float pulsesToAngle(long pulses)
{
  return (float)pulses * 360.0f / (float)PULSES_PER_REV;
}


// ============================================================
// EVENT STAMPS
// Serial.flush() empties the TX buffer BEFORE micros() is
// latched, so the stamp print cannot block and the gap between
// the latched timestamp and the first step edge stays in the
// microsecond range.
// Never called from inside runPulsesAtRate().
// ============================================================
void stamp(const char *ev, long value)
{
  Serial.flush();
  unsigned long us = micros();
  Serial.print(F("#S,")); Serial.print(ev);
  Serial.print(','); Serial.print(us);
  Serial.print(','); Serial.println(value);
}

void stamp2(const char *ev, long value, long extra)
{
  Serial.flush();
  unsigned long us = micros();
  Serial.print(F("#S,")); Serial.print(ev);
  Serial.print(','); Serial.print(us);
  Serial.print(','); Serial.print(value);
  Serial.print(','); Serial.println(extra);
}


// ============================================================
// ACCURATE BRESENHAM PULSE GENERATOR
// (identical maths to the validated yaw sketch)
//
// half-period [us] = 28125 / (4 * rateDps)
//      period [us] = 14062.5 / rateDps
// The integer remainder keeps the average pulse rate exactly
//      rateDps * PULSES_PER_REV / 360   pulses per second,
// so the commanded angular rate carries no quantisation error;
// only the segment duration is quantised.
// ============================================================
void runPulsesAtRate(unsigned long nPulses, unsigned int rateDps)
{
  if (nPulses == 0UL || rateDps == 0) return;

  const unsigned long NUM = 28125UL;
  const unsigned long DEN = 4UL * (unsigned long)rateDps;

  unsigned long q = NUM / DEN;
  unsigned long r = NUM % DEN;

  unsigned long t0 = micros();
  unsigned long targetTime = 0;
  unsigned long remainder = 0;

  bool level = LOW;
  const unsigned long edges = 2UL * nPulses;

  for (unsigned long i = 0; i < edges; i++)
  {
    targetTime += q;
    remainder += r;
    if (remainder >= DEN) { targetTime++; remainder -= DEN; }

    while ((long)(micros() - t0 - targetTime) < 0) { }

    level = !level;
    digitalWrite(STEP_PIN, level);
  }
  digitalWrite(STEP_PIN, LOW);
}


// ============================================================
// SIGNED MOVE
// Sets DIR from the sign of the delta, waits the driver setup
// time, then emits the pulse train. This is the change that
// makes a bidirectional profile possible in one run.
// ============================================================
void moveToPulses(long targetPulses, unsigned int rateDps)
{
  long delta = targetPulses - currentPositionPulses;
  if (delta == 0) return;

  bool positive = (delta > 0);
  digitalWrite(DIR_PIN, positive ? DIR_LEVEL_POS : !DIR_LEVEL_POS);
  delayMicroseconds(DIR_SETUP_US);

  runPulsesAtRate((unsigned long)labs(delta), rateDps);
  currentPositionPulses = targetPulses;
}


// ============================================================
void countdown()
{
  for (int i = START_DELAY_SEC; i > 0; i--)
  {
    Serial.print(F("# starting in ")); Serial.print(i); Serial.println(F(" s"));
    delay(1000);
  }
}


// ============================================================
// STATIC ANGLE PHASE
// ============================================================
void staticPhase()
{
  Serial.println(F("# ===== STATIC PHASE ====="));
  stamp("STATIC_PHASE_START", 0);

  for (byte i = 0; i < NUM_TARGETS; i++)
  {
    long tgt   = angleToPulses((float)STATIC_TARGETS[i]);
    long sDeg  = (long)STATIC_TARGETS[i];

    if (tgt != currentPositionPulses)
    {
      Serial.print(F("#M,MOVE,"));
      Serial.print(sDeg); Serial.print(',');
      Serial.print(pulsesToAngle(tgt), 6); Serial.print(',');
      Serial.println(tgt);

      stamp2("MOVE_START", sDeg, tgt);
      moveToPulses(tgt, MOVE_RATE_DPS);
      stamp2("MOVE_END", sDeg, tgt);
    }

    stamp2("HOLD_START", sDeg, tgt);
    delay(HOLD_TIME_MS);
    stamp2("HOLD_END", sDeg, tgt);
  }

  stamp("STATIC_PHASE_END", 0);
  Serial.println(F("# ===== STATIC PHASE COMPLETE ====="));
}


// ============================================================
// RATE PHASE
// Each leg is a signed constant-rate traverse between
// RATE_LOW_DEG and RATE_HIGH_DEG. Legs are logged individually.
// RATE_LEG_START/END value = SIGNED commanded rate (deg/s),
// extra = SIGNED destination position in pulses.
// ============================================================
void runRateBlock(unsigned int rate, byte cycles)
{
  long lo = angleToPulses((float)RATE_LOW_DEG);
  long hi = angleToPulses((float)RATE_HIGH_DEG);

  for (byte c = 0; c < cycles; c++)
  {
    // leg 1: low -> high, positive rate
    Serial.print(F("#M,LEG,")); Serial.print((long)rate);
    Serial.print(F(",UP,")); Serial.println(hi);
    stamp2("RATE_LEG_START", (long)rate, hi);
    moveToPulses(hi, rate);
    stamp2("RATE_LEG_END", (long)rate, hi);

    // leg 2: high -> low, negative rate
    Serial.print(F("#M,LEG,")); Serial.print(-(long)rate);
    Serial.print(F(",DOWN,")); Serial.println(lo);
    stamp2("RATE_LEG_START", -(long)rate, lo);
    moveToPulses(lo, rate);
    stamp2("RATE_LEG_END", -(long)rate, lo);
  }
}


void ratePhase()
{
  Serial.println(F("# ===== RATE PHASE ====="));
  stamp("RATE_PHASE_START", 0);

  long lo = angleToPulses((float)RATE_LOW_DEG);

  // position to the leg start at the slow transit rate
  stamp2("RATE_POSITION_START", (long)RATE_LOW_DEG, lo);
  moveToPulses(lo, MOVE_RATE_DPS);
  stamp2("RATE_POSITION_END", (long)RATE_LOW_DEG, lo);

  for (byte i = 0; i < NUM_RATES; i++)
    runRateBlock(RATES_DPS[i], RATE_CYCLES[i]);

  if (ENABLE_OPTIONAL_15DPS)
    runRateBlock(OPTIONAL_RATE_DPS, OPTIONAL_RATE_CYCLES);

  stamp("RATE_PHASE_END", 0);
  Serial.println(F("# ===== RATE PHASE COMPLETE ====="));
}


// ============================================================
// COMMAND-BASED RETURN TO ZERO
// This is a commanded return to the zero pulse position. It is
// NOT sensor-verified homing and must not be described as such.
// ============================================================
void commandedReturnToZero()
{
  Serial.print(F("#M,RETURN_ZERO,from,"));
  Serial.println(currentPositionPulses);

  stamp("RETURN_ZERO_START", currentPositionPulses);
  moveToPulses(0, MOVE_RATE_DPS);
  stamp("RETURN_ZERO_END", 0);
}


// ============================================================
// METADATA HEADER
// ============================================================
void printHeader()
{
  Serial.println();
  Serial.println(F("#FW,motor_pitch_roll_FINAL,v2"));
  // [FIX v2] provenance of the working point this run is relative to.
  Serial.print(F("#PREBIAS_APPLIED,"));   Serial.println(prebiasDone ? 1 : 0);
  Serial.print(F("#PREBIAS_DEG,"));       Serial.println(prebiasDeg, 6);
  Serial.print(F("#PREBIAS_PULSES,"));    Serial.println(prebiasPulses);
  Serial.print(F("#PREBIAS_NOTE,pulse-derived commanded pre-position, not encoder-verified"));
  Serial.println();
  Serial.print(F("#RUN_ID,"));           Serial.println(runId);
  Serial.print(F("#AXIS,"));             Serial.println(axisName);
  Serial.print(F("#PPR,"));              Serial.println(PULSES_PER_REV);
  Serial.print(F("#HOLD_S,"));           Serial.println(HOLD_TIME_MS / 1000UL);
  Serial.print(F("#MOVE_RATE_DPS,"));    Serial.println(MOVE_RATE_DPS);
  Serial.print(F("#START_BASELINE_S,")); Serial.println(START_BASELINE_MS / 1000UL);
  Serial.print(F("#END_BASELINE_S,"));   Serial.println(END_BASELINE_MS / 1000UL);
  Serial.print(F("#INTERMISSION_S,"));   Serial.println(INTERMISSION_MS / 1000UL);
  Serial.print(F("#COUNTDOWN_S,"));      Serial.println(START_DELAY_SEC);
  Serial.print(F("#RATE_RANGE_DEG,"));   Serial.print(RATE_LOW_DEG);
  Serial.print(','); Serial.println(RATE_HIGH_DEG);

  Serial.print(F("#STATIC_TARGETS"));
  for (byte i = 0; i < NUM_TARGETS; i++) {
    Serial.print(','); Serial.print(STATIC_TARGETS[i]);
  }
  Serial.println();

  Serial.print(F("#RATES_DPS"));
  for (byte i = 0; i < NUM_RATES; i++) { Serial.print(','); Serial.print(RATES_DPS[i]); }
  if (ENABLE_OPTIONAL_15DPS) { Serial.print(','); Serial.print(OPTIONAL_RATE_DPS); }
  Serial.println();

  Serial.print(F("#RATE_CYCLES"));
  for (byte i = 0; i < NUM_RATES; i++) { Serial.print(','); Serial.print(RATE_CYCLES[i]); }
  if (ENABLE_OPTIONAL_15DPS) { Serial.print(','); Serial.print(OPTIONAL_RATE_CYCLES); }
  Serial.println();
}


// ============================================================
void executeRun()
{
  printHeader();

  digitalWrite(DIR_PIN, DIR_LEVEL_POS);
  delay(50);
  // [FIX v2] The platform is already physically AT the established zero -- the
  // mechanically balanced start if no PREBIAS was commanded, or the held
  // pre-position if one was. Setting the counter to 0 redefines software zero
  // only; it commands NO motion. moveToPulses() is not called here.
  currentPositionPulses = 0;

  countdown();

  stamp2("RUN_BEGIN", (long)runId, (long)(axisName[5] == 'B' ? 2 : 1));

  Serial.println(F("# start baseline (stationary at zero)"));
  stamp("BASELINE_START", 0);
  delay(START_BASELINE_MS);
  stamp("BASELINE_START_END", 0);

  staticPhase();

  Serial.println(F("# intermission"));
  stamp("INTERMISSION_START", 0);
  delay(INTERMISSION_MS);
  stamp("INTERMISSION_END", 0);

  ratePhase();

  commandedReturnToZero();

  Serial.println(F("# end baseline (stationary at zero)"));
  stamp("BASELINE_END", 0);
  delay(END_BASELINE_MS);
  stamp("BASELINE_END_END", 0);

  stamp2("RUN_END", (long)runId, currentPositionPulses);

  Serial.println(F("# TEST COMPLETE"));
  runDone = true;
}


// ============================================================
// COMMAND PARSER:  START,<id>,<AXIS_A|AXIS_B>
// ============================================================
void handleCommand(char *s)
{
  // --------------------------------------------------------
  // PREBIAS,<deg>
  //
  // Moves from the mechanically balanced startup position
  // to a commanded working-point offset, then REDEFINES
  // that held physical position as software zero.
  //
  // IMPORTANT:
  // This is not encoder-verified homing or absolute angle.
  // It is a pulse-derived commanded pre-position.
  // --------------------------------------------------------
  if (strncmp(s, "PREBIAS,", 8) == 0)
  {
    if (runDone) {
      Serial.println(F("#ERR,RUN_ALREADY_DONE_RESET_BOARD"));
      return;
    }

    if (prebiasDone) {
      Serial.println(F("#ERR,PREBIAS_ALREADY_DONE"));
      return;
    }

    float deg = atof(s + 8);

    if (deg < -40.0f || deg > 40.0f) {
      Serial.println(F("#ERR,PREBIAS_RANGE_-40_TO_40"));
      return;
    }

    long tgt = angleToPulses(deg);

    // [FIX v2] Immediate ACK so the logger can confirm the command was parsed
    // before the blocking move begins.
    Serial.print(F("#ACK,PREBIAS,"));
    Serial.println(deg, 6);

    prebiasBeginUs = micros();
    Serial.print(F("#PREBIAS_BEGIN,"));
    Serial.print(deg, 6);
    Serial.print(',');
    Serial.print(tgt);
    Serial.print(F(",us="));
    Serial.println(prebiasBeginUs);

    // [FIX v2] Drain the TX buffer BEFORE the blocking move. moveToPulses()
    // busy-waits on micros() for several seconds; flushing here guarantees the
    // BEGIN line has physically left the UART first, so the logger cannot time
    // out waiting for a line that is still sitting in the buffer.
    Serial.flush();

    // Move at the already validated 5 deg/s transit rate.
    moveToPulses(tgt, MOVE_RATE_DPS);

    // Let the structure settle while motor remains enabled.
    // ENABLE stays asserted: the motor HOLDS this working point.
    delay(PREBIAS_SETTLE_MS);

    // The current held physical position becomes NEW ZERO.
    // No physical motion happens here; only the software counter is redefined.
    prebiasDeg = deg;
    prebiasPulses = tgt;
    currentPositionPulses = 0;
    prebiasDone = true;

    prebiasDoneUs = micros();
    Serial.print(F("#PREBIAS_DONE,"));
    Serial.print(deg, 6);
    Serial.print(F(",NEW_ZERO=0,pulses="));
    Serial.print(tgt);
    Serial.print(F(",us="));
    Serial.println(prebiasDoneUs);
    Serial.flush();

    return;
  }

  if (strncmp(s, "START", 5) != 0) {
    Serial.print(F("#ERR,UNKNOWN_COMMAND,")); Serial.println(s); return;
  }
  if (runDone) { Serial.println(F("#ERR,ALREADY_RUN_RESET_BOARD")); return; }

  char *p1 = strchr(s, ',');
  if (!p1) { Serial.println(F("#ERR,BAD_FORMAT")); return; }
  char *p2 = strchr(p1 + 1, ',');
  if (!p2) { Serial.println(F("#ERR,BAD_FORMAT")); return; }
  *p1 = '\0'; *p2 = '\0';

  int id = atoi(p1 + 1);
  if (id < 1 || id > 20) {
    Serial.print(F("#ERR,BAD_RUN_ID,")); Serial.println(p1 + 1); return;
  }

  char *ax = p2 + 1;
  for (char *c = ax; *c; c++) *c = (char)toupper((unsigned char)*c);

  if (strcmp(ax, "AXIS_A") == 0)      strcpy(axisName, "AXIS_A");
  else if (strcmp(ax, "AXIS_B") == 0) strcpy(axisName, "AXIS_B");
  else { Serial.print(F("#ERR,BAD_AXIS,")); Serial.println(ax); return; }

  runId = id;
  Serial.print(F("#ACK,START,")); Serial.print(runId);
  Serial.print(','); Serial.println(axisName);

  executeRun();
}


// ============================================================
void setup()
{
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(DIR_PIN, DIR_LEVEL_POS);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, MOTOR_ENABLE);   // holding torque on, stays on

  delay(500);

  Serial.println();
  Serial.println(F("#FW,motor_pitch_roll_FINAL,v2"));
  Serial.println(F("#CMD,PREBIAS,<deg -40..40>"));
  Serial.println(F("#CMD,START,<runId 1-20>,<AXIS_A|AXIS_B>"));
  Serial.println(F("READY"));
}


// ============================================================
void loop()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
      continue;
    }
    if (cmdLen < (byte)(sizeof(cmdBuf) - 1)) cmdBuf[cmdLen++] = c;
    else cmdLen = 0;
  }
}
