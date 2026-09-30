// ============================================================
// motor_yaw_FINAL.ino
// ------------------------------------------------------------
// FINAL PRODUCTION SKETCH -- IEEE Aerospace yaw campaign (10 runs)
// UIM5756PM / UIM344 integrated closed-loop NEMA23
//
// Wiring:  DIR -> D8   PLS -> D9   ENA -> D7   COM -> GND
// Motor:   1/128 microstepping, 25600 pulses / revolution
//
// ONE sketch for all ten runs. Nothing is edited between runs.
// The run identity arrives over serial:
//
//     START,<runId 1..10>,<CW|CCW>
//
// Profile (identical for every run, CW and CCW differ only in the
// commanded direction):
//     60 s stationary baseline
//     static angles 0,5,10,20,30,60,90,120,150,180,210,240,270,
//                   300,330,360,720  (magnitudes), 20 s hold each,
//                   transitions at 6 deg/s
//     20 s intermission
//     rates 1,5,10,15,20,25,30 deg/s, 20 s each
//     commanded return to the same orientation modulo one revolution
//     60 s stationary baseline
//
// Duration from START to the end of the final baseline:
//     10 (countdown) + 60 + 460.000 + 20 + 139.996 + 6.668 + 60
//   = 756.664 s
//
// EVENT LOG FORMAT (parsed by motor_serial_logger_FINAL.py):
//     #S,<EVENT>,<micros>,<signed_value>[,<signed_extra>]
// All physical command values carry the direction sign:
// CW -> positive, CCW -> negative. Internal pulse counters remain
// positive magnitudes; only the logged values are signed.
//
// Units per event:
//   RUN_BEGIN              value = direction sign (+1 / -1)
//   BASELINE_START_BEGIN   value = 0
//   BASELINE_START_END     value = 0
//   ANGLE_PHASE_BEGIN/END  value = 0
//   MOVE_START / MOVE_END  value = signed target angle [deg]
//                          extra = signed target position [pulses]
//   HOLD_START / HOLD_END  value = signed target angle [deg]
//                          extra = signed target position [pulses]
//   INTERMISSION_BEGIN/END value = 0
//   RATE_PHASE_BEGIN/END   value = 0
//   RATE_START / RATE_END  value = signed commanded rate [deg/s]
//                          extra = signed segment length [pulses]
//   RETURN_START/RETURN_END value = signed return length [pulses]
//   BASELINE_END_BEGIN/END value = 0
//   RUN_END                value = signed total travel [pulses]
//
// Fractional commanded angles (which are not integers because of
// pulse quantisation) are printed on separate "#M," metadata lines
// so the "#S," lines stay a fixed, machine-parsable shape.
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

const bool MOTOR_ENABLE       = HIGH;   // verified on this hardware
const bool DIR_LEVEL_CW       = HIGH;   // level that produced run 1 (+) direction

const unsigned long PULSES_PER_REV = 25600UL;

// ------------------------------------------------------------
// PROFILE CONSTANTS
// ------------------------------------------------------------
const int ANGLES[] = {
  0, 5, 10, 20, 30, 60, 90, 120, 150,
  180, 210, 240, 270, 300, 330, 360, 720
};
const byte NUM_ANGLES = sizeof(ANGLES) / sizeof(ANGLES[0]);

const unsigned int RATES[] = { 1, 5, 10, 15, 20, 25, 30 };
const byte NUM_RATES = sizeof(RATES) / sizeof(RATES[0]);

const unsigned long HOLD_TIME_MS        = 20000UL;
const unsigned long RATE_TIME_SEC       = 20UL;
const unsigned long INTERMISSION_MS     = 20000UL;
const unsigned long START_BASELINE_MS   = 60000UL;
const unsigned long END_BASELINE_MS     = 60000UL;
const unsigned int  ANGLE_MOVE_RATE_DPS = 6;
const int           START_DELAY_SEC     = 10;

// ------------------------------------------------------------
// RUN STATE
// ------------------------------------------------------------
int  runId   = 0;
int  runSign = +1;                 // +1 = CW, -1 = CCW
char runDirText[4] = "CW";
bool runDone = false;

long currentPositionPulses = 0;    // magnitude of travel from run start

// serial command buffer
char cmdBuf[32];
byte cmdLen = 0;


// ============================================================
// GEOMETRY HELPERS
// ============================================================
long angleToPulses(float deg)
{
  return lround(
    deg * (float)PULSES_PER_REV / 360.0f
  );
}

float pulsesToAngle(long pulses)
{
  return (float)pulses * 360.0f / (float)PULSES_PER_REV;
}


// ============================================================
// EVENT STAMPS
// The micros() value is latched BEFORE any printing so that the
// timestamp is not delayed by serial transmission.
// Never called from inside runPulsesAtRate().
// ============================================================
void stamp(const char *ev, long value)
{
  Serial.flush();                 // TX buffer empty before latching
  unsigned long us = micros();
  Serial.print(F("#S,"));
  Serial.print(ev);
  Serial.print(',');
  Serial.print(us);
  Serial.print(',');
  Serial.println(value);
}

void stamp2(const char *ev, long value, long extra)
{
  Serial.flush();                 // TX buffer empty before latching
  unsigned long us = micros();
  Serial.print(F("#S,"));
  Serial.print(ev);
  Serial.print(',');
  Serial.print(us);
  Serial.print(',');
  Serial.print(value);
  Serial.print(',');
  Serial.println(extra);
}


// ============================================================
// ACCURATE BRESENHAM PULSE GENERATOR  (UNCHANGED)
//
// half-period [us] = 28125 / (4 * rateDps)
//      period [us] = 14062.5 / rateDps
// The integer remainder term keeps the average pulse rate exactly
//      rateDps * PULSES_PER_REV / 360   pulses per second,
// so the commanded angular rate carries no quantisation error.
// ============================================================
void runPulsesAtRate(
  unsigned long nPulses,
  unsigned int rateDps)
{
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

    if (remainder >= DEN)
    {
      targetTime++;
      remainder -= DEN;
    }

    while ((long)(micros() - t0 - targetTime) < 0)
    {
    }

    level = !level;
    digitalWrite(STEP_PIN, level);
  }

  digitalWrite(STEP_PIN, LOW);
}


// ============================================================
// COUNTDOWN
// ============================================================
void countdown()
{
  for (int i = START_DELAY_SEC; i > 0; i--)
  {
    Serial.print(F("# starting in "));
    Serial.print(i);
    Serial.println(F(" s"));
    delay(1000);
  }
}


// ============================================================
// STATIC ANGLE PHASE
// delta is a magnitude, so the profile is identical for CW and CCW.
// ============================================================
void angleTest()
{
  Serial.println(F("# ===== STATIC ANGLE PHASE ====="));
  stamp("ANGLE_PHASE_BEGIN", 0);

  for (byte i = 0; i < NUM_ANGLES; i++)
  {
    long target = angleToPulses((float)ANGLES[i]);
    long delta  = target - currentPositionPulses;

    long sAngle  = (long)runSign * (long)ANGLES[i];
    long sTarget = (long)runSign * target;

    if (delta > 0)
    {
      Serial.print(F("#M,MOVE,"));
      Serial.print(sAngle);
      Serial.print(',');
      Serial.print((float)runSign * pulsesToAngle(target), 6);
      Serial.print(',');
      Serial.println(sTarget);

      stamp2("MOVE_START", sAngle, sTarget);
      runPulsesAtRate((unsigned long)delta, ANGLE_MOVE_RATE_DPS);
      stamp2("MOVE_END", sAngle, sTarget);

      currentPositionPulses = target;
    }

    stamp2("HOLD_START", sAngle, sTarget);
    delay(HOLD_TIME_MS);
    stamp2("HOLD_END", sAngle, sTarget);
  }

  stamp("ANGLE_PHASE_END", 0);
  Serial.println(F("# ===== STATIC ANGLE PHASE COMPLETE ====="));
}


// ============================================================
// ANGULAR RATE PHASE
// ============================================================
void velocityTest()
{
  Serial.println(F("# ===== RATE PHASE ====="));
  stamp("RATE_PHASE_BEGIN", 0);

  for (byte i = 0; i < NUM_RATES; i++)
  {
    unsigned int rate = RATES[i];

    unsigned long pulses =
      (unsigned long)lround(
        (float)rate *
        (float)RATE_TIME_SEC *
        (float)PULSES_PER_REV /
        360.0f
      );

    long sRate   = (long)runSign * (long)rate;
    long sPulses = (long)runSign * (long)pulses;

    // NOTE: this angle divided by the NOMINAL 20 s is not the
    // effective rate. The segment lasts pulses*14062.5/rate us,
    // over which the mean rate is exactly `rate` deg/s.
    Serial.print(F("#M,RATE,"));
    Serial.print(sRate);
    Serial.print(',');
    Serial.print((float)runSign * pulsesToAngle((long)pulses), 6);
    Serial.print(',');
    Serial.println(sPulses);

    stamp2("RATE_START", sRate, sPulses);
    runPulsesAtRate(pulses, rate);
    stamp2("RATE_END", sRate, sPulses);

    currentPositionPulses += (long)pulses;
  }

  stamp("RATE_PHASE_END", 0);
  Serial.println(F("# ===== RATE PHASE COMPLETE ====="));
}


// ============================================================
// COMMANDED RETURN TO THE SAME ORIENTATION MODULO ONE REVOLUTION
//
// Continues in the SAME direction to the next exact multiple of
// PULSES_PER_REV, so every run ends with the platform and all tags
// in the orientation they started in, with no direction reversal.
// For this profile 201955 pulses are travelled, so 2845 pulses
// (40.0078 deg) remain.
// ============================================================
void commandedReturnToWholeRevolution()
{
  long rem = currentPositionPulses % (long)PULSES_PER_REV;

  if (rem == 0)
  {
    Serial.println(F("# already on a whole revolution, no return move"));
    stamp("RETURN_START", 0);
    stamp("RETURN_END", 0);
    return;
  }

  long ret  = (long)PULSES_PER_REV - rem;
  long sRet = (long)runSign * ret;

  Serial.print(F("#M,RETURN,"));
  Serial.print(sRet);
  Serial.print(',');
  Serial.println((float)runSign * pulsesToAngle(ret), 6);

  stamp("RETURN_START", sRet);
  runPulsesAtRate((unsigned long)ret, ANGLE_MOVE_RATE_DPS);
  stamp("RETURN_END", sRet);

  currentPositionPulses += ret;
}


// ============================================================
// HEADER
// ============================================================
void printHeader()
{
  Serial.println();
  Serial.println(F("#FW,motor_yaw_FINAL,v3"));
  Serial.print(F("#RUN_ID,"));           Serial.println(runId);
  Serial.print(F("#RUN_DIR,"));          Serial.println(runDirText);
  Serial.print(F("#RUN_DIR_SIGN,"));     Serial.println(runSign);
  Serial.print(F("#PPR,"));              Serial.println(PULSES_PER_REV);
  Serial.print(F("#STATIC_HOLD_S,"));    Serial.println(HOLD_TIME_MS / 1000UL);
  Serial.print(F("#RATE_HOLD_S,"));      Serial.println(RATE_TIME_SEC);
  Serial.print(F("#START_BASELINE_S,")); Serial.println(START_BASELINE_MS / 1000UL);
  Serial.print(F("#END_BASELINE_S,"));   Serial.println(END_BASELINE_MS / 1000UL);
  Serial.print(F("#MOVE_RATE_DPS,"));    Serial.println(ANGLE_MOVE_RATE_DPS);
  Serial.print(F("#INTERMISSION_S,"));   Serial.println(INTERMISSION_MS / 1000UL);
  Serial.print(F("#COUNTDOWN_S,"));      Serial.println(START_DELAY_SEC);
}


// ============================================================
// RUN
// ============================================================
void executeRun()
{
  printHeader();

  // DIR is set once per run; all motion inside a run is
  // one-directional, so no backlash is taken up mid-run.
  digitalWrite(DIR_PIN, (runSign > 0) ? DIR_LEVEL_CW : !DIR_LEVEL_CW);
  delay(50);                      // driver DIR setup time

  currentPositionPulses = 0;

  countdown();

  stamp("RUN_BEGIN", (long)runSign);

  Serial.println(F("# start baseline (stationary)"));
  stamp("BASELINE_START_BEGIN", 0);
  delay(START_BASELINE_MS);
  stamp("BASELINE_START_END", 0);

  angleTest();

  Serial.println(F("# intermission"));
  stamp("INTERMISSION_BEGIN", 0);
  delay(INTERMISSION_MS);
  stamp("INTERMISSION_END", 0);

  velocityTest();

  commandedReturnToWholeRevolution();

  Serial.println(F("# end baseline (stationary)"));
  stamp("BASELINE_END_BEGIN", 0);
  delay(END_BASELINE_MS);
  stamp("BASELINE_END_END", 0);

  stamp("RUN_END", (long)runSign * currentPositionPulses);

  Serial.println(F("# TEST COMPLETE"));
  runDone = true;
}


// ============================================================
// COMMAND PARSER:  START,<id>,<CW|CCW>
// ============================================================
void handleCommand(char *s)
{
  // strip CR / whitespace already done by caller
  if (strncmp(s, "START", 5) != 0)
  {
    Serial.print(F("#ERR,UNKNOWN_COMMAND,"));
    Serial.println(s);
    return;
  }

  if (runDone)
  {
    Serial.println(F("#ERR,ALREADY_RUN_RESET_BOARD"));
    return;
  }

  char *p1 = strchr(s, ',');
  if (p1 == NULL) { Serial.println(F("#ERR,BAD_FORMAT")); return; }
  char *p2 = strchr(p1 + 1, ',');
  if (p2 == NULL) { Serial.println(F("#ERR,BAD_FORMAT")); return; }

  *p1 = '\0';
  *p2 = '\0';
  char *idStr  = p1 + 1;
  char *dirStr = p2 + 1;

  int id = atoi(idStr);
  if (id < 1 || id > 10)
  {
    Serial.print(F("#ERR,BAD_RUN_ID,"));
    Serial.println(idStr);
    return;
  }

  // upper-case the direction token in place
  for (char *c = dirStr; *c; c++) *c = (char)toupper((unsigned char)*c);

  if (strcmp(dirStr, "CW") == 0)
  {
    runSign = +1;
    strcpy(runDirText, "CW");
  }
  else if (strcmp(dirStr, "CCW") == 0)
  {
    runSign = -1;
    strcpy(runDirText, "CCW");
  }
  else
  {
    Serial.print(F("#ERR,BAD_DIRECTION,"));
    Serial.println(dirStr);
    return;
  }

  runId = id;
  Serial.print(F("#ACK,START,"));
  Serial.print(runId);
  Serial.print(',');
  Serial.println(runDirText);

  executeRun();
}


// ============================================================
void setup()
{
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(DIR_PIN, DIR_LEVEL_CW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, MOTOR_ENABLE);   // holding torque on, stays on

  delay(500);

  Serial.println();
  Serial.println(F("#FW,motor_yaw_FINAL,v3"));
  Serial.println(F("#CMD,START,<runId 1-10>,<CW|CCW>"));
  Serial.println(F("READY"));
}


// ============================================================
void loop()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0)
        handleCommand(cmdBuf);
      cmdLen = 0;
      continue;
    }

    if (cmdLen < (byte)(sizeof(cmdBuf) - 1))
      cmdBuf[cmdLen++] = c;
    else
      cmdLen = 0;          // overlong line, discard
  }
}
