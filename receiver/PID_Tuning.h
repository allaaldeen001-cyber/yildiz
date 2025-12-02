/*
 * PID_Tuning.h - Serial-based PID tuning utility
 * 
 * Allows real-time PID adjustment via Serial Monitor
 * 
 * Usage (in Serial Monitor):
 *   RP1.5  - Set Roll P to 1.5
 *   RI0.05 - Set Roll I to 0.05
 *   RD20   - Set Roll D to 20
 *   PP1.5  - Set Pitch P to 1.5
 *   PI0.05 - Set Pitch I to 0.05
 *   PD20   - Set Pitch D to 20
 *   YP4.0  - Set Yaw P to 4.0
 *   YI0.02 - Set Yaw I to 0.02
 *   YD0    - Set Yaw D to 0
 *   PRINT  - Print current PID values
 *   SAVE   - Print code to paste into receiver.ino
 */

#ifndef PID_TUNING_H
#define PID_TUNING_H

// External PID variables (defined in receiver.ino)
extern float rollKp, rollKi, rollKd;
extern float pitchKp, pitchKi, pitchKd;
extern float yawKp, yawKi, yawKd;

// Serial buffer for commands
char serialBuffer[32];
uint8_t serialIndex = 0;

void processPIDCommand() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        parseCommand(serialBuffer);
        serialIndex = 0;
      }
    } else if (serialIndex < 31) {
      serialBuffer[serialIndex++] = c;
    }
  }
}

void parseCommand(char* cmd) {
  // Convert to uppercase for easier parsing
  for (int i = 0; cmd[i]; i++) {
    cmd[i] = toupper(cmd[i]);
  }
  
  if (strncmp(cmd, "PRINT", 5) == 0) {
    printPIDValues();
    return;
  }
  
  if (strncmp(cmd, "SAVE", 4) == 0) {
    printSaveCode();
    return;
  }
  
  // Parse PID commands (e.g., RP1.5, RD20, etc.)
  if (strlen(cmd) >= 3) {
    char axis = cmd[0];     // R, P, or Y
    char param = cmd[1];    // P, I, or D
    float value = atof(&cmd[2]);
    
    switch (axis) {
      case 'R':  // Roll
        switch (param) {
          case 'P': rollKp = value; Serial.print(F("Roll P = ")); Serial.println(value); break;
          case 'I': rollKi = value; Serial.print(F("Roll I = ")); Serial.println(value); break;
          case 'D': rollKd = value; Serial.print(F("Roll D = ")); Serial.println(value); break;
        }
        break;
        
      case 'P':  // Pitch
        switch (param) {
          case 'P': pitchKp = value; Serial.print(F("Pitch P = ")); Serial.println(value); break;
          case 'I': pitchKi = value; Serial.print(F("Pitch I = ")); Serial.println(value); break;
          case 'D': pitchKd = value; Serial.print(F("Pitch D = ")); Serial.println(value); break;
        }
        break;
        
      case 'Y':  // Yaw
        switch (param) {
          case 'P': yawKp = value; Serial.print(F("Yaw P = ")); Serial.println(value); break;
          case 'I': yawKi = value; Serial.print(F("Yaw I = ")); Serial.println(value); break;
          case 'D': yawKd = value; Serial.print(F("Yaw D = ")); Serial.println(value); break;
        }
        break;
        
      default:
        Serial.println(F("Unknown command. Use RP, RI, RD, PP, PI, PD, YP, YI, YD, PRINT, or SAVE"));
    }
  }
}

void printPIDValues() {
  Serial.println(F("\n=== Current PID Values ==="));
  Serial.print(F("Roll  - P: ")); Serial.print(rollKp, 3);
  Serial.print(F("  I: ")); Serial.print(rollKi, 4);
  Serial.print(F("  D: ")); Serial.println(rollKd, 2);
  
  Serial.print(F("Pitch - P: ")); Serial.print(pitchKp, 3);
  Serial.print(F("  I: ")); Serial.print(pitchKi, 4);
  Serial.print(F("  D: ")); Serial.println(pitchKd, 2);
  
  Serial.print(F("Yaw   - P: ")); Serial.print(yawKp, 3);
  Serial.print(F("  I: ")); Serial.print(yawKi, 4);
  Serial.print(F("  D: ")); Serial.println(yawKd, 2);
  Serial.println();
}

void printSaveCode() {
  Serial.println(F("\n=== Copy this code to receiver.ino ===\n"));
  
  Serial.print(F("float rollKp = ")); Serial.print(rollKp, 3); Serial.println(F(";"));
  Serial.print(F("float rollKi = ")); Serial.print(rollKi, 4); Serial.println(F(";"));
  Serial.print(F("float rollKd = ")); Serial.print(rollKd, 2); Serial.println(F(";"));
  Serial.println();
  
  Serial.print(F("float pitchKp = ")); Serial.print(pitchKp, 3); Serial.println(F(";"));
  Serial.print(F("float pitchKi = ")); Serial.print(pitchKi, 4); Serial.println(F(";"));
  Serial.print(F("float pitchKd = ")); Serial.print(pitchKd, 2); Serial.println(F(";"));
  Serial.println();
  
  Serial.print(F("float yawKp = ")); Serial.print(yawKp, 3); Serial.println(F(";"));
  Serial.print(F("float yawKi = ")); Serial.print(yawKi, 4); Serial.println(F(";"));
  Serial.print(F("float yawKd = ")); Serial.print(yawKd, 2); Serial.println(F(";"));
  Serial.println(F("\n=================================\n"));
}

#endif // PID_TUNING_H
