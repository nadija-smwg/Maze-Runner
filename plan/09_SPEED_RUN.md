# 09 — Speed Run Strategy & Implementation
## Turning Exploration into a Fast, Optimized Run

---

## Overview

After exploration, the robot has a **complete wall map**. The speed run uses this map to race from start to goal on the **shortest path** at maximum speed, without stopping to scan walls.

```mermaid
flowchart LR
    A[Load Saved Maze] --> B[Compute Optimal Path]
    B --> C[Compress into Segments]
    C --> D[Plan Velocity Profile]
    D --> E[Execute Speed Run]
    E --> F[DONE - Victory!]
```

---

## Step 1: Compute the Optimal Path

After the maze is fully explored, flood fill gives us the shortest path. Convert it to a sequence of moves:

```cpp
// Path as a list of directions to take from start to goal
int optimalPath[256];   // Directions: 0=N 1=E 2=S 3=W
int pathLength = 0;

void computeOptimalPath() {
  // Run flood fill with goal as target
  returnToStart = false;
  floodFill();
  
  // Start from (0,0), greedily follow lowest flood values
  int cx = 0, cy = 0;
  pathLength = 0;
  
  while (!isGoal(cx, cy)) {
    int bestDir = -1;
    uint8_t bestVal = 255;
    
    for (int dir = 0; dir < 4; dir++) {
      if (hasWall(cx, cy, dir)) continue;
      int nx = cx + dx[dir];
      int ny = cy + dy[dir];
      if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (floodValues[nx][ny] < bestVal) {
        bestVal = floodValues[nx][ny];
        bestDir = dir;
      }
    }
    
    if (bestDir == -1) {
      Serial.println("ERROR: No path found!");
      return;
    }
    
    optimalPath[pathLength++] = bestDir;
    cx += dx[bestDir];
    cy += dy[bestDir];
  }
  
  Serial.print("Optimal path: ");
  Serial.print(pathLength);
  Serial.println(" cells");
}
```

---

## Step 2: Compress Path into Segments

Instead of stop-and-go cell by cell, merge consecutive straight moves into segments:

```cpp
struct PathSegment {
  enum Type { STRAIGHT, TURN_LEFT, TURN_RIGHT, U_TURN } type;
  int cells;  // For STRAIGHT: number of consecutive cells to traverse
};

PathSegment segments[128];
int numSegments = 0;

void compressPath() {
  numSegments = 0;
  int currentHeading = 0;  // Start facing North
  
  for (int i = 0; i < pathLength; i++) {
    int targetDir = optimalPath[i];
    int turns = (targetDir - currentHeading + 4) % 4;
    
    // Add turn segment if needed
    if (turns == 1) {
      segments[numSegments++] = {PathSegment::TURN_RIGHT, 0};
    } else if (turns == 3) {
      segments[numSegments++] = {PathSegment::TURN_LEFT, 0};
    } else if (turns == 2) {
      segments[numSegments++] = {PathSegment::U_TURN, 0};
    }
    
    currentHeading = targetDir;
    
    // Count consecutive straight moves
    int straightCount = 1;
    while (i + 1 < pathLength && optimalPath[i + 1] == targetDir) {
      straightCount++;
      i++;
    }
    
    segments[numSegments++] = {PathSegment::STRAIGHT, straightCount};
  }
  
  Serial.print("Compressed to ");
  Serial.print(numSegments);
  Serial.println(" segments");
  
  // Print segments for debugging
  for (int i = 0; i < numSegments; i++) {
    switch (segments[i].type) {
      case PathSegment::STRAIGHT:
        Serial.print("STRAIGHT x"); Serial.println(segments[i].cells);
        break;
      case PathSegment::TURN_LEFT:  Serial.println("TURN LEFT"); break;
      case PathSegment::TURN_RIGHT: Serial.println("TURN RIGHT"); break;
      case PathSegment::U_TURN:     Serial.println("U-TURN"); break;
    }
  }
}
```

---

## Step 3: Velocity Profiling

### Trapezoidal Profile
For straight segments, use a trapezoidal velocity profile instead of constant speed:

```
Speed
  ^
  |    ___________
  |   /           \
  |  / Cruise      \
  | / Phase         \
  |/                 \
  +--------------------> Distance
  Accel   Cruise   Decel
```

```cpp
// Trapezoidal velocity profile for multi-cell straight segments
struct VelocityProfile {
  int maxSpeed;      // Maximum PWM during cruise phase
  int minSpeed;      // Minimum PWM at start/end
  float accelDist;   // Distance (in counts) for acceleration phase
  float decelDist;   // Distance (in counts) for deceleration phase
};

int calculateSpeed(VelocityProfile& vp, long currentCounts, long totalCounts) {
  float progress = (float)currentCounts / totalCounts;
  
  float accelFrac = vp.accelDist / totalCounts;
  float decelFrac = vp.decelDist / totalCounts;
  float decelStart = 1.0 - decelFrac;
  
  if (progress < accelFrac) {
    // Acceleration phase — linear ramp up
    float t = progress / accelFrac;
    return vp.minSpeed + (int)(t * (vp.maxSpeed - vp.minSpeed));
  } else if (progress > decelStart) {
    // Deceleration phase — linear ramp down
    float t = (progress - decelStart) / decelFrac;
    return vp.maxSpeed - (int)(t * (vp.maxSpeed - vp.minSpeed));
  } else {
    // Cruise phase — full speed
    return vp.maxSpeed;
  }
}
```

---

## Step 4: Smooth Turns (No Stop Required)

### In-place turns (simple, reliable):
```cpp
void speedRunTurnRight() {
  // Same as exploration but faster
  float targetAngle = 90.0;
  float yaw = 0;
  unsigned long lastTime = millis();
  
  while (abs(yaw) < targetAngle - 2.0) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    yaw += readGyroYaw() * dt;
    
    float remaining = targetAngle - abs(yaw);
    int speed = (remaining > 15) ? 150 : 80;  // Faster than exploration!
    
    setMotors(speed, -speed);
    delay(3);
  }
  
  setMotors(0, 0);
  delay(30);  // Shorter settle time
  robotHeading = (robotHeading + 1) % 4;
}

void speedRunTurnLeft() {
  float targetAngle = 90.0;
  float yaw = 0;
  unsigned long lastTime = millis();
  
  while (abs(yaw) < targetAngle - 2.0) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    yaw += readGyroYaw() * dt;
    
    float remaining = targetAngle - abs(yaw);
    int speed = (remaining > 15) ? 150 : 80;
    
    setMotors(-speed, speed);
    delay(3);
  }
  
  setMotors(0, 0);
  delay(30);
  robotHeading = (robotHeading + 3) % 4;
}
```

### Advanced: Smooth curve turns (no stopping)
```
For advanced micromice, turns can be executed while still moving forward:
- Start turning before reaching the cell boundary
- Follow a circular arc through the turn
- This eliminates stop-turn-go delays

This requires careful geometry calculations and is recommended
only after basic speed runs are working reliably.
```

---

## Step 5: Execute Speed Run

```cpp
void executeSpeedRun() {
  Serial.println("=== SPEED RUN START ===");
  
  // Re-compute path from current maze data
  computeOptimalPath();
  compressPath();
  
  // Reset position and heading
  robotX = 0; robotY = 0;
  robotHeading = 0;  // Facing North
  
  // Speed run velocity profile
  VelocityProfile vp;
  vp.maxSpeed = 200;    // Much faster than exploration (150)
  vp.minSpeed = 100;
  vp.accelDist = 0.2;   // 20% of distance for acceleration
  vp.decelDist = 0.25;  // 25% for deceleration
  
  for (int i = 0; i < numSegments; i++) {
    switch (segments[i].type) {
      case PathSegment::TURN_RIGHT:
        speedRunTurnRight();
        break;
        
      case PathSegment::TURN_LEFT:
        speedRunTurnLeft();
        break;
        
      case PathSegment::U_TURN:
        speedRunTurnRight();
        delay(50);
        speedRunTurnRight();
        break;
        
      case PathSegment::STRAIGHT: {
        int cells = segments[i].cells;
        long totalCounts = (long)(cells * CELL_SIZE_MM / MM_PER_COUNT);
        long startL = encoderL;
        long startR = encoderR;
        
        float yawRef = 0;
        unsigned long lastTime = millis();
        
        while (true) {
          long dL = encoderL - startL;
          long dR = encoderR - startR;
          long avgCount = (dL + dR) / 2;
          
          if (avgCount >= totalCounts) break;
          
          // Velocity profiling
          int speed = calculateSpeed(vp, avgCount, totalCounts);
          
          // Gyro straight correction
          unsigned long now = millis();
          float dt = (now - lastTime) / 1000.0;
          lastTime = now;
          yawRef += readGyroYaw() * dt;
          
          float encDiff = dL - dR;
          float correction = Kp_straight * (yawRef * 2.0 + encDiff * 0.5);
          
          setMotors(speed - correction, speed + correction);
          delay(3);  // Faster loop for speed run
        }
        
        // Update position
        for (int j = 0; j < cells; j++) {
          robotX += dx[robotHeading];
          robotY += dy[robotHeading];
        }
        break;
      }
    }
  }
  
  setMotors(0, 0);
  Serial.println("=== SPEED RUN COMPLETE ===");
  Serial.print("Final position: (");
  Serial.print(robotX); Serial.print(",");
  Serial.print(robotY); Serial.println(")");
}
```

---

## Speed Run Tuning Parameters

| Parameter | Exploration | Speed Run | Notes |
|-----------|-------------|-----------|-------|
| Base speed (PWM) | 120–150 | 180–220 | Start conservative |
| Turn speed (PWM) | 80–120 | 120–150 | Faster but still accurate |
| PID loop interval | 20ms | 5–10ms | Faster corrections at speed |
| Deceleration start | 80% of cell | 75% of segment | Earlier braking at speed |
| Post-turn delay | 50ms | 20–30ms | Less settling time |
| Kp_straight | 2.0–3.0 | 3.0–5.0 | Higher gain for speed |

---

## Speed Run Benchmarks

| Level | Time per cell | 16-cell path | Status |
|-------|--------------|--------------|--------|
| Exploration | 500–800ms | 8–13 sec | Baseline |
| Speed Run v1 (stop-and-turn) | 300–400ms | 5–7 sec | Good first attempt |
| Speed Run v2 (fast in-place turns) | 200–300ms | 3–5 sec | Competitive |
| Speed Run v3 (smooth curve turns) | 150–200ms | 2.5–3.5 sec | Advanced |
| Competition winner | 50–100ms | 1–2 sec | Expert level |

---

## Multiple Speed Runs

Competition rules usually allow multiple runs. Each subsequent run can be faster:

```cpp
int speedRunAttempt = 0;
int speedMultiplier[] = {100, 130, 160, 200};  // % of base speed

// Before each speed run:
speedRunAttempt++;
vp.maxSpeed = 180 * speedMultiplier[min(speedRunAttempt, 3)] / 100;
```

Strategy: First run conservative, second run faster, third run maximum speed.
