# 11 — Data Logging, Telemetry & Visualization
## Real-Time Debugging and Post-Run Analysis

---

## Why Data Logging?

Without data logging, debugging a micromouse is **guesswork**. With it, you can:
- See exactly why the robot turned too far or too short
- Identify sensor noise vs real walls
- Tune PID gains based on actual response curves
- Diagnose I2C failures and timing issues
- Compare exploration vs speed run performance

---

## Method 1: Serial Logging (Simplest)

### Structured CSV Format

```cpp
// Add this to your main loop for structured logging
// Format: timestamp,state,x,y,heading,front,left,right,rpmL,rpmR,pwmL,pwmR,yaw,battery

void logDataCSV() {
  Serial.print(millis()); Serial.print(",");
  Serial.print(state); Serial.print(",");
  Serial.print(robotX); Serial.print(",");
  Serial.print(robotY); Serial.print(",");
  Serial.print(robotHeading); Serial.print(",");
  
  SensorData s = readSensors();
  Serial.print(s.front); Serial.print(",");
  Serial.print(s.left); Serial.print(",");
  Serial.print(s.right); Serial.print(",");
  
  // RPM (calculated from encoder delta)
  static long prevEncL = 0, prevEncR = 0;
  static unsigned long prevTime = 0;
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  if (dt > 0) {
    float rpmL = ((encoderL - prevEncL) / ENCODER_CPR) * (60.0 / dt);
    float rpmR = ((encoderR - prevEncR) / ENCODER_CPR) * (60.0 / dt);
    Serial.print(rpmL, 1); Serial.print(",");
    Serial.print(rpmR, 1); Serial.print(",");
  }
  prevEncL = encoderL;
  prevEncR = encoderR;
  prevTime = now;
  
  Serial.print(pwmL, 0); Serial.print(",");
  Serial.print(pwmR, 0); Serial.print(",");
  
  // Yaw
  Serial.print(yawRef, 2); Serial.print(",");
  
  // Battery
  Serial.println(readBatteryVoltage(), 2);
}

// Call in loop():
// logDataCSV();  // ~50Hz logging rate
```

### Capturing Serial Data:

```
1. Open Arduino Serial Monitor → set baud rate to 115200
2. Copy/paste output into a .csv file
3. Or use a serial terminal that saves to file:
   - CoolTerm (Windows/Mac)
   - PuTTY (set session logging to "All session output")
   - Screen (Linux): screen -L /dev/ttyUSB0 115200
```

---

## Method 2: Bluetooth Telemetry (HC-05)

### Wiring:

```
HC-05 Module         STM32
──────────           ─────
VCC    ──────────→   5V (NOT 3.3V — HC-05 needs 5V)
GND    ──────────→   GND
TXD    ──────────→   PA3 (or any UART RX pin)
RXD    ──────────→   PA2 (or any UART TX pin) via voltage divider*

* HC-05 RXD is 3.3V tolerant on most modules, but check yours.
  If not: use a 1kΩ + 2kΩ voltage divider on the TX line.
```

> [!WARNING]
> PA2 and PA3 are used for right encoder in this project! If using Bluetooth, remap encoders to different pins (e.g., PB3, PB10).

### Bluetooth Serial Code:

```cpp
// Use Serial2 (or HardwareSerial) for Bluetooth
// Serial1 for USB debug, Serial2 for Bluetooth

HardwareSerial Serial2(PA3, PA2);  // RX, TX

void setup() {
  Serial.begin(115200);   // USB debug
  Serial2.begin(9600);    // Bluetooth (HC-05 default baud rate)
  Serial2.println("Micromouse Bluetooth Connected!");
}

void sendTelemetry() {
  // Send compact telemetry over Bluetooth
  Serial2.print("$");  // Start marker
  Serial2.print(millis()); Serial2.print(",");
  Serial2.print(robotX); Serial2.print(",");
  Serial2.print(robotY); Serial2.print(",");
  Serial2.print(robotHeading); Serial2.print(",");
  
  SensorData s = readSensors();
  Serial2.print(s.front); Serial2.print(",");
  Serial2.print(s.left); Serial2.print(",");
  Serial2.print(s.right); Serial2.print(",");
  Serial2.print(readBatteryVoltage(), 1);
  Serial2.println("#");  // End marker
}

// Remote control commands (receive from phone/laptop):
void checkBluetoothCommands() {
  if (Serial2.available()) {
    char cmd = Serial2.read();
    switch (cmd) {
      case 'S': state = EXPLORING; break;   // Start
      case 'X': state = DONE; break;        // Emergency stop
      case 'R': state = SPEED_RUN; break;   // Speed run
      case 'D': printMazeMap(); break;       // Dump maze
      case 'C': calibrateGyro(); break;      // Calibrate
    }
  }
}
```

### Connecting from Laptop/Phone:
```
1. Pair with HC-05 (default PIN: 1234 or 0000)
2. Use a Bluetooth serial terminal:
   - Android: "Serial Bluetooth Terminal" app (free)
   - Windows: Connect via COM port + PuTTY
   - Python: Use pyserial with Bluetooth COM port
```

---

## Method 3: Python Visualization Scripts

### Install Requirements:
```bash
pip install pyserial matplotlib numpy
```

### Real-Time PID Plotter:

```python
#!/usr/bin/env python3
"""Real-time PID response plotter for Micromouse"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# Configuration
PORT = "COM3"  # Change to your port
BAUD = 115200
MAX_POINTS = 500

# Data buffers
times = deque(maxlen=MAX_POINTS)
rpm_left = deque(maxlen=MAX_POINTS)
rpm_right = deque(maxlen=MAX_POINTS)
pwm_left = deque(maxlen=MAX_POINTS)
pwm_right = deque(maxlen=MAX_POINTS)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle("Micromouse PID Telemetry", fontsize=14)

ser = serial.Serial(PORT, BAUD, timeout=1)

def parse_line(line):
    """Parse CSV: timestamp,state,x,y,heading,front,left,right,rpmL,rpmR,pwmL,pwmR,yaw,battery"""
    try:
        parts = line.strip().split(",")
        if len(parts) >= 12:
            return {
                "time": int(parts[0]) / 1000.0,
                "rpmL": float(parts[8]),
                "rpmR": float(parts[9]),
                "pwmL": float(parts[10]),
                "pwmR": float(parts[11]),
            }
    except (ValueError, IndexError):
        pass
    return None

def update(frame):
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8", errors="ignore")
            data = parse_line(line)
            if data:
                times.append(data["time"])
                rpm_left.append(data["rpmL"])
                rpm_right.append(data["rpmR"])
                pwm_left.append(data["pwmL"])
                pwm_right.append(data["pwmR"])
        except Exception:
            pass
    
    if len(times) > 1:
        ax1.clear()
        ax1.plot(list(times), list(rpm_left), 'b-', label='RPM Left')
        ax1.plot(list(times), list(rpm_right), 'r-', label='RPM Right')
        ax1.set_ylabel("RPM")
        ax1.legend(loc='upper right')
        ax1.set_title("Motor Speed")
        ax1.grid(True, alpha=0.3)
        
        ax2.clear()
        ax2.plot(list(times), list(pwm_left), 'b-', label='PWM Left')
        ax2.plot(list(times), list(pwm_right), 'r-', label='PWM Right')
        ax2.set_ylabel("PWM (0-255)")
        ax2.set_xlabel("Time (s)")
        ax2.legend(loc='upper right')
        ax2.set_title("Motor PWM Output")
        ax2.grid(True, alpha=0.3)

ani = animation.FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.show()
ser.close()
```

### Maze Map Visualizer:

```python
#!/usr/bin/env python3
"""Visualize the maze map after exploration"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

MAZE_SIZE = 16
CELL_SIZE = 1  # Unit cell size for drawing

def parse_maze_data(filename):
    """Parse wall data from serial log file.
    Expected format: one line per cell, 'x,y,walls' where walls is a bitmask."""
    walls = np.zeros((MAZE_SIZE, MAZE_SIZE), dtype=int)
    
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("WALL:"):
                parts = line.replace("WALL:", "").split(",")
                x, y, w = int(parts[0]), int(parts[1]), int(parts[2])
                walls[x][y] = w
    
    return walls

def draw_maze(walls, path=None, title="Micromouse Maze"):
    """Draw the maze with walls and optional path overlay."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.set_xlim(-0.5, MAZE_SIZE - 0.5)
    ax.set_ylim(-0.5, MAZE_SIZE - 0.5)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=16)
    
    # Draw cells
    for x in range(MAZE_SIZE):
        for y in range(MAZE_SIZE):
            # Cell background
            color = '#f0f0f0'
            if x == 0 and y == 0:
                color = '#90EE90'  # Start = green
            elif (x == 7 or x == 8) and (y == 7 or y == 8):
                color = '#FFB6C1'  # Goal = pink
            
            rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                     linewidth=0.5, edgecolor='lightgray',
                                     facecolor=color)
            ax.add_patch(rect)
            
            # Draw walls
            w = walls[x][y]
            wall_style = {'color': 'black', 'linewidth': 2}
            
            if w & 1:  # North wall
                ax.plot([x - 0.5, x + 0.5], [y + 0.5, y + 0.5], **wall_style)
            if w & 2:  # East wall
                ax.plot([x + 0.5, x + 0.5], [y - 0.5, y + 0.5], **wall_style)
            if w & 4:  # South wall
                ax.plot([x - 0.5, x + 0.5], [y - 0.5, y - 0.5], **wall_style)
            if w & 8:  # West wall
                ax.plot([x - 0.5, x - 0.5], [y - 0.5, y + 0.5], **wall_style)
    
    # Draw path if provided
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, 'b-o', linewidth=2, markersize=5, alpha=0.7)
    
    # Labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    
    # Draw border
    ax.plot([-0.5, MAZE_SIZE - 0.5], [-0.5, -0.5], 'k-', linewidth=3)
    ax.plot([-0.5, MAZE_SIZE - 0.5], [MAZE_SIZE - 0.5, MAZE_SIZE - 0.5], 'k-', linewidth=3)
    ax.plot([-0.5, -0.5], [-0.5, MAZE_SIZE - 0.5], 'k-', linewidth=3)
    ax.plot([MAZE_SIZE - 0.5, MAZE_SIZE - 0.5], [-0.5, MAZE_SIZE - 0.5], 'k-', linewidth=3)
    
    plt.tight_layout()
    plt.savefig("maze_map.png", dpi=150)
    plt.show()

# Usage:
# walls = parse_maze_data("serial_log.txt")
# draw_maze(walls, title="Explored Maze")
```

---

## Adding Maze Dump to Firmware

Add this function to your main code so you can dump the maze map over serial:

```cpp
void printMazeMap() {
  Serial.println("=== MAZE MAP DUMP ===");
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (walls[x][y] > 0 || visited[x][y]) {
        Serial.print("WALL:");
        Serial.print(x); Serial.print(",");
        Serial.print(y); Serial.print(",");
        Serial.println(walls[x][y]);
      }
    }
  }
  Serial.println("=== END MAZE MAP ===");
  
  // Also print flood values as grid
  Serial.println("\nFlood values:");
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (floodValues[x][y] < 10) Serial.print(" ");
      if (floodValues[x][y] < 100) Serial.print(" ");
      Serial.print(floodValues[x][y]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
```

---

## Sensor Data Recording for Analysis

### Log sensor readings during a run for post-analysis:

```cpp
// Circular buffer for storing sensor history
#define LOG_SIZE 500  // ~10 seconds at 50Hz

struct LogEntry {
  unsigned long timestamp;
  int front, left, right;
  float rpmL, rpmR;
  float yaw;
  int pwmL, pwmR;
  uint8_t cellX, cellY;
};

LogEntry dataLog[LOG_SIZE];
int logIndex = 0;
bool logFull = false;

void addLogEntry() {
  SensorData s = readSensors();
  
  dataLog[logIndex].timestamp = millis();
  dataLog[logIndex].front = s.front;
  dataLog[logIndex].left = s.left;
  dataLog[logIndex].right = s.right;
  dataLog[logIndex].yaw = yawRef;
  dataLog[logIndex].pwmL = (int)pwmL;
  dataLog[logIndex].pwmR = (int)pwmR;
  dataLog[logIndex].cellX = robotX;
  dataLog[logIndex].cellY = robotY;
  
  logIndex = (logIndex + 1) % LOG_SIZE;
  if (logIndex == 0) logFull = true;
}

void dumpLog() {
  Serial.println("timestamp,front,left,right,rpmL,rpmR,yaw,pwmL,pwmR,cellX,cellY");
  
  int start = logFull ? logIndex : 0;
  int count = logFull ? LOG_SIZE : logIndex;
  
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % LOG_SIZE;
    Serial.print(dataLog[idx].timestamp); Serial.print(",");
    Serial.print(dataLog[idx].front); Serial.print(",");
    Serial.print(dataLog[idx].left); Serial.print(",");
    Serial.print(dataLog[idx].right); Serial.print(",");
    Serial.print(dataLog[idx].rpmL, 1); Serial.print(",");
    Serial.print(dataLog[idx].rpmR, 1); Serial.print(",");
    Serial.print(dataLog[idx].yaw, 2); Serial.print(",");
    Serial.print(dataLog[idx].pwmL); Serial.print(",");
    Serial.print(dataLog[idx].pwmR); Serial.print(",");
    Serial.print(dataLog[idx].cellX); Serial.print(",");
    Serial.println(dataLog[idx].cellY);
  }
  
  Serial.print("Total entries: "); Serial.println(count);
}

// Call addLogEntry() in your main loop
// Call dumpLog() after a run completes (or via Bluetooth command 'D')
```

---

## Quick Reference: Logging Commands

If you implement the Bluetooth command handler, use these single-character commands:

| Command | Action |
|---------|--------|
| `S` | Start exploration |
| `X` | Emergency stop |
| `R` | Start speed run |
| `D` | Dump maze map |
| `L` | Dump data log |
| `C` | Calibrate gyro |
| `B` | Report battery voltage |
| `M` | Print current maze state |
| `T` | Run PID auto-tune |
| `N` | Clear saved maze |
