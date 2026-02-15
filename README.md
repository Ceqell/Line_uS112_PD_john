# Line Following Robot: uS112 PD Controller (John Variation)

## Overview
This code implements a high-performance **PD (Proportional-Derivative) control system** for a line-following robot using the uS112 board. It relies on the QTRSensors library to read a 9-sensor array and adjusts two DC motors to keep the robot centered on a black line.

The "Golden Standard" status refers to the implementation of the **PD Control Loop** combined with **Hardcoded Route Optimization**. Instead of just reacting to the line, this robot "knows" the track (evidenced by the sequence of timers and turns in the `loop` function), allowing it to accelerate blindly on straightaways and brake precisely for turns.

---

## 1. The "Secret Sauce": PD Control Explained
The robot uses `Kp` and `Kd` tuning constants to calculate steering adjustments. The center position of the line is defined as **4000**.

The **Error** is calculated as:  
`error = Position - 4000`

The motor adjustment (`PDValue`) is calculated using this formula:
$$PDValue = (K_p \cdot error) + (K_d \cdot (error - last\_error))$$

### Variables
* **$K_p$ (Proportional): "The Steering Strength"**
    * It looks at *where you are right now*.
    * If the robot is far off-center, $K_p$ creates a large correction.
* **$K_d$ (Derivative): "The Dampener"**
    * It looks at *how fast you are moving away/towards the line* (`error - last_error`).
    * It prevents "overshooting" (wobbling) by fighting the proportional term if the robot is swinging back to the center too fast.

**In this code:**
The function `stdPD` uses these values to adjust the left and right motor speeds:
* `leftSpeed = runSpeed + PDValue`
* `rightSpeed = runSpeed - PDValue`

---

## 2. Hardware Configuration
* **Sensors:** 9-sensor array (QTR-RC sequence) connected to digital pins:
    `19, 18, 17, 16, 15, 14, 11, 10, 9`.
* **Motors:**
    * **Motor 1:** PWM Pin 5, Direction Pins 2 & 3.
    * **Motor 2:** PWM Pin 6, Direction Pins 4 & 7.
* **User Input:** A button/switch on Pin 8 (`SW_PORT`) to start the routine.

---

## 3. Code Structure & Key Functions

### `setup()` & Calibration
* **`initRobot()`**: Sets up motor pins.
* **`setValue()`**: Instead of calibrating every time it turns on, this code uses **hardcoded calibration values** (`minValue` and `maxValue` arrays) stored in global variables. This ensures consistent performance between runs without needing to re-scan the white/black levels every time.

### `loop()` - The Race Track Memory
This is where the robot's "knowledge" of the track lives. It executes a pre-planned sequence of moves optimized for a specific track.
1.  **Wait for Start:** Waits for button press.
2.  **Sequence of Moves:**
    * `runTimer(...)`: Go straight (PD control) for X milliseconds.
    * `STOP`: Brake to prevent flying off.
    * `ChkCross(...)`: Handle an intersection.
    * `turnLeft()` / `turnRight()`: Execute 90-degree turns.

### `stdPD(int runSpeed, float Kp, float Kd)`
This is the atomic unit of the control system.
1.  Reads the line position (`qtrrc.readLine`).
2.  Calculates the PD value.
3.  Adjusts motor speeds.
4.  **Safety Check:** If sensors see all black (`> 600`), it assumes it's on a cross/stop line and forces motors to `runSpeed` (go straight).

### `runTimer(int runSpeed, float Kp, float Kd, int Timer)`
Runs the `stdPD` logic inside a loop for a specific duration (`Timer` in milliseconds). This allows the robot to "blindly" trust the line for a set time (e.g., a long straightaway) where it can use aggressive Kp/Kd settings.

### `ChkCross(int runSpeed, float Kp, float Kd)`
Handles intersections (Crosses).
1.  Reads sensors.
2.  **Logic:** It waits while the outer sensors see white (line is centered), then waits while they see black (crossing the perpendicular line). This ensures the robot physically passes over the intersection before switching to the next command.

### `turnLeft()` / `turnRight()`
These are "blocking" turns:
1.  **Blind Spin:** The robot spins motors in opposite directions (`-100, 100`) for a fixed time (e.g., 120ms) to start the turn.
2.  **Sensor Alignment:** It continues spinning *until* the specific sensor (Sensor 0 for left, Sensor 7 for right) sees the line (`< 600`), ensuring it locks onto the new path perfectly.

---

## 4. How to Use/Tune This Code
1.  **Calibrate:** Uncomment `readTune()` in setup, run it once, copy the Serial Monitor values into the `minValue` and `maxValue` arrays.
2.  **Map the Track:** Modify the `loop()` function to match your track layout.
    * *Long straight?* Add a `runTimer` with high speed.
    * *Sharp curve?* Add a `runTimer` with lower speed and higher `Kd`.
    * *90° Turn?* Call `turnLeft()` or `turnRight()`.
3.  **Tune PID:**
    * Adjust `Kp` (0.037 in code) and `Kd` (0.15 in code) in the `runTimer` calls.
    * If it wobbles, increase `Kd`. If it's too slow to turn, increase `Kp`.

### 5. Addendum; Tuning Guide: Finding the Sweet Spot

This robot relies on balancing two values. When tuning, change only one variable at a time by small increments.

**Proportional Constant ($K_p$) – The "Strength"**
* **Increase ($\uparrow$):** The robot snaps back to the line faster.
    * *Too High:* The robot starts shaking violently (oscillating) even on straight lines.
* **Decrease ($\downarrow$):** The drive becomes smoother and softer.
    * *Too Low:* The robot becomes "lazy" and drifts off the track on sharp curves.

**Derivative Constant ($K_d$) – The "Damper"**
* **Increase ($\uparrow$):** Smooths out the jittering and stabilizes the robot.
    * *Too High:* The robot becomes sluggish; it resists turning and reacts too slowly to sudden corners (over-damped).
* **Decrease ($\downarrow$):** Makes the robot more agile and responsive.
    * *Too Low:* The stability is lost and the robot begins to wobble/overshoot the line again (under-damped).
