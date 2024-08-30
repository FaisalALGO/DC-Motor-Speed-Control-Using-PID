# DC-Motor-Speed-Control-Using-PID

## Project Overview

This project demonstrates a PID-controlled DC motor speed regulation system using an Arduino Uno (ATmega328P) microcontroller. The goal of the project is to accurately spin a motor at four distinct speeds—30 RPM, 60 RPM, 180 RPM, and 400 RPM—and cycle through these speeds in a specific order. The motor speed is held at each setpoint for 5 seconds, with a PID controller ensuring the motor reaches and maintains the target speed.

## Goal

The program spins the motor at the following speeds in sequence:
30 RPM
60 RPM
180 RPM
400 RPM
Then, it cycles back down through the speeds: 180, 60, 30 RPM.
At each speed, the motor will hold for 5 seconds before switching to the next speed. The PID controller is used to adjust the motor speed dynamically, ensuring the motor reaches the desired RPM and stabilizes at each setpoint.

## Components Used
Arduino Uno (ATmega328P): Microcontroller board that implements the PID control algorithm and drives the motor through the L298N motor driver.
Pololu 37D Gearmotor with Encoder [Product Link](https://www.pololu.com/product/1442): The motor used in this project is the Pololu 37D gearmotor, which features an encoder for precise feedback on motor speed. This feedback is critical for the PID control to function properly.
L298N Motor Driver: This motor driver board controls the motor's speed and direction based on signals from the Arduino Uno.
Power Supply: The motor requires a stable external power source, separate from the Arduino's supply.(12V)

## System Design
The PID (Proportional-Integral-Derivative) controller is designed to regulate the speed of the motor by adjusting the PWM signal sent to the motor driver. The encoder on the motor provides real-time feedback on the current RPM, which allows the Arduino to calculate the error between the desired and actual speed. The PID controller then adjusts the motor's input to reduce this error.

## Features
Real-time speed control using PID algorithm.
Encoder feedback for precise RPM measurements.
Cyclic speed transitions between 30 RPM, 60 RPM, 180 RPM, and 400 RPM.
Maintains each speed for 5 seconds before transitioning to the next speed.
PID Tuning
The PID controller was tuned to achieve smooth and stable motor operation across all RPM setpoints. It minimizes overshoot and settling time, ensuring accurate control over motor speed.

## Wiring Diagram
<table>
  <tr>
    <td>
      <img src="wiring_diagram.jpg" alt="Wiring Diagram" width="400" height="300" />
    </td>
    <td style="vertical-align: top; padding-left: 20px;">
      <p><strong>Connections</strong></p>
      <pre>
Encoder output A --> PD3     
Encoder output B --> PD2  
-----------------------------
PD6 --> L298n Driver Input 1 (IN1)
PD5 --> L298n Driver Input 2 (IN2)
-----------------------------------
L298n Driver 5V output --> Vcc encoder
L298n Driver output 1 (OUT1) --> motor power terminal (RED)
L298n Driver output 2 (OUT2) --> motor power terminal (BLACK)
      </pre>
    </td>
  </tr>
</table>

## How It Works
The Arduino Uno continuously reads the encoder values to determine the motor's current speed.
The PID algorithm calculates the error between the desired RPM and the actual RPM.
The motor driver adjusts the voltage to the motor, correcting the speed to match the target RPM.
The program cycles through the speed setpoints in the following sequence: 30 → 60 → 180 → 400 → 180 → 60 → 30.
At each speed, the motor holds for 5 seconds before moving to the next target.

##  Usage
To use this project, upload the provided code to your Arduino Uno. Ensure all components are connected as per the wiring diagram, and provide adequate power to the motor driver.

## Demo Video

https://github.com/user-attachments/assets/681f1b5e-a0ca-43f8-aca1-042f6f05c41c



