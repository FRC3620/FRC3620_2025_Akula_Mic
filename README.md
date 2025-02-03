# IO Assignments

## Digital IO
* DIO 0: Practice Robot Jumper
* DIO 1: FL Absolute Encoder
* DIO 2: FR Absolute Encoder
* DIO 3: RL Absolute Encoder
* DIO 4: RR Absolute Encoder
* DIO 5: Intake Front Encoder
* DIO 6: Intake Rear Encoder
* DIO 7: Elevator Bottom Limit Switch
* DIO 8: Climber Encoder

## Analog IO

## PWM

## Motor Controllers
* 1-8: Drive
* TalonFX 9 (was 11): ESEF Elevator Motor A
* TalonFX 10 (was 12): ESEF Elevator Motor B
* TalonFX 11 (was 10): ESEF Shoulder Motor
* TalonFX 12 (was 9): ESEF End Effector Motor
* TalonFX 13 (was 15): Climber
* TalonFX 14 (was 13): AFI Pivot Motor
* SparkMax 15 (was 14): AFI roller Motor 

# PDB assignments

# Vision

## Joehan limelight locations

### limelight-front:

* LL UP: 0.29845 m
* LL FRONT: 0.33655 m
* LL PITCH: 25

### limelight-back:

* LL UP: 0.3429 m
* LL FRONT: -0.33655 m
* LL PITCH: 25
* LL YAW: 180

# Driver Controller

# Programming Requirnements
* Mechanisms
  * Eleveator 
    * 2 stage - 2 Krakens.
    * Motors linked by belts.
    * Hard stop/limit switch
  * Shoulder+arm(Algae+Coral)
    * 1 Kraken for the Shoulder(restraints to not break robot)(Shoulder)
    * 1 Neo for the the belts to hold algae(Arm)
    * Absolute encoder on the pivot
    * Sensor on arm to identify coral
    * Manual controls
  * Climber
    * 1 motor back and forth(Like ground intake) 
    * Absolute Encoder
    * both manual and set position
    * Algae ground intake(Not huge priority)
    * 1 kraken swinging up and down(pivot)
    * 1 550 spining wheels
    * Absolute encoders
    * Hardstop back
    * set positions
* Cameras
    * 1 on coral pickup side
    * 1 on desposit side
    * Has to be very precise
    * How do we manage to keep it consistent in the long-term?       