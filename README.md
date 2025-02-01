# IO Assignments

## Digital IO
* DIO 0: Practice Robot Jumper

## Analog IO

## PWM

## Motor Controllers
* TalonFX 9: ESEF End Effector Motor
* TalonFX 10: ESEF Shoulder Motor
* TalonFX 11: ESEF Elevator Motor A
* TalonFX 12: ESEF Elevator Motor B


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