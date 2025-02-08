// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Volts;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.RobotMode;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFElevatorMechanism {

    

    boolean encoderCalibrated = false;
    Timer calibrationTimer;
    // to save a requested position if encoder is not calibrated
    Double requestedPositionWhileCalibrating = null;

    DigitalInput homeLimitSwitch = new DigitalInput(7);

    TalonFXConfiguration elevatorAConfig = new TalonFXConfiguration();
    TalonFXConfiguration elevatorBConfig = new TalonFXConfiguration();
    public TalonFX elevatorA;
    public TalonFX elevatorB;
    final int ELEVATORA_MOTORID = 9;
    final int ELEVATORB_MOTORID = 10;

    private final double positionConversion = 9/(2 * Math.PI);

    final PositionVoltage elevatorARequest = new PositionVoltage(0).withSlot(0); // check if update frequency to
                                                                                 // follower needs to be updated
    // update frequency will also affect PID

    // Add CANCoders

    public ESEFElevatorMechanism() { // constructor

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ELEVATORA_MOTORID,
                "Elevator Motor A")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.elevatorA = new TalonFX(ELEVATORA_MOTORID);
            // this.shoulderEncoder = new CANcoder(10);
            TalonFXConfiguration elevatorAConfigs = new TalonFXConfiguration();
            elevatorAConfigs.Slot0.kG = 0.3; // Gravity FeedForward
            elevatorAConfigs.Slot0.kS = 0; // Friction FeedForward
            elevatorAConfigs.Slot0.kP = 1.0; // an error of 1 rotation results in x Volt output
            elevatorAConfigs.Slot0.kI = 0;
            elevatorAConfigs.Slot0.kD = 0;
            elevatorAConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

            elevatorAConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            elevatorAConfigs.MotorOutput.withPeakForwardDutyCycle(0.1);
            elevatorAConfigs.MotorOutput.withPeakReverseDutyCycle(-0.025);
            elevatorAConfigs.Voltage.withPeakForwardVoltage(12 * 0.1);
            elevatorAConfigs.Voltage.withPeakReverseVoltage(-12 * 0.025);

            //elevatorAConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            //elevatorAConfigs.MotorOutput.PeakForwardDutyCycle = 0.05;
            //elevatorAConfigs.MotorOutput.PeakReverseDutyCycle = 0.025;
            //elevatorAConfigs.Voltage.PeakForwardVoltage = (0.05 * 12);
            //elevatorAConfigs.Voltage.PeakReverseVoltage = (0.025 * 12);
            
            //elevatorA.setPosition(0);
            
            elevatorA.getConfigurator().apply(elevatorAConfigs); // Applies the Config to the motor

        }

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ELEVATORB_MOTORID,
                "Elevator Motor B")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.elevatorB = new TalonFX(ELEVATORB_MOTORID);
            elevatorB.setPosition(0);

            elevatorB.setControl(new Follower(ELEVATORA_MOTORID, false));
        }

    }

    public void periodic() {

        // only do something if we actually have a motor
    if (elevatorA != null && elevatorB != null) {
        if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
          if (!encoderCalibrated) {
            // If the robot is running, and the encoder is "not calibrated," run motor very
            // slowly towards the switch
            elevatorA.set(-0.01); 
              // we have a timer, has the motor had power long enough to spin up
                if (!homeLimitSwitch.get()) {
                  // motor is not moving, hopefully it's against the stop
                  encoderCalibrated = true;
                  elevatorA.set(0.0);
                  elevatorA.setPosition(0);
                  setElevatorPosition(0);

                  // If there was a requested position while we were calibrating, go there
                  if (requestedPositionWhileCalibrating != null) {
                    setElevatorPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }
            }
          }
        }
      }

        if (elevatorA != null) {
            SmartDashboard.putNumber("frc3620/Elevator/AMotorActualPosition",
                    elevatorA.getPosition().getValueAsDouble() * positionConversion);
            SmartDashboard.putNumber("frc3620/Elevator/AMotorAppliedOutput", elevatorA.get());
        }
        if (elevatorB != null) {
            SmartDashboard.putNumber("frc3620/Elevator/BMotorActualPosition",
                    elevatorB.getPosition().getValueAsDouble() * positionConversion);
            SmartDashboard.putNumber("frc3620/Elevator/BMotorAppliedOutput", elevatorB.get());

        }
        SmartDashboard.putBoolean("frc3620/Elevator/HomeLimitSwitchPressed", !homeLimitSwitch.get());
        SmartDashboard.putBoolean("frc3620/Elevator/Calibrated", encoderCalibrated);
        
    }

    public void setElevatorPosition(double position) {

        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/Elevator/RequestedPosition", position);

        position = position / positionConversion;
        MathUtil.clamp(position, 0, 35);
        
        if (elevatorA != null) {
            elevatorA.setControl(elevatorARequest.withPosition(position));
        }
    }

    public double getElevatorPosition() {
        return elevatorA.getPosition().getValueAsDouble() * positionConversion;
    }

    

}
