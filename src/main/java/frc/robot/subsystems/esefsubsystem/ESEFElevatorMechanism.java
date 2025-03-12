// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Inches;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ESEFElevatorMechanism {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  private final MotionMagicVoltage elevatorMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  
  public static final Distance kElevatorMinHeight = Inches.of(0.0);
  public static final Distance kElevatorMaxHeight = Inches.of(58);

  boolean encoderCalibrated = false;

  // to save a requested position if encoder is not calibrated
  Distance requestedPositionWhileCalibrating = null;

  AnalogInput homeLimitSwitch = new AnalogInput(0);

  int resetCounter = 0;

  TalonFXConfiguration elevatorAConfig = new TalonFXConfiguration();
  TalonFXConfiguration elevatorBConfig = new TalonFXConfiguration();
  public TalonFX elevatorA;
  public TalonFX elevatorB;
  final int ELEVATORA_MOTORID = 9;
  final int ELEVATORB_MOTORID = 10;

  private final Distance positionConversion = Inches.of(9 / (2 * Math.PI));

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
      elevatorAConfigs.Slot0.kP = 1.2; // an error of 1 rotation results in x Volt output
      elevatorAConfigs.Slot0.kI = 0;
      elevatorAConfigs.Slot0.kD = 0;
      elevatorAConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      elevatorAConfigs.MotionMagic.MotionMagicCruiseVelocity = 125; // Max speed in Rotations per second
      elevatorAConfigs.MotionMagic.MotionMagicAcceleration = 100; // Max acceleration in Rotations per second^2
      elevatorAConfigs.MotionMagic.MotionMagicJerk = 200; // Smooth out acceleration (optional)

      elevatorAConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
      elevatorAConfigs.MotorOutput.withPeakForwardDutyCycle(0.6);
      elevatorAConfigs.MotorOutput.withPeakReverseDutyCycle(-0.35);
      elevatorAConfigs.Voltage.withPeakForwardVoltage(12 * 0.6);
      elevatorAConfigs.Voltage.withPeakReverseVoltage(-12 * 0.35);

      elevatorAConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      // elevatorAConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      // elevatorAConfigs.MotorOutput.PeakForwardDutyCycle = 0.05;
      // elevatorAConfigs.MotorOutput.PeakReverseDutyCycle = 0.025;
      // elevatorAConfigs.Voltage.PeakForwardVoltage = (0.05 * 12);
      // elevatorAConfigs.Voltage.PeakReverseVoltage = (0.025 * 12);

      // elevatorA.setPosition(0);

      elevatorA.getConfigurator().apply(elevatorAConfigs); // Applies the Config to the motor

    }

    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, ELEVATORB_MOTORID,
        "Elevator Motor B")
        || RobotContainer.shouldMakeAllCANDevices()) {
      this.elevatorB = new TalonFX(ELEVATORB_MOTORID);
      //TalonFXConfiguration elevatorBConfigs = new TalonFXConfiguration();
      //elevatorBConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
      //elevatorB.getConfigurator().apply(elevatorBConfigs);
      elevatorB.setPosition(0);

      elevatorB.setControl(new Follower(ELEVATORA_MOTORID, false));
    }

  }

  public void periodic() {
    SmartDashboard.putNumber("frc3620/Elevator/HomeSwitchInput", homeLimitSwitch.getVoltage());

    // only do something if we actually have a motor
    if (elevatorA != null && elevatorB != null) {
      if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
        if (!encoderCalibrated) {
          // If the robot is running, and the encoder is "not calibrated," run motor very
          // slowly towards the switch
          elevatorA.set(-0.05);
          if (homeSwitchHit()) {
            // we are home
            encoderCalibrated = true;
            elevatorA.set(0.0);
            elevatorA.setPosition(0);
            setSetpoint(Inches.of(0));

            // If there was a requested position while we were calibrating, go there
            if (requestedPositionWhileCalibrating != null) {
              setSetpoint(requestedPositionWhileCalibrating);
              requestedPositionWhileCalibrating = null;
            }
          }
        }
      }
      if(homeSwitchHit() && getCurrentHeight().in(Inches) > 0.5) {
        logger.info ("reset elevator because of home switch");
        elevatorA.setPosition(0);
        resetCounter++;
        SmartDashboard.putNumber("frc3620/Elevator/ResetCount", ++resetCounter);
        encoderCalibrated = true;
      }
    }

    if (elevatorA != null) {
      SmartDashboard.putNumber("frc3620/Elevator/AMotorAppliedPower", elevatorA.get());
      SmartDashboard.putNumber("frc3620/Elevator/AMotorAppliedCurrent", elevatorA.getStatorCurrent().getValueAsDouble());
    }
    if (elevatorB != null) {
      SmartDashboard.putNumber("frc3620/Elevator/BMotorAppliedPower", elevatorB.get());
      SmartDashboard.putNumber("frc3620/Elevator/BMotorAppliedCurrent", elevatorB.getStatorCurrent().getValueAsDouble());
    }
    SmartDashboard.putBoolean("frc3620/Elevator/HomeLimitSwitchPressed", homeSwitchHit());
    SmartDashboard.putBoolean("frc3620/Elevator/Calibrated", encoderCalibrated);
    SmartDashboard.putNumber("frc3620/Elevator/ActualPosition", getCurrentHeight().in(Inches));
    SmartDashboard.putNumber("frc3620/Elevator/ActualBPosition", getCurrentHeightB().in(Inches));

  }

  public boolean homeSwitchHit() {
    return homeLimitSwitch.getVoltage() > 2.0;
  }

  public void setSetpoint(Distance position) {
    SmartDashboard.putNumber("frc3620/Elevator/RequestedPosition", position.in(Inches));

    double motorRotations = position.in(Inches) / positionConversion.in(Inches); // Convert inches to rotations
    motorRotations = MathUtil.clamp(motorRotations, 0, 40.2);  // how was this magic number determined?????

    if (elevatorA != null && encoderCalibrated) {
      elevatorA.setControl(elevatorMotionMagicRequest.withPosition(motorRotations));    }
  }

  public Distance getCurrentHeight() {
    if (elevatorA != null) {
      return Inches.of(elevatorA.getPosition().getValueAsDouble() * positionConversion.in(Inches));
    } else {
      return Inches.of(0);
    }
  }

  public Distance getCurrentHeightB() {
    if (elevatorB != null) {
      return Inches.of(elevatorB.getPosition().getValueAsDouble() * positionConversion.in(Inches));
    } else {
      return Inches.of(0);
    }
  }

}
