// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AFISubsystem extends SubsystemBase {
  /** Creates a new AFISubsystem. */
  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  public TalonFX pivot;
  public DutyCycleEncoder frontEncoder;
  public DutyCycleEncoder rearEncoder;
  Angle rearEncoderOffset;
  Angle frontEncoderOffset;

  enum WhichEncoderToUse {
    FRONT, REAR
  }
//Front Encoder is backwards use rear until fixed.
  WhichEncoderToUse whichEncoderToUse = WhichEncoderToUse.REAR;

  boolean relativeEncoderSet = false;
  Timer relativeEncoderTimer = new Timer();

  final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
  final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  public TalonFX roller;

  final int AFIPIVOTMOTORID = 14;
  final int AFIROLLERMOTORID = 15;

  PIDController pid;

  // this is the ratio of motor rotations to intake arm rotations
  final static double MOTOR_TO_INTAKE_RATIO = 5 * 5 * (3.0 / 2.0);

  public AFISubsystem() {
    // constructor
    frontEncoder = new DutyCycleEncoder(5);
    rearEncoder = new DutyCycleEncoder(6);
    frontEncoderOffset = Degrees.of(RobotContainer.robotParameters.getIntakeFrontEncoderOffset());
    rearEncoderOffset = Degrees.of(RobotContainer.robotParameters.getIntakeRearEncoderOffset());

    pid = new PIDController(
      2,
      0,
      0
    );

    SmartDashboard.putString("frc3620/AFI/WhichAbsoluteEncoder", whichEncoderToUse.toString());
    relativeEncoderTimer.reset();
    relativeEncoderTimer.start();

    // Pivot
    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIPIVOTMOTORID, "AFIPivot")
        || RobotContainer.shouldMakeAllCANDevices()) {
      this.pivot = new TalonFX(AFIPIVOTMOTORID);
      // this.shoulderEncoder = new CANcoder(10);
     TalonFXConfiguration configs = new TalonFXConfiguration();

            //configs.Slot0.kG = 0.0; // Gravity FeedForward
            //configs.Slot0.kS = 0; // Friction FeedForward
            //configs.Slot0.kP = 1; // an error of 1 rotation results in x Volt output
            //configs.Slot0.kI = 0;
            //configs.Slot0.kD = 0;

           // configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

            //configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            //configs.MotorOutput.withPeakForwardDutyCycle(0.1);
            //configs.MotorOutput.withPeakReverseDutyCycle(-0.1);
            //configs.Voltage.withPeakForwardVoltage(12 * 0.1);
            //configs.Voltage.withPeakReverseVoltage(12 * -0.1);
    } // Applies the Config to the shoulder motor

    // Roller
    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIROLLERMOTORID, "AFIRoller")
        || RobotContainer.shouldMakeAllCANDevices()) {
      this.roller = new TalonFX(AFIROLLERMOTORID);
    }

  }

  @Override
  public void periodic() {
    if (!relativeEncoderSet) {
      if (relativeEncoderTimer.hasElapsed(5)) {
        if (isEncoderConnected()) {
          logger.info("AFI absolute angle is {}", getAbsoluteIntakeAngle().in(Degrees));
          logger.info("AFI relative angle is before {}", pivot.getPosition().getValue().in(Degrees));
          logger.info("setting the posistion {}", getAbsoluteIntakeAngle().times(MOTOR_TO_INTAKE_RATIO).in(Degrees));
          pivot.setPosition(getAbsoluteIntakeAngle().times(MOTOR_TO_INTAKE_RATIO));
          logger.info("AFI relative angle is after {}", pivot.getPosition().getValue().in(Degrees));

          relativeEncoderSet = true;

        }
      }
    }

    double output = pid.calculate(getAbsoluteIntakeAngle().in(Rotations));
    pivot.set(MathUtil.clamp(output, -0.1, 0.1));

    SmartDashboard.putNumber("frc3620/AFI/pivotOutput", output);

    SmartDashboard.putNumber("frc3620/AFI/PivotFrontAbsolutePositionRaw", Rotations.of(frontEncoder.get()).in(Degrees));
    SmartDashboard.putNumber("frc3620/AFI/PivotRearAbsolutePositionRaw", Rotations.of(rearEncoder.get()).in(Degrees));

    // logger.info("AFI absolute angle is {}",
    // getAbsoluteIntakeAngle().in(Degrees));

    // This method will be called once per scheduler run
    if (pivot != null) {
      SmartDashboard.putNumber("frc3620/AFI/PivotFrontAbsolutePosition", getFrontAbsoluteIntakeAngle().in(Degrees));
      SmartDashboard.putNumber("frc3620/AFI/PivotRearAbsolutePosition", getRearAbsoluteIntakeAngle().in(Degrees));
      SmartDashboard.putNumber("frc3620/AFI/PivotRelativePosition",
          pivot.getPosition().getValue().div(MOTOR_TO_INTAKE_RATIO).in(Degrees));
      SmartDashboard.putNumber("frc3620/AFI/MotorVoltage", pivot.getMotorVoltage().getValueAsDouble());
    }
  }

  public void setPivotPosition(Angle position) {
    // set the shoulder to the desired position Cat
    SmartDashboard.putNumber("frc3620/AFI/PivotRequestedPosition", position.in(Degrees));

    if (pivot != null) {
      //pivot.setControl(pivotRequest.withPosition(position.times(MOTOR_TO_INTAKE_RATIO)));
      pid.setSetpoint(position.in(Rotations));
    }
  }

  boolean isEncoderConnected() {
    if (whichEncoderToUse == WhichEncoderToUse.FRONT) {
      return frontEncoder.isConnected();
    } else {
      return rearEncoder.isConnected();
    }
  }

  Angle getFrontAbsoluteIntakeAngle() {
    return Rotations.of(frontEncoder.get()).minus(frontEncoderOffset);
  }

  Angle getRearAbsoluteIntakeAngle() {
    return Rotations.of(rearEncoder.get()).minus(rearEncoderOffset);
  }

  public Angle getAbsoluteIntakeAngle() {
    if (whichEncoderToUse == WhichEncoderToUse.FRONT) {
      return getFrontAbsoluteIntakeAngle();
    } else {
      return getRearAbsoluteIntakeAngle();
    }

  }

  public void setAFIRollerSpeed(double speed) {
    roller.setControl(rollerRequest.withOutput(speed));
  }
}
