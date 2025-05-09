// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

  public double measuredRollerSpeed;
  //Front Encoder is backwards use rear until fixed.
  WhichEncoderToUse whichEncoderToUse = WhichEncoderToUse.REAR;
  
  final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
  final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  public TalonFX roller;

  final int AFIPIVOTMOTORID = 14;
  final int AFIROLLERMOTORID = 15;

  final Angle AFIMINPOSITION = Degrees.of(0);
  final Angle AFIMAXPOSITION = Degrees.of(85);

  PIDController pid;

  // this is the ratio of motor rotations to intake arm rotations
  final static double MOTOR_TO_INTAKE_RATIO = 5 * 5 * (3.0 / 2.0);

  final double ffg = 0.02;

  public AFISubsystem() {
    // constructor
    frontEncoder = new DutyCycleEncoder(5);
    rearEncoder = new DutyCycleEncoder(6);
    frontEncoder.setInverted(true);
    frontEncoderOffset = Degrees.of(RobotContainer.robotParameters.getIntakeFrontEncoderOffset());
    rearEncoderOffset = Degrees.of(RobotContainer.robotParameters.getIntakeRearEncoderOffset());

    pid = new PIDController(
        0.75,
        0,
        0);

    SmartDashboard.putString("frc3620/AFI/WhichAbsoluteEncoder", whichEncoderToUse.toString());

    // Pivot
    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIPIVOTMOTORID, "AFIPivot")
        || RobotContainer.shouldMakeAllCANDevices()) {
      this.pivot = new TalonFX(AFIPIVOTMOTORID);

      //TalonFXConfiguration configs = new TalonFXConfiguration();

      // configs.Slot0.kG = 0.0; // Gravity FeedForward
      // configs.Slot0.kS = 0; // Friction FeedForward
      // configs.Slot0.kP = 1; // an error of 1 rotation results in x Volt output
      // configs.Slot0.kI = 0;
      // configs.Slot0.kD = 0;

      // configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      // configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
      // configs.MotorOutput.withPeakForwardDutyCycle(0.1);
      // configs.MotorOutput.withPeakReverseDutyCycle(-0.1);
      // configs.Voltage.withPeakForwardVoltage(12 * 0.1);
      // configs.Voltage.withPeakReverseVoltage(12 * -0.1);
    } // Applies the Config to the shoulder motor

    // Roller
    if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIROLLERMOTORID, "AFIRoller")
        || RobotContainer.shouldMakeAllCANDevices()) {
      this.roller = new TalonFX(AFIROLLERMOTORID);

      CurrentLimitsConfigs afiRollerLimit = new CurrentLimitsConfigs();
      afiRollerLimit.SupplyCurrentLimit = 20;
      afiRollerLimit.SupplyCurrentLimitEnable = true;

      roller.getConfigurator().apply(afiRollerLimit);
      roller.setNeutralMode(NeutralModeValue.Brake);
    }

    setPivotPosition(Degrees.of(85));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("frc3620/AFI/PivotFrontAbsolutePosition", getFrontAbsoluteIntakeAngle().in(Degrees));
    SmartDashboard.putNumber("frc3620/AFI/PivotRearAbsolutePosition", getRearAbsoluteIntakeAngle().in(Degrees));

  
    if (pivot != null) {
      double ffoutput = ffg * Math.cos(getAbsoluteIntakeAngle().in(Radian));
      double pidoutput = pid.calculate(getAbsoluteIntakeAngle().in(Rotations));
      double motorOutput = MathUtil.clamp(pidoutput + ffoutput, -0.5, 0.2);
      pivot.set(motorOutput);

      SmartDashboard.putNumber("frc3620/AFI/pivotpidOutput", pidoutput);
      SmartDashboard.putNumber("frc3620/AFI/pivotffOutput", ffoutput);
      SmartDashboard.putNumber("frc3620/AFI/pivotOutput", motorOutput);
    }
    
    if (roller != null) {
      measuredRollerSpeed = roller.getVelocity().getValue().in(RPM);
      SmartDashboard.putNumber("frc3620/AFI/measureRollerVelocity", measuredRollerSpeed);
    }

    SmartDashboard.putNumber("frc3620/AFI/PivotFrontAbsolutePositionRaw", Rotations.of(frontEncoder.get()).in(Degrees));
    SmartDashboard.putNumber("frc3620/AFI/PivotRearAbsolutePositionRaw", Rotations.of(rearEncoder.get()).in(Degrees));

    if (pivot != null) {
      SmartDashboard.putNumber("frc3620/AFI/PivotRelativePosition",
          pivot.getPosition().getValue().div(MOTOR_TO_INTAKE_RATIO).in(Degrees));
      SmartDashboard.putNumber("frc3620/AFI/PivotMotorVoltage", pivot.getMotorVoltage().getValueAsDouble());
    }
  }

  public void setPivotPosition(Angle position) {
    // set the shoulder to the desired position Cat
    SmartDashboard.putNumber("frc3620/AFI/PivotRequestedPosition", position.in(Degrees));

    if (pivot != null) {
      // pivot.setControl(pivotRequest.withPosition(position.times(MOTOR_TO_INTAKE_RATIO)));
      pid.setSetpoint(Utilities.clamp(position, AFIMINPOSITION, AFIMAXPOSITION).in(Rotations));
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
    SmartDashboard.putNumber("frc3620/AFI/RollerRequestedPower", speed);
    if (roller != null) {
      roller.setControl(rollerRequest.withOutput(speed));
    }
  }
}
