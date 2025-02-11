// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AFISubsystem extends SubsystemBase {
  /** Creates a new AFISubsystem. */
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    public TalonFX pivot;
    public DutyCycleEncoder frontEncoder;
    public DutyCycleEncoder rearEncoder;

    final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
    final DutyCycleOut rollerRequest = new DutyCycleOut(0);


    //SparkMaxConfig rollerConfig = new SparkMaxConfig();
    public TalonFX roller;

    final int AFIPIVOTMOTORID = 14;
    final int AFIROLLERMOTORID = 15;

    // this is the ratio of motor rotations to intake arm rotations
    final static double MOTOR_TO_INTAKE_RATIO = 5 * 5 * (3.0 / 2.0);
    
  

  public AFISubsystem() {
    //constructor

  
    //Pivot
     if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIPIVOTMOTORID, "AFIPivot")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.frontEncoder = new DutyCycleEncoder(5); //Down is .292
            this.rearEncoder = new DutyCycleEncoder(6);
            this.pivot = new TalonFX(AFIPIVOTMOTORID);
            // this.shoulderEncoder = new CANcoder(10);
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kG = 0; // Gravity FeedForward
            slot0Configs.kS = 0; // Friction FeedForward
            slot0Configs.kP = 1; // an error of 1 rotation results in x Volt output
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            pivot.getConfigurator().apply(slot0Configs);

            pivot.setPosition(AbsoluteIntakeAngle().times(MOTOR_TO_INTAKE_RATIO));
          } // Applies the Config to the shoulder motor

      //Roller
      if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, AFIROLLERMOTORID, "AFIRoller")
                || RobotContainer.shouldMakeAllCANDevices()) {
            this.roller = new TalonFX(AFIROLLERMOTORID);
          } 
          
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     if (pivot != null) {
            SmartDashboard.putNumber("frc3620/AFI/PivotActualPosition", AbsoluteIntakeAngle().in(Degrees));
            SmartDashboard.putNumber("frc3620/AFI/PivotRelativePosition", pivot.getPosition().getValue().div(MOTOR_TO_INTAKE_RATIO).in(Degrees));
        }
    }

    public void setPivotPosition(Angle position) {
        // set the shoulder to the desired position Cat
        SmartDashboard.putNumber("frc3620/AFI/PivotRequestedPosition", position.in(Degrees));

        if (pivot != null) {
            pivot.setControl(pivotRequest.withPosition(position.times(MOTOR_TO_INTAKE_RATIO)));
        }
    }

    double frontEncoderOffset = 0.5575;
    public Angle AbsoluteIntakeAngle() {
      Angle rv = Rotations.of(rearEncoder.get() - frontEncoderOffset);
      return rv;

    }

    public Angle absoluteIntakeAngle() {
      return AbsoluteIntakeAngle();
    }

  public void setAFIRollerSpeed(double speed){
    roller.setControl(rollerRequest.withOutput(speed));
  }
}
