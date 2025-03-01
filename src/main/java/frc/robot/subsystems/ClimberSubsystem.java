package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

<<<<<<< HEAD
import java.security.Key;
import java.util.function.DoubleSupplier;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.Utilities;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
=======
import org.usfirst.frc3620.CANDeviceType;
>>>>>>> ESEFTuning
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

    final int CLIMBER_MOTORID = 13;

    static public Angle pos1 = Degrees.of(95);
    static public Angle pos2 = Degrees.of(180);
    
    TalonFX motor;
    public DutyCycleEncoder absEncoder;
    Angle absEncoderOffset;

    final Angle MINPOSITION = Degrees.of(0);
    final Angle MAXPOSITION = Degrees.of(180);

    // Motion Magic Control
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public ClimberSubsystem() {
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, CLIMBER_MOTORID, "Climber")
                || RobotContainer.shouldMakeAllCANDevices()) {
            motor = new TalonFX(CLIMBER_MOTORID);
            MotorOutputConfigs _MotorOutputConfigs = new MotorOutputConfigs();

            _MotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motor.getConfigurator().apply(_MotorOutputConfigs);

            // Configure PID for Motion Magic
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = 500;  // Proportional gain (TUNE THIS)
            slot0Configs.kI = 0.0;  // Integral gain
            slot0Configs.kD = 0.0;  // Derivative gain
            slot0Configs.kV = 0.1;  // Velocity feedforward
            slot0Configs.kA = 0.0;  // Acceleration feedforward
            motor.getConfigurator().apply(slot0Configs);
<<<<<<< HEAD
            */
            motor.setNeutralMode(NeutralModeValue.Brake);

=======

            // Configure Motion Magic
            MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
            mmConfigs.MotionMagicCruiseVelocity = 50; // Max speed (TUNE THIS)
            mmConfigs.MotionMagicAcceleration = 30;   // Acceleration (TUNE THIS)
            mmConfigs.MotionMagicJerk = 5;            // Jerk smoothing (Optional)
            motor.getConfigurator().apply(mmConfigs);

            motor.setNeutralMode(NeutralModeValue.Brake);
>>>>>>> ESEFTuning
        }

        absEncoder = new DutyCycleEncoder(9);
        absEncoderOffset = Degrees.of(RobotContainer.robotParameters.getClimberEncoderOffset());

<<<<<<< HEAD
        //setPostion(Degrees.of(90));
=======
        // Zero Falcon encoder to absolute encoder reading at startup
        double absolutePositionRotations = getClimberAngle().in(Rotations);
        motor.setPosition(absolutePositionRotations);
>>>>>>> ESEFTuning

        // Move climber to starting position
        setPosition(Degrees.of(90));
    }

    @Override
    public void periodic() {
        if (motor != null) {
<<<<<<< HEAD
            SmartDashboard.putNumber("frc3620/climer postion", motor.getPosition().getValueAsDouble());
        }
        SmartDashboard.putNumber("frc3620/climerabsoluteposition", getClimberAngle().in(Degrees));

=======
            SmartDashboard.putNumber("Falcon Encoder Position", motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Absolute Climber Position", getClimberAngle().in(Degrees));
            SmartDashboard.putNumber("Climber Output", motor.get());
        }
    }

    public void setPosition(Angle cpos) {
        if (motor != null) {
            // Convert degrees to rotations and clamp within limits
            double rotations = cpos.in(Rotations);
            rotations = Math.max(MINPOSITION.in(Rotations), Math.min(rotations, MAXPOSITION.in(Rotations)));

            // Use Motion Magic to move to position
            motor.setControl(motionMagicRequest.withPosition(rotations));
        }
        SmartDashboard.putNumber("Requested Climber Position", cpos.in(Degrees));
>>>>>>> ESEFTuning
    }

    public void setClimberPower(double power) {
        SmartDashboard.putNumber("frc3620/manualclimberPower",power);

        if (motor != null) {
            motor.set(power);
        }
    }

    Angle getClimberAngle() {
        return Rotations.of(absEncoder.get()).minus(absEncoderOffset);
    }
}
