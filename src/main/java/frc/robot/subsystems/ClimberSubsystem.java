package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.security.Key;
import java.util.function.DoubleSupplier;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.Utilities;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public ClimberSubsystem() {

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, CLIMBER_MOTORID, "Climber")
                || RobotContainer.shouldMakeAllCANDevices()) {
            motor = new TalonFX(CLIMBER_MOTORID);
            MotorOutputConfigs _MotorOutputConfigs = new MotorOutputConfigs();

            _MotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motor.getConfigurator().apply(_MotorOutputConfigs);

            // in init function, set slot 0 gains
            /*
            Slot0Configs slot0Configs = new Slot0Configs();

            slot0Configs.kP = 4.8; // An error of 1 rotation results in 2.4 V output
            slot0Configs.kI = 1; // no output for integrated error
            slot0Configs.kD = 0.003; // A velocity of 1 rps results in 0.1 V output
            motor.getConfigurator().apply(slot0Configs);
            */
            motor.setNeutralMode(NeutralModeValue.Brake);

        }
        absEncoder = new DutyCycleEncoder(9);
        absEncoderOffset = Degrees.of(RobotContainer.robotParameters.getClimberEncoderOffset());

        //setPostion(Degrees.of(90));

    }

    @Override
    public void periodic() {
        if (motor != null) {
            SmartDashboard.putNumber("frc3620/climer postion", motor.getPosition().getValueAsDouble());
        }
        SmartDashboard.putNumber("frc3620/climerabsoluteposition", getClimberAngle().in(Degrees));

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
