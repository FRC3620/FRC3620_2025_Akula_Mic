package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.Utilities;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

    public ClimberSubsystem() {
        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, CLIMBER_MOTORID, "Climber")
                || RobotContainer.shouldMakeAllCANDevices()) {
            motor = new TalonFX(CLIMBER_MOTORID);
            MotorOutputConfigs _MotorOutputConfigs = new MotorOutputConfigs();

            _MotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motor.getConfigurator().apply(_MotorOutputConfigs);

            motor.setNeutralMode(NeutralModeValue.Brake);
        }

        absEncoder = new DutyCycleEncoder(9);
        absEncoderOffset = Degrees.of(RobotContainer.robotParameters.getClimberEncoderOffset());
    }

    @Override
    public void periodic() {
        if (motor != null) {
            SmartDashboard.putNumber("frc3620/climer postion", motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("frc3620/Climber Output", motor.get());
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
