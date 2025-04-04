package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

    TalonFX motor;
    public DutyCycleEncoder absEncoder;
    Angle absEncoderOffset;

    final Angle MINPOSITION = Degrees.of(58.7);
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
        absEncoder.setInverted(true);
    }

    @Override
    public void periodic() {
        if (motor != null) {
            SmartDashboard.putNumber("frc3620/climber/motorPosition", motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("frc3620/climber/motorOutput", motor.get());

        }
        SmartDashboard.putNumber("frc3620/climber/absolutePosition", getClimberAngle().in(Degrees));
    }

    public void setClimberPower(double power) {
        SmartDashboard.putNumber("frc3620/climber/manualClimberPower", power);

        if (motor != null) {
            Angle climberAngle = getClimberAngle();
            if ((climberAngle.lt(MINPOSITION) && power < 0)
                    || (climberAngle.gt(MAXPOSITION) && power > 0)) {
                motor.set(0);
            } else {
                motor.set(power);
            }
        }
    }

    Angle getClimberAngle() {
        return Rotations.of(absEncoder.get()).minus(absEncoderOffset);
    }
}
