package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

    final int CLIMBER_MOTORID = 13;

    static public double pos1 = 0;
    static public double pos2 = 2;
    TalonFX motor;
    public DutyCycleEncoder absEncoder;
    Angle absEncoderOffset;

    public ClimberSubsystem() {

        if (RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, CLIMBER_MOTORID, "Climber")
                || RobotContainer.shouldMakeAllCANDevices()) {
            motor = new TalonFX(CLIMBER_MOTORID);

            // in init function, set slot 0 gains
            Slot0Configs slot0Configs = new Slot0Configs();

            slot0Configs.kP = 4.8; // An error of 1 rotation results in 2.4 V output
            slot0Configs.kI = 1; // no output for integrated error
            slot0Configs.kD = 0.003; // A velocity of 1 rps results in 0.1 V output
            motor.getConfigurator().apply(slot0Configs);
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
        absEncoder = new DutyCycleEncoder(9);
        absEncoderOffset = Degrees.of(RobotContainer.robotParameters.getClimberEncoderOffset());
    }

    @Override
    public void periodic() {
        if (motor != null) {
            SmartDashboard.putNumber("frc3620/climer postion", motor.getPosition().getValueAsDouble());
        }
        SmartDashboard.putNumber("frc3620/climerabsoluteposition", getClimberAngle().in(Degrees));
    }

    public void setPostion(Double cpos) {

        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        if (motor != null) {

            // set position to 10 rotations
            motor.setControl(m_request.withPosition(cpos));
            // motor.set(0.4);

        }
        SmartDashboard.putNumber("frc3620/requested climer postion", cpos);

    }

    public void setClimberPower(double power) {
        if (motor != null) {
            motor.set(power);
        }
    }

    Angle getClimberAngle() {
        return Rotations.of(absEncoder.get()).minus(absEncoderOffset);
    }
}
