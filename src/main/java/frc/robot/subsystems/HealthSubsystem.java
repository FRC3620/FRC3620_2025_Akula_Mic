package frc.robot.subsystems;

import java.util.EnumSet;

import org.usfirst.frc3620.motors.MotorWatcher;
import org.usfirst.frc3620.motors.MotorWatcherMetric;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  MotorWatcher swerveMotorWatcher;

  EnumSet<MotorWatcherMetric> whatToWatch = EnumSet.allOf(MotorWatcherMetric.class);
  
  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    if(RobotContainer.swerveSubsystem!=null){
      swerveMotorWatcher = new MotorWatcher("SmartDashboard/frc3620/health/swerve");
  
      for (var mapEntry : RobotContainer.swerveSubsystem.getSwerveDrive().getModuleMap().entrySet()) {
        var name = mapEntry.getKey();
        var swerveModule = mapEntry.getValue();
        var angleMotor = swerveModule.getAngleMotor().getMotor();
        var driveMotor = swerveModule.getDriveMotor().getMotor();
  
        swerveMotorWatcher.addMotor(name + "/angle", angleMotor, whatToWatch);
        swerveMotorWatcher.addMotor(name + "/drive", driveMotor, whatToWatch);
      }

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler ru
    if(swerveMotorWatcher!=null)
    swerveMotorWatcher.collect(true);

    // need to look at contents of swerveMotorWatcher and do SmartDashboard and wpilib alerts, as well
    // as lights
  }
}
