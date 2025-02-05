package frc.robot.subsystems;

import java.util.*;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.motors.MotorWatcher;
import org.usfirst.frc3620.motors.MotorWatcherMetric;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  public static final String DIAGNOSTIC_ALERT_GROUP_NAME = "Diagnostics";
  public final static TaggedLogger logger = LoggingMaster.getLogger(HealthSubsystem.class);

  MotorWatcher swerveMotorWatcher;

  Alert disconnectedEncodersAlert = new Alert(HealthSubsystem.DIAGNOSTIC_ALERT_GROUP_NAME, "", Alert.AlertType.kError);
  Map<String, DutyCycleEncoder> encoders = new TreeMap<>();
  Map<String, Boolean> encoderWasConnectedLastTime = new TreeMap<>();
  Set<String> disconnectedEncoders = new TreeSet<>();

  EnumSet<MotorWatcherMetric> whatSwerveMetricsToWatch = EnumSet.allOf(MotorWatcherMetric.class);

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    if(RobotContainer.swerveSubsystem != null) {
      swerveMotorWatcher = new MotorWatcher("SmartDashboard/frc3620/health/swerve");
  
      for (var mapEntry : RobotContainer.swerveSubsystem.getSwerveDrive().getModuleMap().entrySet()) {
        var name = mapEntry.getKey();
        var swerveModule = mapEntry.getValue();
        var angleMotor = swerveModule.getAngleMotor().getMotor();
        var driveMotor = swerveModule.getDriveMotor().getMotor();
  
        swerveMotorWatcher.addMotor(name + "/angle", angleMotor, whatSwerveMetricsToWatch);
        swerveMotorWatcher.addMotor(name + "/drive", driveMotor, whatSwerveMetricsToWatch);

        Object encoder = swerveModule.getAbsoluteEncoder().getAbsoluteEncoder();
        if (encoder instanceof DutyCycleEncoder) {
          encoders.put(name, (DutyCycleEncoder) encoder);
          encoderWasConnectedLastTime.put(name, true);
        }
      }
    }
  }

  Random r = new Random(); // only used for testing. creating it and then ignoring it doesn't hurt

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(swerveMotorWatcher != null) {
      swerveMotorWatcher.collect(true);
    }

    boolean needToUpdate = false;
    disconnectedEncoders.clear();

    for (var mapEntry : encoders.entrySet()) {
      var name = mapEntry.getKey();
      var dutyCycleEncoder = mapEntry.getValue();
      var isConnected = dutyCycleEncoder.isConnected();
      var wasConnectedLastTime = encoderWasConnectedLastTime.get(name);

      // use this to do some testing.
      // isConnected = (r.nextDouble() > 0.005) ? wasConnectedLastTime : ! wasConnectedLastTime;
      // end testing code

      if (!isConnected) {
        disconnectedEncoders.add(name);
      }
      if (isConnected != wasConnectedLastTime) {
        needToUpdate = true;
        if (!isConnected) {
          disconnectedEncoders.add(name);
          logger.error ("Swerve absolute encoder disconnected: {}", name);
        } else {
          logger.error ("Swerve absolute encoder reconnected: {}", name);
        }
      }
      encoderWasConnectedLastTime.put(name, isConnected);
    }
    if (needToUpdate) {
      if (disconnectedEncoders.isEmpty()) {
        disconnectedEncodersAlert.set(false);
      } else {
        disconnectedEncodersAlert.setText("Disconnected swerve absolute encoders: " + String.join(", ", disconnectedEncoders));
        disconnectedEncodersAlert.set(true);
      }
    }

    // need to look at contents of swerveMotorWatcher and disconnectedEncoders
    // and do SmartDashboard and wpilib alerts, as well as lights
  }
}
