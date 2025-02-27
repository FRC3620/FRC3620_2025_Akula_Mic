package frc.robot.commands.swervedrive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CompoundAlert;
import org.usfirst.frc3620.Utilities.SlidingWindowStats;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.motors.MotorWatcherFetcher;
import org.usfirst.frc3620.motors.MotorWatcherMetric;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HealthSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SwerveDriveDiagnosticCommand extends Command {
  final int slidingWindowSize = 100; // 50 samples a second, so 2 seconds

  SwerveSubsystem swerveSubsystem;
  double power;

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  Map<String, MotorWatcherFetcher> driveMotorFetchers = new HashMap<>();
  Map<String, SlidingWindowStats> driveMotorCurrents = new HashMap<>();

  Map<String, MotorWatcherFetcher> azimuthMotorFetchers = new HashMap<>();
  Map<String, SlidingWindowStats> azimuthMotorCurrents = new HashMap<>();

  public enum Failures {
    DRIVE_SPEED_BAD, DRIVE_CURRENT_BAD,
    AZIMUTH_SPEED_BAD, AZIMUTH_CURRENT_BAD,
    AZIMUTH_ENCODER_DISCONNECTED,
    AZIMUTH_ZERO_BAD;
  }

  CompoundAlert compoundAlert = new CompoundAlert(HealthSubsystem.CHECKLIST_GROUP_NAME, getClass());

  /** Creates a new SwerveDriveDiagnosticCommand. */
  public SwerveDriveDiagnosticCommand(SwerveSubsystem swerveSubsystem) {
    this(swerveSubsystem, 0.2);
  }

  public SwerveDriveDiagnosticCommand(SwerveSubsystem swerveSubsystem, double power) {
    this.swerveSubsystem = swerveSubsystem;
    this.power = power;

    for (var nameAndSwerveModule : swerveSubsystem.getSwerveDrive().getModuleMap().entrySet()) {
      var name = nameAndSwerveModule.getKey();

      Object driveMotor = nameAndSwerveModule.getValue().getDriveMotor().getMotor();
      driveMotorFetchers.put(name, MotorWatcherFetcher.create(driveMotor));

      Object azimuthMotor = nameAndSwerveModule.getValue().getAngleMotor().getMotor();
      azimuthMotorFetchers.put(name, MotorWatcherFetcher.create(azimuthMotor));

      driveMotorCurrents.put(name, new SlidingWindowStats(slidingWindowSize));
      azimuthMotorCurrents.put(name, new SlidingWindowStats(slidingWindowSize));
    }

    compoundAlert.warning("Pending");

    postResultStringToSmartDashboard("pending");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (var stat : driveMotorCurrents.values()) {
      stat.clear();
    }
    for (var stat : azimuthMotorCurrents.values()) {
      stat.clear();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EnumSet<Failures> failure = EnumSet.noneOf(Failures.class);

    if (driveMotorFetchers.size() > 0) {
      List<Double> speeds = new ArrayList<>(driveMotorFetchers.size());
      List<Double> currents = new ArrayList<>(driveMotorFetchers.size());
      for (var nameAndMotorFetcher : driveMotorFetchers.entrySet()) {
        String name = nameAndMotorFetcher.getKey();
        var fetcher = nameAndMotorFetcher.getValue();

        fetcher.setPower(power);
        var currentStats = driveMotorCurrents.get(name);

        fetcher.collect(EnumSet.of(MotorWatcherMetric.OUTPUT_CURRENT, MotorWatcherMetric.VELOCITY));

        double velocity = fetcher.getVelocity();
        speeds.add(velocity);
        SmartDashboard.putNumber("frc3620/diagnostics/" + name + "/drive/speed", velocity);

        double current = fetcher.getOutputCurrent();
        currentStats.addValue(current);
        double mean = currentStats.getMean();
        currents.add(mean);
        SmartDashboard.putNumber("frc3620/diagnostics/" + name + "/drive/current", mean);
      }

      double minSpeed = Collections.min(speeds);
      double maxSpeed = Collections.max(speeds);
      double minCurrent = Collections.min(currents);
      double maxCurrent = Collections.max(currents);

      if (maxSpeed == 0) {
        failure.add(Failures.DRIVE_SPEED_BAD);
      } else {
        if (minSpeed / maxSpeed < .9) {
          failure.add(Failures.DRIVE_SPEED_BAD);
        }
      }

      if (maxCurrent == 0 || minCurrent == 0) {
        failure.add(Failures.DRIVE_CURRENT_BAD);
      } else {
        if (minCurrent / maxCurrent < .25) {
          failure.add(Failures.DRIVE_CURRENT_BAD);
        }
      }

    }

    if (azimuthMotorFetchers.size() > 0) {
      List<Double> speeds = new ArrayList<>(azimuthMotorFetchers.size());
      List<Double> currents = new ArrayList<>(azimuthMotorFetchers.size());
      for (var nameAndMotorFetcher : azimuthMotorFetchers.entrySet()) {
        String name = nameAndMotorFetcher.getKey();
        var fetcher = nameAndMotorFetcher.getValue();

        fetcher.setPower(power);
        var currentStats = azimuthMotorCurrents.get(name);

        fetcher.collect(
            EnumSet.of(MotorWatcherMetric.OUTPUT_CURRENT, MotorWatcherMetric.VELOCITY, MotorWatcherMetric.POSITION));

        double velocity = fetcher.getVelocity();
        speeds.add(velocity);
        SmartDashboard.putNumber("frc3620/diagnostics/" + name + "/azimuth/speed", velocity);

        double current = fetcher.getOutputCurrent();
        currentStats.addValue(current);
        double mean = currentStats.getMean();
        currents.add(mean);
        SmartDashboard.putNumber("frc3620/diagnostics/" + name + "/azimuth/current", mean);
      }

      double minSpeed = Collections.min(speeds);
      double maxSpeed = Collections.max(speeds);
      double minCurrent = Collections.min(currents);
      double maxCurrent = Collections.max(currents);

      if (maxSpeed == 0) {
        failure.add(Failures.AZIMUTH_SPEED_BAD);
      } else {
        if (minSpeed / maxSpeed < .9) {
          failure.add(Failures.AZIMUTH_SPEED_BAD);
        }
      }

      if (maxCurrent == 0 || minCurrent == 0) {
        failure.add(Failures.AZIMUTH_CURRENT_BAD);
      } else {
        if (minCurrent / maxCurrent < .25) {
          failure.add(Failures.AZIMUTH_CURRENT_BAD);
        }
      }
    }

    // TODO: test to see if absolute encoders are connected

    // TODO: test to see if encoders match

    // handle FAILURE here
    if (failure.isEmpty()) {
      compoundAlert.info("Passed");
      postResultStringToSmartDashboard("passed");
    } else {
      String text = "Failed, " + failure.toString();
      compoundAlert.error(text);
      postResultStringToSmartDashboard(text);
    }
  }

  void postResultStringToSmartDashboard (String text) {
    SmartDashboard.putString("frc3620/" + getClass().getSimpleName() + "/result", text);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
