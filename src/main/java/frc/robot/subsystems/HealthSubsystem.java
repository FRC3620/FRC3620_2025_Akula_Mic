package frc.robot.subsystems;

import java.util.*;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.motors.MotorWatcher;
import org.usfirst.frc3620.motors.MotorWatcherMetric;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  public static final String HARDWARE_ALERT_GROUP_NAME = "frc3620/Hardware Alert";
  public static final String CHECKLIST_GROUP_NAME = "frc3620/Checklist";
  public final static TaggedLogger logger = LoggingMaster.getLogger(HealthSubsystem.class);

  // this stuff is for watching encoders
  EncoderWatcher encoderWatcher;
  Alert disconnectedEncodersAlert = new Alert(HARDWARE_ALERT_GROUP_NAME, "", AlertType.kError);

  // record swerve motor data
  MotorWatcher swerveMotorWatcher;
  EnumSet<MotorWatcherMetric> whatSwerveMetricsToWatch = EnumSet.allOf(MotorWatcherMetric.class);

  // watch the power distribution system
  PDWatcher pdWatcher;
  Alert pdStickyFaultAlert = new Alert(HARDWARE_ALERT_GROUP_NAME, "", AlertType.kWarning);

  Timer timer_2s = new Timer();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    encoderWatcher = new EncoderWatcher();

    encoderWatcher.addEncoder("Intake Front Absolute", RobotContainer.afiSubsystem.frontEncoder);
    encoderWatcher.addEncoder("Intake Rear Absolute", RobotContainer.afiSubsystem.rearEncoder);

    if (RobotContainer.swerveSubsystem != null) {
      swerveMotorWatcher = new MotorWatcher("SmartDashboard/frc3620/health/swerve");
      for (var mapEntry : RobotContainer.swerveSubsystem.getSwerveDrive().getModuleMap().entrySet()) {
        var name = mapEntry.getKey();

        var swerveModule = mapEntry.getValue();

        Object encoder = swerveModule.getAbsoluteEncoder().getAbsoluteEncoder();
        if (encoder instanceof DutyCycleEncoder) {
          encoderWatcher.addEncoder("swerve " + name, (DutyCycleEncoder) encoder);
        }

        var angleMotor = swerveModule.getAngleMotor().getMotor();
        var driveMotor = swerveModule.getDriveMotor().getMotor();
        swerveMotorWatcher.addMotor(name + "/angle", angleMotor, whatSwerveMetricsToWatch);
        swerveMotorWatcher.addMotor(name + "/drive", driveMotor, whatSwerveMetricsToWatch);
      }
    }

    if (RobotContainer.powerDistribution != null) {
      pdWatcher = new PDWatcher(RobotContainer.powerDistribution);
    }

    timer_2s.reset();
    timer_2s.start();
  }

  Random r = new Random(); // only used for testing. creating it and then ignoring it doesn't hurt

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (swerveMotorWatcher != null) {
      swerveMotorWatcher.collect(true);
    }

    if (encoderWatcher != null) {
      processWatcher(encoderWatcher,
          disconnectedEncodersAlert,
          "Absolute encoder(s) disconnected: {}",
          "Absolute encoder(s) disconnected: {}",
          "Absolute encoder(s) broken: ");
    }

    if (timer_2s.advanceIfElapsed(2.0)) {
      periodic_2s();
    }

    // need to look at contents of swerveMotorWatcher and disconnectedEncoders
    // and do SmartDashboard and wpilib alerts, as well as lights

  }

  void periodic_2s() {
    if (pdWatcher != null) {
      processWatcher(pdWatcher,
          pdStickyFaultAlert,
          "New sticky PD faults: {}",
          "Cleared sticky PD faults: {}",
          "PD Sticky Faults: ");
    }
  }

  void processWatcher(Watcher w, Alert alert, String justBrokenLogMessage, String justFixedLogMessage,
      String currentBrokenMessagePrefix) {
    w.update();

    boolean changed = false;
    if (!w.justBroken.isEmpty()) {
      logger.error(justBrokenLogMessage, w.justBroken);
      changed = true;
    }
    if (!w.justFixed.isEmpty()) {
      logger.info(justFixedLogMessage, w.justFixed);
      changed = true;
    }
    if (changed) {
      if (w.broken.isEmpty()) {
        alert.set(false);
        alert.setText("");
      } else {
        alert.set(true);
        alert.setText(currentBrokenMessagePrefix + w.broken);
      }
    }

  }

  abstract class Watcher {
    static final Set<String> empty = Collections.emptySet();
    Set<String> previouslyBroken, broken, justBroken, justFixed;

    Watcher() {
      previouslyBroken = empty;
      broken = empty;
    }

    void update() {
      previouslyBroken = broken;
      broken = new TreeSet<>();
      collect();

      if (!previouslyBroken.equals(broken)) {
        justBroken = new TreeSet<>(broken);
        justBroken.removeAll(previouslyBroken);
        justFixed = new TreeSet<>(previouslyBroken);
        justFixed.removeAll(broken);
      } else {
        justBroken = empty;
        justFixed = empty;
      }
    }

    abstract void collect();
  }

  class PDWatcher extends Watcher {
    PowerDistribution pd;

    PDWatcher(PowerDistribution pd) {
      this.pd = pd;
    }

    void collect() {
      PowerDistributionStickyFaults sticky = pd.getStickyFaults();
      if (sticky.Brownout)
        broken.add("Brownout");
      if (sticky.CanBusOff)
        broken.add("CanBusOff");
      if (sticky.CanWarning)
        broken.add("CanWarning");
      if (sticky.FirmwareFault)
        broken.add("FirmwareFault");
      if (sticky.HardwareFault)
        broken.add("HardwareFault");
      for (int i = 0; i < pd.getNumChannels(); i++) {
        if (sticky.getBreakerFault(i))
          broken.add("Channel " + i);
      }
    }
  }

  class EncoderWatcher extends Watcher {
    Map<String, DutyCycleEncoder> encoders = new TreeMap<>();

    public void addEncoder(String name, DutyCycleEncoder encoder) {
      name = name + " (" + encoder.getSourceChannel() + ")";
      encoders.put(name, encoder);
    }

    void collect() {
      for (var mapEntry : encoders.entrySet()) {
        var dutyCycleEncoder = mapEntry.getValue();
        if (!dutyCycleEncoder.isConnected()) {
          var name = mapEntry.getKey();
          broken.add(name);
        }
      }
    }
  }
}
