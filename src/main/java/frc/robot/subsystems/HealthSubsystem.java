package frc.robot.subsystems;

import java.util.*;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.motors.MotorWatcher;
import org.usfirst.frc3620.motors.MotorWatcherFetcher;
import org.usfirst.frc3620.motors.MotorWatcherMetric;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkySubsystem.HealthStatus;

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

  Tracer tracer = new Tracer();

  Map<String, HealthStatus> healthMap = new HashMap<>();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    encoderWatcher = new EncoderWatcher();
    updateNTForDisconnectEncoders(new String[0]);

    encoderWatcher.addEncoder("Intake Front Absolute", RobotContainer.afiSubsystem.frontEncoder);
    encoderWatcher.addEncoder("Intake Rear Absolute", RobotContainer.afiSubsystem.rearEncoder);
    encoderWatcher.addEncoder("Climber Absolute", RobotContainer.climberSubsystem.absEncoder);

    if (RobotContainer.swerveSubsystem != null) {
      swerveMotorWatcher = new MotorWatcher("SmartDashboard/frc3620/health/swerve");
      swerveMotorWatcher.setTracer(tracer);
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

  @Override
  public void periodic() {
    long t0 = RobotController.getFPGATime();
    tracer.clearEpochs();

    checkSwerveMotors();
    checkAbsoluteEncoders();

    if (timer_2s.advanceIfElapsed(2.0)) {
      checkPowerDistribution();
    }

    if (RobotContainer.powerDistribution != null) {
      SmartDashboard.putNumber("frc3620/power/energy", RobotContainer.powerDistribution.getTotalEnergy());
    }
    tracer.addEpoch("gather energy data");

    for (var healthMapEntry : healthMap.entrySet()) {
      SmartDashboard.putString("frc3620/health/status/" + healthMapEntry.getKey(), healthMapEntry.getValue().toString());
    }
    RobotContainer.blinkySubsystem.setHealthStatus(Collections.max(healthMap.values()));
    tracer.addEpoch("calculate healthStatus");

    long t = RobotController.getFPGATime() - t0; // microseconds
    if (t > 10000) { // 10ms
      tracer.printEpochs(out -> logger.info("HealthSubsystem.periodic ran long: {}us, {}", t, out));
    }
  }

  /*
  DutyCycleEncoderSim sim = new DutyCycleEncoderSim(RobotContainer.climberSubsystem.absEncoder);
  @Override
  public void simulationPeriodic() {
    sim.setConnected(false);
  }
  */

  void checkSwerveMotors() {
    if (swerveMotorWatcher != null) {
      swerveMotorWatcher.collect(true);

      Set<HealthStatus> healthStati = new HashSet<>();
      healthStati.add(HealthStatus.OKAY);
      for (var mwi : swerveMotorWatcher.getCollectedInformation()) {
        MotorWatcherFetcher f = mwi.getFetcher();
        for (var datum : mwi.getMetrics()) {
          if (datum == MotorWatcherMetric.TEMPERATURE) {
            double temperature = datum.getValue(f);
            if (temperature > 100) {
              healthStati.add(HealthStatus.ERROR);
            } else if (temperature > 70) {
              healthStati.add(HealthStatus.WARNING);
            }
          }
        }
      }
      healthMap.put("motorTemperature", Collections.max(healthStati));
      tracer.addEpoch("checking swerve motors");
    }
  }

  void checkAbsoluteEncoders() {
    if (encoderWatcher != null) {
      boolean changed = processWatcher(encoderWatcher,
          disconnectedEncodersAlert,
          "Absolute encoder(s) disconnected: {}",
          "Absolute encoder(s) reconnected: {}",
          "Absolute encoder(s) broken: ");
      if (changed) {
        updateNTForDisconnectEncoders(encoderWatcher.broken.toArray(String[]::new));
      }
      healthMap.put("encoders", encoderWatcher.broken.size() > 0 ? BlinkySubsystem.HealthStatus.HAIRONFIRE : BlinkySubsystem.HealthStatus.OKAY);
      tracer.addEpoch("checking encoders");
    }
  }

  void updateNTForDisconnectEncoders(String[] disconnectedEncoders) {
    SmartDashboard.putStringArray("frc3620/health/disconnectedEncoders", disconnectedEncoders);
  }

  void checkPowerDistribution() {
    if (pdWatcher != null) {
      boolean changed = processWatcher(pdWatcher,
          pdStickyFaultAlert,
          "New sticky PD faults: {}",
          "Cleared sticky PD faults: {}",
          "PD Sticky Faults: ");
      if (changed) {
        SmartDashboard.putStringArray("frc3620/power/stickyFaults", pdWatcher.broken.toArray(String[]::new));
      }
      tracer.addEpoch("check sticky faults");
    }
  }

  boolean processWatcher(Watcher w, Alert alert, String justBrokenLogMessage, String justFixedLogMessage,
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
    return changed;
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
