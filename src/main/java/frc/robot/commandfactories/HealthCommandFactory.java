package frc.robot.commandfactories;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.stream.Stream;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ChecklistCommand;

public class HealthCommandFactory {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  public void setupSmartDashboardCommands() {
    File file = new File(Filesystem.getDeployDirectory(), "checklist.txt");
    try (Stream<String> lines = Files.lines(Paths.get(file.getAbsolutePath()))) {
      lines.forEach(this::addItem); // Process each line
    } catch (IOException e) {
      logger.error(e, "Trouble reading checklist file");
    }
  }

  void addItem(String s) {
    s = s.strip();
    if (s.length() > 0)
      SmartDashboard.putData("checklist/" + s, new ChecklistCommand(s).withName(s));
  }
}
