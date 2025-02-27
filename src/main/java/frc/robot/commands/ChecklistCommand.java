package frc.robot.commands;

import org.usfirst.frc3620.CompoundAlert;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ChecklistCommand extends InstantCommand {
  public static String CHECKLIST_ALERT_GROUPNAME = "frc3620/Checklist";

  enum State {
    PENDING,
    PASSED,
    FAILED
  }

  int numberOfStateOrdinals = State.values().length;

  State state;

  CompoundAlert compoundAlert;

  public ChecklistCommand(String name) {
    compoundAlert = new CompoundAlert(CHECKLIST_ALERT_GROUPNAME, name);
    state = State.PENDING;
    update();
  }

  void update() {
    switch (state) {
      case PENDING:
        compoundAlert.warning("Pending");
        break;

      case PASSED:
        compoundAlert.info("Passed");
        break;

      case FAILED:
        compoundAlert.error("Failed");
        break;

      default:
        break;
    }
  }

  public void initialize() {
    int ordinal = state.ordinal();
    ordinal = ordinal + 1;
    if (ordinal >= numberOfStateOrdinals) ordinal = 0;
    state = State.values()[ordinal];
    update();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
