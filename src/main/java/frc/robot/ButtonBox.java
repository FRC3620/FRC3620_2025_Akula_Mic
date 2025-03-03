package frc.robot;

import java.util.*;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

public class ButtonBox {
  public enum ButtonId {
    // these are ordered the same as the button numbers from the 
    // buttonbox "joystick"
    A1, A2, A3, A4,
    B1, B2, B3, B4,
    C1, C2, C3, C4,
    D1, D2, D3, D4;

    public int joystickButtonId() {
      return this.ordinal() + 1;
    }
  }

  public static class CommandPair {
    Command first, second;
    CommandPair (Command first, Command second) {
      this.first = first;
      this.second = second;
    }
    public Command getFirstCommand() {
      return first;
    }
    public Command getSecondCommand() {
      return second;
    }
  }

  Map<ButtonId, CommandPair> commandMap = new HashMap<>();

  GenericHID hid;
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  
  public ButtonBox (GenericHID hid) {
    this.hid = hid;
  }

  public void addButtonMapping (ButtonId buttonId, Command firstCommand, Command secondCommand) {
    commandMap.put(buttonId, new CommandPair(firstCommand, secondCommand));
  }

  public ButtonId getSelectedButton() {
    int maxButtonCount = Math.min(hid.getButtonCount(), ButtonId.values().length);
    for (int buttonIndex = 1; buttonIndex <= maxButtonCount; buttonIndex++) {
      boolean hit = hid.getRawButton(buttonIndex);
      logger.info("button {} = {}", buttonIndex, hit);
      if (hid.getRawButton(buttonIndex)) {
        return ButtonId.values()[buttonIndex-1];
      }
    }
    return null;
  }

  public CommandPair getSelectedCommandPair() {
    String name = hid.getName();
    logger.info ("name of HID is {}", name);
    ButtonId buttonId = getSelectedButton();
    logger.info ("No button depressed button is {}", buttonId);
    if (buttonId == null) {
      return null;
    }
    CommandPair commandPair = commandMap.get(buttonId);
    if (commandPair == null) {
      logger.info ("No command for button {}", buttonId);
    }
    return commandPair;
  }

}
