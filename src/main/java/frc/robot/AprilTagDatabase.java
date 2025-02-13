package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.HashMap;
import java.util.Map;

public class AprilTagDatabase {
  static Map<Integer, AprilTag> idToAprilTagMap = new HashMap<>();

  static {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    for (var aprilTag : aprilTagFieldLayout.getTags()) idToAprilTagMap.put(aprilTag.ID, aprilTag);
  }

  public static AprilTag getTagById(int id) {
    return idToAprilTagMap.get(id);
  }
}
