package frc.robot.subsystems;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

import org.usfirst.frc3620.NTPublisher;
import org.usfirst.frc3620.NTStructs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public enum Camera {
    FRONT("limelight-front"), BACK("limelight-back");

    public final String limelightName;

    Camera(String limelightName) {
      this.limelightName = limelightName;
    }
  }

  public class CameraData {
    // gets set whenever we have new data
    AtomicBoolean haveNewMetatag1Pose = new AtomicBoolean(false);

    Pose2d metatag1Pose;
    double metatag1Timestamp;
    int metatag1TagCount;

    CameraData(Camera c) {
      /*
       * see
       * https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html#using-networktableinstance-to-listen-for-changes
       * 
       * we set the haveNew* atomic(s) when the corresponding network entry is updated,
       * and in periodic(), only
       * go through all the work of reading it if we got a new one.
       */
      DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> haveNewMetatag1Pose.set(true));
    }

    public Pose2d getMetatag1Pose() {
      return metatag1Pose;
    }

    public double getMetatag1Timestamp() {
      return metatag1Timestamp;
    }

    public int getMetatag1TagCount() {
      return metatag1TagCount;
    }
  }

  Map<Camera, CameraData> cameraData = new TreeMap<>();

  public VisionSubsystem() {
    cameraData.put(Camera.FRONT, new CameraData(Camera.FRONT));
    cameraData.put(Camera.BACK, new CameraData(Camera.BACK));
  }

  @Override
  public void periodic() {
    for (var entry : cameraData.entrySet()) {
      var cameraData = entry.getValue();

      if (cameraData.haveNewMetatag1Pose.getAndSet(false)) {
        var limelightName = entry.getKey().limelightName;
        PoseEstimate m = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (m != null) {
          cameraData.metatag1Pose = m.pose;
          cameraData.metatag1Timestamp = m.timestampSeconds;
          cameraData.metatag1TagCount = m.tagCount;

          var prefix = "SmartDashboard/frc3620/vision/" + limelightName + "/";
          NTPublisher.putNumber(prefix + "targetCount", m.tagCount);
          NTStructs.publish(prefix + "poseEstimate", m.pose);
        }
      }
    }
  }

  public CameraData getCameraData(Camera camera) {
    return cameraData.get(camera);
  }

}
