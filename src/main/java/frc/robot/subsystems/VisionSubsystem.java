package frc.robot.subsystems;

import java.nio.file.attribute.PosixFileAttributes;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.usfirst.frc3620.NTPublisher;
import org.usfirst.frc3620.NTStructs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.VisionSubsystem.CameraData.MegaTagData;
import swervelib.SwerveDrive;

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
    final String limelightName;
    public final MegaTagData megaTag1 = new MegaTagData("megaTag1");
    public final MegaTagData megaTag2 = new MegaTagData("megaTag2");

    CameraData(Camera c) {
      limelightName = c.limelightName;
      /*
       * see
       * https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-
       * change.html#using-networktableinstance-to-listen-for-changes
       * 
       * we set the haveNew* atomic(s) when the corresponding network entry is
       * updated, and in periodic() we only go through all the work of reading
       * the network entry if it had been updated.
       */
      DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag1.haveNewPose.set(true));

      entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_orb_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag2.haveNewPose.set(true));
    }

    public String getLimelightName() {
      return limelightName;
    }

    public class MegaTagData {
      // gets set whenever we have new data
      AtomicBoolean haveNewPose = new AtomicBoolean(false);

      final String megaTagName;

      PoseEstimate poseEstimate;

      MegaTagData(String megaTagName) {
        this.megaTagName = megaTagName;
      }

      public String getMegaTagName() {
        return megaTagName;
      }

      public PoseEstimate getPoseEstimate() {
        return poseEstimate;
      }

      public String getLimelightName() {
        return limelightName;
      }
    }
  }

  Map<Camera, CameraData> allCameraData = new TreeMap<>();
  Set<CameraData> allCameraDataAsSet;

  public VisionSubsystem() {
    allCameraData.put(Camera.FRONT, new CameraData(Camera.FRONT));
    allCameraData.put(Camera.BACK, new CameraData(Camera.BACK));
    allCameraData = Map.copyOf(allCameraData); // make immutable
    allCameraDataAsSet = Set.copyOf(allCameraData.values());
  }

  void processMegaTag(MegaTagData megaTagData, Supplier<PoseEstimate> supplier) {
    if (megaTagData.haveNewPose.getAndSet(false)) {
      PoseEstimate m = supplier.get();

      if (m != null) {
        megaTagData.poseEstimate = m;

        var prefix = "SmartDashboard/frc3620/vision/" + megaTagData.getLimelightName() + "/" + megaTagData.megaTagName
            + "/";
        NTPublisher.putNumber(prefix + "targetCount", m.tagCount);
        NTStructs.publish(prefix + "poseEstimate", m.pose);
      
      }
    }
  }

  @Override
  public void periodic() {
    double yaw = 0;
    double yawRate = 0;
    double pitch = 0;
    boolean doRejectUpdate = false;
    SwerveDrive sd = null;
    if (RobotContainer.swerveSubsystem != null) {
      sd = RobotContainer.swerveSubsystem.getSwerveDrive();
      yaw = sd.getYaw().getDegrees();
      pitch = sd.getPitch().getDegrees();
      // need to convert this to degrees / s.
      // yawRate = sd.getGyro().getYawAngularVelocity();
    }
    for (var cameraData : allCameraData.values()) {
      
      LimelightHelpers.SetRobotOrientation(cameraData.limelightName, yaw, yawRate, pitch, 0, 0, 0);
      processMegaTag(cameraData.megaTag1, () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraData.limelightName));
      processMegaTag(cameraData.megaTag2,
          () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraData.limelightName));

      // update robot odometry from vision
      if (Math.abs(yawRate) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                            // vision updates
      {
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate == null) {
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate.tagCount == 0) {
        doRejectUpdate = true;
      }
      else if(sd == null){
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate.pose.getX() >= 17){
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate.pose.getX() <= 0){
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate.pose.getY() >= 8.5){
        doRejectUpdate = true;
      }
      else if (cameraData.megaTag2.poseEstimate.pose.getY() <= 0){
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        sd.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        sd.addVisionMeasurement(cameraData.megaTag2.poseEstimate.pose, cameraData.megaTag2.poseEstimate.timestampSeconds);
      }
    }

  }

  public CameraData getCameraData(Camera camera) {
    return allCameraData.get(camera);
  }

  public Set<CameraData> getAllCameraData() {
    return allCameraDataAsSet;
  }

}
