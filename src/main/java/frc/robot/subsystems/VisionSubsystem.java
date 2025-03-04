package frc.robot.subsystems;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.dyn4j.geometry.Rotatable;
import org.tinylog.Logger;
import org.usfirst.frc3620.NTPublisher;
import org.usfirst.frc3620.NTStructs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.VisionSubsystem.CameraData.MegaTagData;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  private Map<Translation2d, Integer> translationToTagMap = new HashMap<>();

  private Map<Integer, Translation2d> tagToTranslationMap = new HashMap<>();

  private Map<Integer, Pose2d> tagToStickPose2dLeft = new HashMap<>();

  private Map<Integer, Pose2d> tagToStickPose2dRight = new HashMap<>();

  private Translation2d centerBlueReef;

  private List<Translation2d> tagTranslations = new ArrayList<>();

  static Optional<Alliance> color;

  double maxDistanceFromCenterToBeClose = 3;// Distance in meters

  String lastLoggedError;

  public enum Camera {
    FRONT("limelight-front"), BACK("limelight-back");

    public final String limelightName;

    Camera(String limelightName) {
      this.limelightName = limelightName;
    }
  }

  public enum WhichBlueStick {
    BSTICKA(5.71, 3.80, Rotation2d.fromDegrees(-180)),
    BSTICKB(5.71, 4.26, Rotation2d.fromDegrees(-180)),
    BSTICKC(5.17, 5.2, Rotation2d.fromDegrees(-120)),
    BSTICKD(5.1, 5.5, Rotation2d.fromDegrees(-120)),
    BSTICKE(4, 5.35, Rotation2d.fromDegrees(-60)),
    BSTICKF(3.6, 5.18, Rotation2d.fromDegrees(-60)),
    BSTICKG(3.26, 4.2, Rotation2d.fromDegrees(0)),
    BSTICKH(3.26, 3.84, Rotation2d.fromDegrees(0)),
    BSTICKI(3.85, 3, Rotation2d.fromDegrees(60)),
    BSTICKJ(3.96, 2.9, Rotation2d.fromDegrees(60)),
    BSTICKK(4.98, 2.9, Rotation2d.fromDegrees(120)),
    BSTICKL(5.1, 3.09, Rotation2d.fromDegrees(120));

    public final Pose2d pose;

    WhichBlueStick(double x, double y, Rotation2d rotation) {
      pose = new Pose2d(x, y, rotation);
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

      double distanceToClosestSeenTarget;

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

      public double getDistanceToClosestSeenTarget() {
        return distanceToClosestSeenTarget;
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

    setUpTagMaps();
  }

  void processMegaTag(MegaTagData megaTagData, Supplier<PoseEstimate> supplier, Pose2d currentPose) {
    if (megaTagData.haveNewPose.getAndSet(false)) {
      PoseEstimate m = supplier.get();

      if (m != null) {
        megaTagData.poseEstimate = m;

        var prefix = "SmartDashboard/frc3620/vision/" + megaTagData.getLimelightName() + "/" + megaTagData.megaTagName
            + "/";
        NTPublisher.putNumber(prefix + "targetCount", m.tagCount);
        NTStructs.publish(prefix + "poseEstimate", m.pose);
        if (currentPose != null) {
          NTPublisher.putNumber(prefix + "distanceFromSwervePose", currentPose.getTranslation().getDistance(m.pose.getTranslation()));
        }

        if (aprilTagFieldLayout != null) {
          List<Pose3d> targetPoses = new ArrayList<>();

          double distanceToClosestSeenTarget = 100000;

          for (var fiducial : m.rawFiducials) {
            Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose(fiducial.id);
            if (aprilTagPose.isPresent()) {
              targetPoses.add(aprilTagPose.get());
              double distanceToThisTag = m.pose.getTranslation()
                  .getDistance(aprilTagPose.get().getTranslation().toTranslation2d());
              if (distanceToThisTag < distanceToClosestSeenTarget) {
                distanceToClosestSeenTarget = distanceToThisTag;
              }
              megaTagData.distanceToClosestSeenTarget = distanceToClosestSeenTarget;
              NTPublisher.putNumber(prefix + "distance to closest seen tag", distanceToClosestSeenTarget);
            }
          }
          NTStructs.publish(prefix + "targets", targetPoses.toArray(new Pose3d[0]));
        }
      }
    }
  }

  void setUpTagMaps() {

    for (int tagID = 1; tagID <= 22; tagID++) {

      Translation2d translation = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().getTranslation()
          .toTranslation2d();

      translationToTagMap.put(translation, tagID);
      tagToTranslationMap.put(tagID, translation);

      tagTranslations.add(translation);

    }

    tagToStickPose2dLeft.put(17, WhichBlueStick.BSTICKI.pose);
    tagToStickPose2dLeft.put(18, WhichBlueStick.BSTICKG.pose);
    tagToStickPose2dLeft.put(19, WhichBlueStick.BSTICKE.pose);
    tagToStickPose2dLeft.put(20, WhichBlueStick.BSTICKC.pose);
    tagToStickPose2dLeft.put(21, WhichBlueStick.BSTICKA.pose);
    tagToStickPose2dLeft.put(22, WhichBlueStick.BSTICKK.pose);

    tagToStickPose2dRight.put(17, WhichBlueStick.BSTICKJ.pose);
    tagToStickPose2dRight.put(18, WhichBlueStick.BSTICKH.pose);
    tagToStickPose2dRight.put(19, WhichBlueStick.BSTICKF.pose);
    tagToStickPose2dRight.put(20, WhichBlueStick.BSTICKD.pose);
    tagToStickPose2dRight.put(21, WhichBlueStick.BSTICKB.pose);
    tagToStickPose2dRight.put(22, WhichBlueStick.BSTICKL.pose);

    centerBlueReef = tagToTranslationMap.get(17).plus(tagToTranslationMap.get(20)).div(2);

  }

  @Override
  public void periodic() {

    // gets alliance color
    color = DriverStation.getAlliance();

    // added this to handle case where color is not yet set, otherwise we blow up in
    // the simulator
    if (color.isEmpty())
      return;

    double yaw = 0;
    double yawRate = 0;
    double pitch = 0;
    SwerveDrive sd = null;
    Pose2d currentSwervePose = null;
    if (RobotContainer.swerveSubsystem != null) {
      sd = RobotContainer.swerveSubsystem.getSwerveDrive();
      yaw = sd.getYaw().getDegrees();
      pitch = sd.getPitch().getDegrees();
      // need to convert this to degrees / s.
      // yawRate = sd.getGyro().getYawAngularVelocity();
      currentSwervePose = sd.getPose();
    }

    for (var cameraData : allCameraData.values()) {
      
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(cameraData.limelightName, yaw, yawRate, pitch, 0, 0, 0);
      processMegaTag(cameraData.megaTag1, () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraData.limelightName), currentSwervePose);
      processMegaTag(cameraData.megaTag2,
          () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraData.limelightName), currentSwervePose);

      // update robot odometry from vision
      if (Math.abs(yawRate) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                   // vision updates
      {
        doRejectUpdate = true;
        if (lastLoggedError != "Angular Velocity") {
          // Logger.info("Vision Reject : Angular Velocity", "");
          lastLoggedError = "Angular Velocity";
        }
      } else if (cameraData.megaTag2.poseEstimate == null) {
        doRejectUpdate = true;
        if (lastLoggedError != "megaTag2Pose = null") {
          // Logger.info("Vision Reject : megaTag2 Pose = null", "");
          lastLoggedError = "megaTag2Pose = null";
        }
      } else if (cameraData.megaTag2.poseEstimate.tagCount == 0) {
        doRejectUpdate = true;
        if (lastLoggedError != "No Visible Tags") {
          // Logger.info("Vision Reject : No Visible Tags", "");
          lastLoggedError = "No Visible Tags";
        }
      } else if (cameraData.megaTag2.getDistanceToClosestSeenTarget() > 4.75) {
        doRejectUpdate = true;
        if (lastLoggedError != "Closest Tag Too Far") {
          // Logger.info("Vision Reject : No Swerve Drive", "");
          lastLoggedError = "Closet Tag Too Far";
      } 
    }
       else if (sd == null) {
        doRejectUpdate = true;
        if (lastLoggedError != "No Swerve Drive") {
          // Logger.info("Vision Reject : No Swerve Drive", "");
          lastLoggedError = "No Swerve Drive";
        }
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
        if (lastLoggedError != "No Error") {
          // Logger.info("Vision Reject: No Error", "");
          lastLoggedError = "No Error";
        }

        double distanceError = sd.getPose().getTranslation()
            .getDistance(cameraData.megaTag2.poseEstimate.pose.getTranslation());

        double translationStdDev = Math.min(50.0, Math.max(0.4, distanceError * 2.0));
        // double rotationStdDev = Math.min(50.0, Math.max(0.3, distanceError * 0.3));

        // sd.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));//originally
        // .7, .7, 9999999
        sd.setVisionMeasurementStdDevs(VecBuilder.fill(translationStdDev, translationStdDev, 9999999));

        sd.addVisionMeasurement(cameraData.megaTag2.poseEstimate.pose,
            cameraData.megaTag2.poseEstimate.timestampSeconds);
      }
    }

    if (RobotContainer.swerveSubsystem != null) {
      SmartDashboard.putNumber("frc3620/vision/nearestTagID",
          getNearestTagID(RobotContainer.swerveSubsystem.getPose()));
    }
  }

  public CameraData getCameraData(Camera camera) {
    return allCameraData.get(camera);
  }

  public Set<CameraData> getAllCameraData() {
    return allCameraDataAsSet;
  }

  public int getNearestTagID(Pose2d pose) {
    Translation2d translation = pose.getTranslation();
    Translation2d nearestTagTranslation = translation.nearest(tagTranslations);

    if (translation.getDistance(centerBlueReef) < maxDistanceFromCenterToBeClose) {
      return translationToTagMap.get(nearestTagTranslation);
    } else {
      return -1;
    }
  }

  public Pose2d getNearestLeftStickPose(int tagID) {
    if (tagID == -1) {
      return RobotContainer.swerveSubsystem.getPose();
    } else {

      return tagToStickPose2dLeft.get(tagID);

    }
  }

  public Pose2d getNearestRightStickPose(int tagID) {
    if (tagID == -1) {
      return RobotContainer.swerveSubsystem.getPose();
    } else {

      return tagToStickPose2dRight.get(tagID);

    }
  }

}
