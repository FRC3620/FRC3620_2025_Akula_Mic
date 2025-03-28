package frc.robot;

import org.usfirst.frc3620.RobotParametersBase;

import com.fasterxml.jackson.annotation.JsonProperty;

public class RobotParameters extends RobotParametersBase {
    String swerveDirectoryName;

    double intakeFrontEncoderOffset = 0;
    double intakeRearEncoderOffset = 0;
    double climberEncoderOffset = 0;
    @JsonProperty("shortLEDStrip") boolean shortLEDStrip = false;

    //Camera Data
    String cameraNameFront; 
    String cameraNameBack;
    double frontCameraFront = 0;
    double frontCameraRight = 0;
    double frontCameraUp = 0;
    double frontCameraRoll = 0;
    double frontCameraPitch = 0;
    double frontCameraYaw = 0;
    double backCameraFront = 0;
    double backCameraRight = 0;
    double backCameraUp = 0;
    double backCameraRoll = 0;
    double backCameraPitch = 0;
    double backCameraYaw = 0;

    public String getSwerveDirectoryName() {
        return swerveDirectoryName;
    }

    public double getIntakeFrontEncoderOffset() {
        return intakeFrontEncoderOffset;
    }

    public double getIntakeRearEncoderOffset() {
        return intakeRearEncoderOffset;
    }

    public double getClimberEncoderOffset() {
        return climberEncoderOffset; 
    }

    public boolean isShortLEDStrip() {
        return shortLEDStrip;
    }

    public String getFrontCameraName()  { return cameraNameFront; }
    public String getBackCameraName()   { return cameraNameBack; }

    public double getFrontCameraFront() { return frontCameraFront; }
    public double getFrontCameraRight() { return frontCameraRight; }
    public double getFrontCameraUp()    { return frontCameraUp; }
    public double getFrontCameraRoll()  { return frontCameraRoll; }
    public double getFrontCameraPitch() { return frontCameraPitch; }
    public double getFrontCameraYaw()   { return frontCameraYaw; }

    public double getBackCameraFront() { return backCameraFront; }
    public double getBackCameraRight() { return backCameraRight; }
    public double getBackCameraUp()    { return backCameraUp; }
    public double getBackCameraRoll()  { return backCameraRoll; }
    public double getBackCameraPitch() { return backCameraPitch; }
    public double getBackCameraYaw()   { return backCameraYaw; }






    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveDirectoryName=" + swerveDirectoryName);
        sb.append (", intakeFrontEncoderOffset=" + intakeFrontEncoderOffset);
        sb.append (", intakeRearEncoderOffset=" + intakeRearEncoderOffset);
        sb.append(", climberEncoderOffset=" + climberEncoderOffset);
        sb.append ("]");
        return sb.toString();
    }


}