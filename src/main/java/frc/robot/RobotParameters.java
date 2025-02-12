package frc.robot;

import org.usfirst.frc3620.RobotParametersBase;

public class RobotParameters extends RobotParametersBase {
    String swerveDirectoryName;

    double intakeFrontEncoderOffset = 0;
    double intakeRearEncoderOffset = 0;

    public String getSwerveDirectoryName() {
        return swerveDirectoryName;
    }

    public double getIntakeFrontEncoderOffset() {
        return intakeFrontEncoderOffset;
    }

    public double getIntakeRearEncoderOffset() {
        return intakeRearEncoderOffset;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveDirectoryName=" + swerveDirectoryName);
        sb.append (", intakeFrontEncoderOffset=" + intakeFrontEncoderOffset);
        sb.append (", intakeRearEncoderOffset=" + intakeRearEncoderOffset);
        sb.append ("]");
        return sb.toString();
    }


}