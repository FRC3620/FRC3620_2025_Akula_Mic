package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ESEFMech {
	static final double kShoulderLength = Units.inchesToMeters(18);
	
	// Create a Mechanism2d visualization of ESEF
	private final Mechanism2d m_mech2d = new Mechanism2d(
			3 * kShoulderLength,
			(kShoulderLength + ESEFElevatorMechanism.kElevatorMaxHeightMeters) * 1.1);
	private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ESEF", 1.5 * kShoulderLength, 0);
	private final MechanismLigament2d elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d(
			"Elevator",
			ESEFElevatorMechanism.kElevatorMinHeightMeters,
			90,
			40,
			new Color8Bit(Color.kBlue)));
	private final MechanismLigament2d shoulderMech2d = elevatorMech2d.append(new MechanismLigament2d(
			"Shoulder",
			kShoulderLength,
			ESEFShoulderMechanism.kShoulderDefaultSetpointDegrees,
			20,
			new Color8Bit(Color.kYellow)));

	public ESEFMech() {
	}

	public void setElevatorHeight(Distance d) {
		elevatorMech2d.setLength(d.in(Meters));
	}

	public void setShoulderAngle(Angle a) {
		shoulderMech2d.setAngle(a.in(Degrees) - 90);
	}

	public Mechanism2d getMech() {
		return m_mech2d;
	}

}
