import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.junit.Test;
import org.usfirst.frc3620.Utilities;

public class MeasureClamp {

    // @Test
    public void testDistance() {
        System.out.println(Utilities.clamp(Inches.of(0), Meters.of(1), Feet.of(10)));
        System.out.println(Utilities.clamp(Inches.of(40), Meters.of(1), Feet.of(10)));
        System.out.println(Utilities.clamp(Inches.of(400), Meters.of(1), Feet.of(10)));
    }

    @Test
    public void testAngle() {
        System.out.println(Utilities.clamp(Radians.of(0), Degrees.of(360), Rotations.of(1.5)));
        System.out.println(Utilities.clamp(Radians.of(7), Degrees.of(360), Rotations.of(1.5)));
        System.out.println(Utilities.clamp(Radians.of(14), Degrees.of(360), Rotations.of(1.5)));
    }
}
