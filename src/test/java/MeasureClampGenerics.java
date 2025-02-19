import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.junit.Test;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

public class MeasureClampGenerics {
    /*

    static <T extends Measure> T clamp (T v, T min, T max) {
        System.out.println("checking " + v);
        if (v.lt(min)) {
            return fixup(min, v.unit());
        } else if (v.gt(max)) {
            return fixup(max, v.unit());
        }
        return v;
    }

    static <T extends Measure<?>, TU extends Unit> T fixup (T v, TU u) {
        if (v.baseUnit() == u) {
            System.out.println (v + " " + u);
            return v;
        }

        double mag = v.in(u);
        Measure<?> rv = u.of(mag);
        System.out.println (v + " " + mag + " " + u + " " + rv);
        return rv;
    }

    @Test
    public void test0() {
        System.out.println (clamp(Inches.of(0), Meters.of(1), Feet.of(10)));
        System.out.println (clamp(Inches.of(40), Meters.of(1), Feet.of(10)));
        System.out.println (clamp(Inches.of(400), Meters.of(1), Feet.of(10)));
    }

    */

}
