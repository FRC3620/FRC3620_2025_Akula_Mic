import static edu.wpi.first.units.Units.Degrees;

import org.junit.Test;

public class AngleTests {

    @Test
    public void curious() {
        System.out.println (Degrees.of(0).toLongString());
        System.out.println (Degrees.ofBaseUnits(0).toLongString());
    }
}
