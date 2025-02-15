import java.util.ArrayList;
import java.util.List;

import org.junit.Test;
import org.junit.jupiter.api.Assertions;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ShapeBoundary;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class ShapeTestOne {
    List<Translation2d> vertices = new ArrayList<>();
    ShapeBoundary shape = new ShapeBoundary();

	@Test
    public void triangleTest() {
        List<Translation2d> triangle = new ArrayList<>();
        triangle.add(new Translation2d(4.5, 2.5));
        triangle.add(new Translation2d(7.4, 8.4));
        triangle.add(new Translation2d(1.8, 8.2));

        Translation2d point = new Translation2d(3.2, 2.7);

        boolean inside = shape.isPointInside(point, triangle);
        System.out.println ("triangle = " + triangle + ", point = " + point + ", inside = " + inside);
        Assertions.assertFalse(inside);
    }
}
