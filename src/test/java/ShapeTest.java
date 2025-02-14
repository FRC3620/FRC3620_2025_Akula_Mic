import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;
import org.junit.jupiter.api.Assertions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ShapeBoundary;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class ShapeTest {
    List<Translation2d> vertices = new ArrayList<>();
    ShapeBoundary shape = new ShapeBoundary();

    public ShapeTest() {
        // Define the vertices of the rectangle
        vertices.add(new Translation2d(0, 0)); // Bottom-left corner
        vertices.add(new Translation2d(0, 5)); // Top-left corner
        vertices.add(new Translation2d(10, 5)); // Top-right corner
        vertices.add(new Translation2d(10, 0)); // Bottom-right corner
    }

    @Test
    public void do01() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 3), vertices));
    }

    @Test
    public void do02() {

        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 30), vertices));
    }

    @Test
    public void do03() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(500, 3), vertices));
    }

    @Test
    public void do04() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 300), vertices));
    }

    @Test
    public void do05() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 3), vertices));
    }

    @Test
    public void do06() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 3), vertices));
    }

    @Test
    public void do07() {
        Assertions.assertFalse(shape.isPointInside(new Translation2d(50, 3), vertices));
    }

    @Test
    public void do08() {
        Assertions.assertTrue(shape.isPointInside(new Translation2d(5, 3), vertices));
    }

    @Test
    public void do09() {
        Assertions.assertTrue(shape.isPointInside(new Translation2d(5, 1), vertices));
    }

    @Test
    public void do10() {
        Assertions.assertTrue(shape.isPointInside(new Translation2d(6, 3), vertices));

    }
}
