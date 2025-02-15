import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.Test;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.RepeatedTest;

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

    Random r = new Random();

    @DisplayName("Triangle Monte Carlo test")
	@RepeatedTest(value = 100, name = "{displayName} - repetition {currentRepetition} of {totalRepetitions}")
    public void triangleTest() {
        List<Translation2d> triangle = ShapeTestUtilities.randomTriangle(r);
        double area = areaOfTriangle(triangle);

        double rayCastingHits = 0, hits = 0, totalThrows = 0;

        for (int i = 0; i < 100000; i++) {
            Translation2d p = new Translation2d(r.nextDouble(), r.nextDouble());
            boolean inside = shape.isPointInside(p, triangle);
            totalThrows++;
            if (inside) hits++;

            boolean rayCastInside = isPointInPolygon(p, triangle); 
            if (rayCastInside) rayCastingHits++;

            Assertions.assertEquals(inside, rayCastInside, () -> "triangle = " + triangle + ", point = " + p + ", inside = " + inside + ", rayCastingInside = " + rayCastInside);
        }

        double ratio = (double) hits / (double) totalThrows;
        double rayCastingRatio = (double) rayCastingHits / (double) totalThrows;
        
        System.out.println ("area = " + area + ", tested ratio = " + ratio + ", rayCastingRatio = " + rayCastingRatio);
        Assertions.assertEquals(area, rayCastingRatio, 0.005);
    }

    List<Translation2d> randomTriangle(long seed) {
        Random r = new Random(seed);
        ArrayList<Translation2d> rv = new ArrayList<>();
        rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
        rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
        rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
        return rv;
    }

    double areaOfTriangle(List<Translation2d> p) {
        // https://stackoverflow.com/a/42092643/27304527
        Double[] x = p.stream().map(Translation2d::getX).collect(Collectors.toList()).toArray(new Double[0]);
        Double[] y = p.stream().map(Translation2d::getY).collect(Collectors.toList()).toArray(new Double[0]);

        // https://www.cuemath.com/geometry/area-of-triangle-in-coordinate-geometry/
        return 0.5 * Math.abs(x[0] * (y[1] - y[2]) + x[1] * (y[2] - y[0]) + x[2] * (y[0] - y[1]));
    }

    public static boolean isPointInPolygon(Translation2d point, List<Translation2d> polygon) {
        int intersections = 0;
        int n = polygon.size();

        for (int i = 0; i < n; i++) {
            Translation2d p1 = polygon.get(i);
            Translation2d p2 = polygon.get((i + 1) % n);

            if (p1.getY() == p2.getY()) continue; // Skip horizontal edges

            if (point.getY() < Math.min(p1.getY(), p2.getY())) continue; // Point is below the edge
            if (point.getY() >= Math.max(p1.getY(), p2.getY())) continue; // Point is above the edge

            double xIntersection = (point.getY() - p1.getY()) * (p2.getX() - p1.getX()) / (p2.getY() - p1.getY()) + p1.getX();

            if (p1.getX() == p2.getX() || point.getX() <= xIntersection) {
                intersections++;
            }
        }

        return (intersections % 2) != 0;
    }

}
