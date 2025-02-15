import java.util.*;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;

public class ShapeTestUtilities {
  
  public static List<Translation2d> randomTriangle(Random r) {
    ArrayList<Translation2d> rv = new ArrayList<>();
    rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
    rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
    rv.add(new Translation2d(r.nextDouble(), r.nextDouble()));
    return rv;
}

  public static double areaOfTriangle(List<Translation2d> p) {
    // https://stackoverflow.com/a/42092643/27304527
    Double[] x = p.stream().map(Translation2d::getX).collect(Collectors.toList()).toArray(new Double[0]);
    Double[] y = p.stream().map(Translation2d::getY).collect(Collectors.toList()).toArray(new Double[0]);

    // https://www.cuemath.com/geometry/area-of-triangle-in-coordinate-geometry/
    return 0.5 * Math.abs(x[0] * (y[1] - y[2]) + x[1] * (y[2] - y[0]) + x[2] * (y[0] - y[1]));
  }

}
