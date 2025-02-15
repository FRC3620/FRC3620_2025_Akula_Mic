package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class ShapeBoundary {
    private List<Translation2d> vertices;

    public ShapeBoundary() {
        this.vertices = vertices;
    }

    public List<Translation2d> getVertices() {
        return vertices;
    }

    public boolean isPointInside(Translation2d point,List<Translation2d> vertices) {
        int windingNumber = 0;
        int numVertices = vertices.size();
        
        for (int i = 0; i < numVertices; i++) {
            Translation2d v1 = vertices.get(i);
            Translation2d v2 = vertices.get((i + 1) % numVertices);
            
            if (v1.getY() <= point.getY()) {
                if (v2.getY() > point.getY() && isLeft(v1, v2, point) > 0) {
                    windingNumber++;
                }
            } else {
                if (v2.getY() <= point.getY() && isLeft(v1, v2, point) < 0) {
                    windingNumber--;
                }
            }
        }
        
        return windingNumber != 0; // Nonzero means inside
    }

    private double isLeft(Translation2d a, Translation2d b, Translation2d p) {
        return (b.getX() - a.getX()) * (p.getY() - a.getY()) - (b.getY() - a.getY()) * (p.getX() - a.getX());
    }
}
