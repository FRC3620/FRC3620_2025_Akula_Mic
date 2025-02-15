// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class ShapeBoundary {

    // https://chat.deepseek.com/a/chat/s/b83653a5-875b-4310-a834-81804759d1cb

    private List<Translation2d> vertices;

    public ShapeBoundary() {
        vertices = new ArrayList<>();
        // Define the vertices of the rectangle
        vertices.add(new Translation2d(0, 0)); // Bottom-left corner
        vertices.add(new Translation2d(0, 5)); // Top-left corner
        vertices.add(new Translation2d(10, 5)); // Top-right corner
        vertices.add(new Translation2d(10, 0)); // Bottom-right corner
    }

    public List<Translation2d> getVertices() {
        return vertices;
    }

    public boolean isPointInside(Translation2d point, List<Translation2d> vertices) {

        int numVertices = vertices.size();
        boolean inside = false;

        // Loop through each edge of the polygon
        for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
            Translation2d vertex1 = vertices.get(i); // Current vertex
            Translation2d vertex2 = vertices.get(j); // Previous vertex

            // Check if the point is within the y-range of the edge
            if (((vertex1.getY() > point.getY()) != (vertex2.getY() > point.getY())) &&
            // Check if the point is to the left of the edge's intersection with the ray
                    (point.getX() < (vertex2.getX() - vertex1.getX()) * (point.getY() - vertex1.getY())
                            / (vertex2.getY() - vertex1.getY()) + vertex1.getX())) {
                inside = true;
            }
        }
        return inside;
    }
}
