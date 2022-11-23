package org.firstinspires.ftc.teamcode.robot;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class AutonomousMovement {
    private final Vector2D TileDimensions = new Vector2D(23.5f, 23.5f);
    private final Vector2D TileNumber = new Vector2D(6.0, 6.0);
    private final Vector2D ArenaDimensions =
        new Vector2D(
            TileDimensions.getX() * TileNumber.getX(),
            TileDimensions.getY() * TileNumber.getY()
        );

    private final double MoveToSpd = 0.5;
    private final double MoveToSlowDist = 6;
    private final double MoveToSlowSpd = 0.25;
    private final double MoveToStopDist = 0.15;

    private final double ArmStopDist = 0.1;
}
