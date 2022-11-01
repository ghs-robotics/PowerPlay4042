package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;

public class AutonomousMovement {
    private final Vector2D TileDimensions = new Vector2D(23.5f, 23.5f);
    private final Vector2D TileNumber = new Vector2D(6, 6);
    private final Vector2D ArenaDimensions =
        new Vector2D(
            TileDimensions.getX() * TileNumber.getX(),
            TileDimensions.getY() * TileNumber.getY()
        );

    private final double MoveToSpd = 0.6;
    private final double MoveToSlowDist = 3;
    private final double MoveToSlowSpd = 0.35;
    private final double MoveToStopDist = 0.1;

    public Vector2D TileCords( Vector2D index, Vector2D percentInTile ) {
        //index from ( 0, 0 ) to ( 5, 5 )
        Vector2D bottomLeftCorner = new Vector2D(
                ( index.getX() * TileDimensions.getX() ) - ( ArenaDimensions.getX() / 2 ),
                ( index.getY() * TileDimensions.getY() ) - ( ArenaDimensions.getY() / 2 )
        );
        Vector2D distInTile = new Vector2D(
                percentInTile.getX() * TileDimensions.getX(),
                percentInTile.getY() * TileDimensions.getY()
        );
        return new Vector2D(
                bottomLeftCorner.getX() + distInTile.getX(),
                bottomLeftCorner.getY() + distInTile.getY()
        );
    }
    public Vector2D RelativeToGlobalPos( Vector2D tileDist, SampleMecanumDrive smd ) {
        Pose2d crntPos = smd.getPoseEstimate();
        return new Vector2D(
                crntPos.getX() + tileDist.getX() * TileDimensions.getX(),
                crntPos.getY() + tileDist.getY() * TileDimensions.getY()
        );
    }
    public void MoveToPos(Vector2D target, SampleMecanumDrive smd, Telemetry telemetry ) {
        Pose2d crntPos = smd.getPoseEstimate();
        double crntSpd = MoveToSpd;

        int axesMovedOn = 0;
        boolean moveOnXAxis = false;

        //Check if Y axis of the robot is further from a poll
        if ( Math.abs( ( Math.abs( crntPos.getX() ) % TileDimensions.getX() ) - ( TileDimensions.getX() / 2 ) ) >
                Math.abs( ( Math.abs( crntPos.getY() ) % TileDimensions.getY() ) - ( TileDimensions.getY() / 2 ) ) ) {
            moveOnXAxis = false;
        }

        while ( axesMovedOn < 2 ) {
            crntPos = smd.getPoseEstimate();

            if ( moveOnXAxis ) {
                double dif = target.getX() - crntPos.getX();
                double absDif = Math.abs( dif );

                if ( absDif > MoveToStopDist) {
                    if ( absDif <= MoveToSlowDist ) crntSpd = MoveToSlowSpd;
                    smd.setWeightedDrivePower( new Pose2d( Math.signum( dif ) * crntSpd, 0, 0 ) );
                }
                else {
                    moveOnXAxis = false;
                    crntSpd = MoveToSpd;
                    axesMovedOn++;
                }
            }
            else {
                double dif = target.getY() - crntPos.getY();
                double absDif = Math.abs( dif );

                if ( Math.abs( dif ) > MoveToStopDist) {
                    if ( absDif <= MoveToSlowDist ) crntSpd = MoveToSlowSpd;
                    smd.setWeightedDrivePower( new Pose2d( 0, -Math.signum( dif ) * crntSpd, 0 ) );
                }
                else {
                    moveOnXAxis = true;
                    crntSpd = MoveToSpd;
                    axesMovedOn++;
                }
            }

            Pose2d estimate = smd.getPoseEstimate();

            telemetry.addData("x pos:", estimate.getX());
            telemetry.addData("y pos:", estimate.getY());
            telemetry.addData("heading", Math.toDegrees(estimate.getHeading()));

            smd.update();
            telemetry.update();
        }
    }
}
