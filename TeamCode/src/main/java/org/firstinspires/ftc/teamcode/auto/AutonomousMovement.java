package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class AutonomousMovement {
    public static final Vector2D ArenaDimensions = new Vector2D( 144, 144 );
    public static final Vector2D TileDimensions = new Vector2D( 24, 24 );
    public static final Vector2D TileNumber =
        new Vector2D(
            ArenaDimensions.getX() / TileDimensions.getX(),
            ArenaDimensions.getY() / TileDimensions.getY()
        );

    public static final double StopDistThreshold = 0.1;

    public static Vector2D TileCords( Vector2D index, Vector2D percentInTile ) {
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
    public static void MoveToPos( Vector2D target ) {
        Pose2d crntPos = new Pose2d( 0, 0, 0 );
        float spd = 1; //Could be modified throughout the function to smoothly stop and start.

        int axesMovedOn = 0;
        boolean moveOnXAxis = false;

        //Check if X axis of the robot is further from a poll
        if ( Math.abs( ( Math.abs( crntPos.getX() ) % TileDimensions.getX() ) - ( TileDimensions.getX() / 2 ) ) <
            Math.abs( ( Math.abs( crntPos.getY() ) % TileDimensions.getY() ) - ( TileDimensions.getY() / 2 ) ) ) {
            moveOnXAxis = true;
        }

        while ( axesMovedOn < 2) {
            crntPos = new Pose2d( 0, 0, 0 );

            if ( moveOnXAxis ) {
                double dif = target.getX() - crntPos.getX();
                if ( dif > StopDistThreshold ) {
                    new Pose2d( Math.signum( dif ) * spd, 0, 0 );
                    //Set velocity to this ^^^
                }
                else {
                    moveOnXAxis = false;
                    axesMovedOn++;
                }
            }
            else {
                double dif = target.getY() - crntPos.getY();
                if ( dif > StopDistThreshold ) {
                    new Pose2d( 0, Math.signum( dif ) * spd, 0 );
                    //Set velocity to this ^^^
                }
                else {
                    moveOnXAxis = true;
                    axesMovedOn++;
                }
            }
        }
    }
}
