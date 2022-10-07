package org.firstinspires.ftc.teamcode.auto;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class AutonomousMovement {
    public static final Vector2D ArenaDimensions = new Vector2D( 144, 144 );
    public static final Vector2D TileDimensions = new Vector2D( 24, 24 );
    public static final Vector2D TileNumber =
        new Vector2D(
            ArenaDimensions.getX() / TileDimensions.getX(),
            ArenaDimensions.getY() / TileDimensions.getY()
        );

    public static Vector2D TileCords( Vector2D index, Vector2D percentInTile ) {
        //index from ( 0, 0 ) to ( 5, 5 )
        Vector2D bottomLeftCorner = new Vector2D(
            index.getX() * ArenaDimensions.getX() / TileNumber.getX(),
            index.getY() * ArenaDimensions.getY() / TileNumber.getY()
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
}
