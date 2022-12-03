package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class DriveBase {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    private final Vector2D TileDimensions = new Vector2D(23.5f, 23.5f);
    private final Vector2D TileNumber = new Vector2D(6.0, 6.0);
    private final Vector2D ArenaDimensions =
            new Vector2D(
                    TileDimensions.getX() * TileNumber.getX(),
                    TileDimensions.getY() * TileNumber.getY()
            );

    private final double MoveToSpd = 0.5;//0.45;
    private final double MoveToSlowDistStart = 0.75;
    private final double MoveToSlowDistEnd = 6;
    private final double MoveToSlowSpd = 0.3;//0.2;
    private final double MoveToStopDist = 0.15;


    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRDrive");

    }
    public void calculateDrivePower(double x, double y, double rot){
        rot = -rot;
        double frontLeft = rot - x + y;
        double backLeft = rot + x + y;
        double frontRight = rot - x - y;
        double backRight = rot + x - y;

        sendDrivePower(frontLeft, backLeft, frontRight, backRight);
    }

    public void sendDrivePower(double frontLeft, double backLeft, double frontRight, double backRight){
        //reversed to match motor polarity or something
        frontLeftDrive.setPower(frontLeft);
        backLeftDrive.setPower(backLeft);
        frontRightDrive.setPower(frontRight);
        backRightDrive.setPower(backRight);
    }

    public Vector2D TileCords( Vector2D tile ) {
        //index from ( 0, 0 ) to ( 6, 6 )
        return new Vector2D(
                ( tile.getX() * TileDimensions.getX() ) - ( ArenaDimensions.getX() / 2 ),
                ( tile.getY() * TileDimensions.getY() ) - ( ArenaDimensions.getY() / 2 )
        );
    }
    public Vector2D RelativeToGlobalPos( Vector2D tileDist, SampleMecanumDrive smd ) {
        Pose2d crntPos = smd.getPoseEstimate();
        return new Vector2D(
                crntPos.getX() + tileDist.getX() * TileDimensions.getX(),
                crntPos.getY() + tileDist.getY() * TileDimensions.getY()
        );
    }
    public void MoveToPos(boolean moveOnXAxis, Vector2D target, SampleMecanumDrive smd, Telemetry telemetry ) {
        double crntSpd = MoveToSpd;

        smd.update();
        Pose2d crntPos = smd.getPoseEstimate();

        int axesMovedOn = 0;

        //Check if Y axis of the robot is further from a poll
        /*if ( Math.abs( ( Math.abs( crntPos.getX() ) % TileDimensions.getX() ) - ( TileDimensions.getX() / 2 ) ) >
                Math.abs( ( Math.abs( crntPos.getY() ) % TileDimensions.getY() ) - ( TileDimensions.getY() / 2 ) ) ) {
            moveOnXAxis = false;
        }*/

        double dif = moveOnXAxis ? target.getX() - crntPos.getX() : target.getY() - crntPos.getY();
        double absDif = Math.abs( dif );

        double startDif = absDif;
        while ( axesMovedOn < 2 ) {
            smd.update();
            crntPos = smd.getPoseEstimate();

            dif = moveOnXAxis ? target.getX() - crntPos.getX() : target.getY() - crntPos.getY();
            absDif = Math.abs( dif );

            if ( absDif > MoveToStopDist) {
                if ( absDif <= MoveToSlowDistEnd || absDif > startDif - MoveToSlowDistStart) crntSpd = MoveToSlowSpd;
                else crntSpd = MoveToSpd;

                if (moveOnXAxis) smd.setWeightedDrivePower( new Pose2d( Math.signum( dif ) * crntSpd, 0, 0 ) );
                else smd.setWeightedDrivePower( new Pose2d( 0, -Math.signum( dif ) * crntSpd, 0 ) );
            }
            else {
                moveOnXAxis = !moveOnXAxis;
                crntSpd = MoveToSpd;
                axesMovedOn++;

                dif = moveOnXAxis ? target.getX() - crntPos.getX() : target.getY() - crntPos.getY();
                absDif = Math.abs( dif );

                startDif = absDif;
            }
        }

        smd.setWeightedDrivePower(new Pose2d(0, 0, 0));

        smd.update();
        telemetry.update();
    }
    public void MoveAlongPath(boolean moveOnXAxis, ArrayList<Double> distances, SampleMecanumDrive smd, Telemetry telemetry ){
        for (double distance : distances ) {
            double crntSpd = MoveToSpd;

            smd.update();
            Pose2d crntPos = smd.getPoseEstimate();

            double startDist = moveOnXAxis ? crntPos.getX() : crntPos.getY();
            double targetDist = startDist + (distance * (moveOnXAxis ? TileDimensions.getX() : TileDimensions.getY()));

            double dif = moveOnXAxis ? targetDist - crntPos.getX() : targetDist - crntPos.getY();
            double absDif = Math.abs( dif );

            double startDif = absDif;
            while (absDif > MoveToStopDist){
                if ( absDif <= MoveToSlowDistEnd || absDif > startDif - MoveToSlowDistStart) crntSpd = MoveToSlowSpd;
                else crntSpd = MoveToSpd;

                Pose2d movePose;
                if (moveOnXAxis) movePose = new Pose2d(Math.signum(dif) * crntSpd, 0, 0);
                else movePose = new Pose2d(0, Math.signum(-dif) * crntSpd, 0);

                smd.setWeightedDrivePower( movePose );

                smd.update();
                crntPos = smd.getPoseEstimate();

                /*telemetry.addData("crntPosX: ", crntPos.getX());
                telemetry.addData("crntPosY: ", crntPos.getY());
                telemetry.addData("crntPosHeading: ", crntPos.getHeading());*/

                dif = moveOnXAxis ? targetDist - crntPos.getX() : targetDist - crntPos.getY();
                absDif = Math.abs( dif );

                telemetry.update();
            }

            moveOnXAxis = !moveOnXAxis;
        }

        smd.setWeightedDrivePower(new Pose2d(0, 0, 0));

        smd.update();
        telemetry.update();
    }
}
