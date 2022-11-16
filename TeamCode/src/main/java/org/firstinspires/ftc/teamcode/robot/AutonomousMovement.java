package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;

import java.util.ArrayList;

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

    private final double GripperStopDist = 0.01;

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

        while ( axesMovedOn < 2 ) {
            smd.update();
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

            smd.setWeightedDrivePower(new Pose2d(0, 0, 0));

            smd.update();
            telemetry.update();
        }
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

            while (absDif > MoveToStopDist){
                if ( absDif <= MoveToSlowDist ) crntSpd = MoveToSlowSpd;

                Pose2d movePose;
                if (moveOnXAxis) movePose = new Pose2d(Math.signum(dif) * crntSpd, 0, 0);
                else movePose = new Pose2d(0, Math.signum(-dif) * crntSpd, 0);

                smd.setWeightedDrivePower( movePose );

                smd.update();
                crntPos = smd.getPoseEstimate();

                telemetry.addData("crntPosX: ", crntPos.getX());
                telemetry.addData("crntPosY: ", crntPos.getY());
                telemetry.addData("crntPosHeading: ", crntPos.getHeading());

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
    public void LiftToPos( int targetPos, Arm arm, Telemetry telemetry ) {
        DcMotorEx liftMotor1 = arm.liftMotor1;
        DcMotorEx liftMotor2 = arm.liftMotor2;

        int crntPos = liftMotor1.getCurrentPosition();
        int error = liftMotor2.getCurrentPosition() - arm.liftMotor2.getCurrentPosition();//add to telemetry?

        double dif = targetPos - crntPos;
        double absDif = Math.abs( dif );

        liftMotor1.setTargetPosition(targetPos);
        liftMotor2.setTargetPosition(targetPos); //account for error?

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.driveArm(1);//does this stay set or need called every frame

        while (absDif > ArmStopDist) {
            crntPos = liftMotor1.getCurrentPosition();

            dif = targetPos - crntPos;
            absDif = Math.abs( dif );
        }

        arm.driveArm(0);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void RotateGripper(boolean intake, Arm arm) {   //ADD CRSERVO COMPATIBILITY
/*        Servo gripServo = arm.gripServo;

        double targetPos = intake ? Servo.MIN_POSITION : Servo.MAX_POSITION;
        gripServo.setPosition(targetPos);

        double crntPos = gripServo.getPosition();

        double dif = targetPos - crntPos;
        double absDif = Math.abs( dif );

        while(absDif > GripperStopDist) {
            crntPos = gripServo.getPosition();

            dif = targetPos - crntPos;
            absDif = Math.abs( dif );*/
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 750) {
            if (intake)
                arm.runGripperContinuous(true, false);
            else
                arm.runGripperContinuous(false, true);
        }
    }
}
