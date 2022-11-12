package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;

@TeleOp
public class Tele1 extends LinearOpMode {
    //Input Variables
    private final float dpadInputScaler = 1; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.1f; // 0.5 = no effect | 0.0 = max effect

    private Vector2D targetPos = new Vector2D( 0, 0 );

    //Robot robot = new Robot(hardwareMap, telemetry);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Robot robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive robot = new SampleMecanumDrive( hardwareMap );

        boolean switchDrive = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 1           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //get input
            Pose2d input = GetInput();

            //robot.setWeightedDrivePower(new Pose2d(-input.getX(), input.getY(), input.getHeading()));

            robot.calculateMetaDrive(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
            /*//switch drive
            if (gamepad1.a)
                switchDrive = !switchDrive;

            //driving
            if (switchDrive)
                robot.metaDrivePower(hInput, vInput, rInput);
            else
                robot.setWeightedDrivePower(new Pose2d(-hInput, vInput, rInput));*/


            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            Pose2d estimate = robot.getPoseEstimate();

            telemetry.addData("targetPosX:", targetPos.getX() );
            telemetry.addData("targetPosY:", targetPos.getY() );

            telemetry.addData("x pos:", estimate.getX());
            telemetry.addData("y pos:", estimate.getY());
            telemetry.addData("heading", Math.toDegrees(estimate.getHeading()));
            telemetry.addData("switch ", switchDrive);
            telemetry.addLine();

            telemetry.addData("horizontalInput", input.getX());
            telemetry.addData("verticalInput", input.getY());
            telemetry.addData("rotationInput", input.getHeading());

            //telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            //telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            //telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

            robot.update();
            telemetry.update();
        }
    }
    private Pose2d GetInput() {
        double hAxis = 0;
        double vAxis = 0;
        double rAxis = 0;

        if ( gamepad1.dpad_right ) hAxis++;
        if ( gamepad1.dpad_left ) hAxis--;

        if ( gamepad1.dpad_up ) vAxis++;
        if ( gamepad1.dpad_down ) vAxis--;

        if ( hAxis == 0 && vAxis == 0 ) {
            hAxis = gamepad1.left_stick_x;
            vAxis = gamepad1.left_stick_y;

            double mag = Math.hypot( hAxis, vAxis );
            double curveMag = LinearBezierY( mag );

            hAxis /= mag;
            vAxis /= mag;

            hAxis *= curveMag;
            vAxis *= curveMag;
        }
        else {
            hAxis *= dpadInputScaler;
            vAxis *= dpadInputScaler;
        }

        rAxis = LinearBezierY( gamepad1.right_stick_x );

        return new Pose2d( hAxis, vAxis, rAxis );
    }
    private double LinearBezierY( double t ){
        //Uses the Y coordinates of 3 points to solve for the Y coordinate along the linear bezier curve at percentage "t"
        double negativeValue = 1;
        if ( t < 0 ) {
            t *= -1;
            negativeValue = -1;
        }
        if ( t > 1) t = 1;

        float y1 = 0;
        float y2 = bezierP2Y;
        float y3 = 1;

        double oneMinusT = 1 - t;
        return negativeValue * ( ( oneMinusT * oneMinusT * y1 ) + ( 2 * oneMinusT * t * y2 ) + ( t * t * y3 ) );
    }
}