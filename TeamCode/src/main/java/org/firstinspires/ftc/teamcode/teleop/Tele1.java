package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
//Want to try using Opmode instead of LinearOp since I heard this is better for TeleOp
public class Tele1 extends LinearOpMode {
    private final float dpadInputScaler = 1; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.1f; // 0.5 = no effect | 0.0 = max effect

    private Vector2D targetPos = new Vector2D(0, 0);

    Robot robot;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //reset lift at start
            double sec = runtime.seconds();
            //boolean release = sec < 3;

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 1           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //get input
            //Pose2d input = GetInput();
            Pose2d input = new Pose2d(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            //Movement
            bot.smd.setWeightedDrivePower(
                new Pose2d(-input.getY(), input.getX(), input.getHeading())
            );

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //Arm Movement
            bot.arm.driveArm(gamepad2.left_stick_y);

            //Gripper Movement
            bot.arm.gripper(gamepad2.left_bumper, gamepad2.right_bumper);

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            Pose2d estimate = bot.smd.getPoseEstimate();

            telemetry.addData("crntPosX:", estimate.getX());
            telemetry.addData("crntPosY:", estimate.getY());
            telemetry.addData("crntPosHeading:", estimate.getHeading());

            telemetry.addData("targetPosX:", targetPos.getX());
            telemetry.addData("targetPosY:", targetPos.getY());

            telemetry.addData("horizontalInput", input.getY());
            telemetry.addData("verticalInput", input.getX());
            telemetry.addData("rotationInput", input.getHeading());
            //telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);

            //telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            //telemetry.addData("gamepad2.right_stick_x", gamepad2.right_stick_x);
            //telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);

            //telemetry.addData("rotationInput", input.getX());
            //telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
            //telemetry.addData("horizontalInput", input.getY());
            //telemetry.addData("verticalInput", input.getHeading());

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