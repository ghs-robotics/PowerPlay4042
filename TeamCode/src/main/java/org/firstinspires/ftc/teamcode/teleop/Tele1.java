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
    private final float dpadInputScaler = 0.5f; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.5f; // 0.5 = no effect | 0.0 = max effect

    private Pose2d tempInputScaler = new Pose2d(0.75, 0.75, 0.6);

    private Vector2D targetPos = new Vector2D(0, 0);

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        bot.arm.resetLiftPos(!opModeIsActive());
        while (opModeIsActive()){
            //reset lift at start
            double sec = runtime.seconds();
            //boolean release = sec < 3;

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 1           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //GET INPUT
            Pose2d input = GetInput();
            /*Pose2d input = new Pose2d(
                gamepad1.left_stick_x * tempInputScaler.getX(),
                gamepad1.left_stick_y * tempInputScaler.getY(),
                gamepad1.right_stick_x * tempInputScaler.getHeading()
            );*/

            //MOVEMENT
            bot.smd.setWeightedDrivePower(
               new Pose2d(-input.getY(), input.getX(), input.getHeading())
            );

            /*//dpad movement
            if (gamepad1.dpad_up)
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1, 0 ), bot.smd );

            if (gamepad1.dpad_down)
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( -1, 0 ), bot.smd );

            if (gamepad1.dpad_left)
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 0, 1 ), bot.smd );

            if (gamepad1.dpad_right)
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 0, -1 ), bot.smd );*/

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //run arm to position and release cone
            //reset, low, middle, high
            bot.arm.runLiftToPos(gamepad2.b, gamepad2.a, gamepad2.x, gamepad2.y);

            //ARM MOVEMENT
            bot.arm.driveArm(-gamepad2.left_stick_y);

            //GRIPPER MOVEMENT
            bot.arm.runGripper(gamepad2.left_bumper, gamepad2.right_bumper);

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            Pose2d estimate = bot.smd.getPoseEstimate();

            telemetry.addData("lift 1", bot.arm.liftMotor1.getCurrentPosition());
            telemetry.addData("lift 2", bot.arm.liftMotor2.getCurrentPosition());

//            telemetry.addData("targetPosX:", targetPos.getX());
//            telemetry.addData("targetPosY:", targetPos.getY());
//
//            telemetry.addData("horizontalInput", input.getY());
//            telemetry.addData("verticalInput", input.getX());
//            telemetry.addData("rotationInput", input.getHeading());
            //telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);

            //telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            //telemetry.addData("gamepad2.right_stick_x", gamepad2.right_stick_x);
            telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);

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