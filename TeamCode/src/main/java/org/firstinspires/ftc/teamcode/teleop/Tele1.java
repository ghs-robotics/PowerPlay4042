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
    private final float dpadInputScaler = 0.35f; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.5f; // 0.5 = no effect | 0.0 = max effect

    private Pose2d inputScaler = new Pose2d(0.75, 0.75, 0.75);
    private double YToXMovementRatio = 0.8;

    private Vector2D targetPos = new Vector2D(0, 0);

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, telemetry);

        //false = non-meta drive
        boolean driveType = false;

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

            Controller1(bot);

            /*//to change Drive type
            if (gamepad1.a)
                driveType = !driveType;

            //STANDARD DRIVE - inputs need to be updated, ran out of time to correct
            if (driveType)
                bot.smd.setWeightedDrivePower(
                        new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
                //META DRIVE - inputs need to be updated, ran out of time to correct
            else
                bot.smd.calculateMetaDrive(
                        new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                );*/

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            Controller2(bot);

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            /*Pose2d estimate = bot.smd.getPoseEstimate();

            //telemetry.addData("gripper status: ", bot.arm.gripperStatus());
            telemetry.addData("lift 1 pos", bot.arm.liftMotor1.getCurrentPosition());
            telemetry.addData("lift 2 pos", bot.arm.liftMotor2.getCurrentPosition());
            telemetry.addData("drive mode: ", bot.smd.returnDriveType(driveType));

            telemetry.update();*/
        }
    }
    private void Controller1(Robot bot) {
        //GET INPUT
        Pose2d input = GetInput();
        Pose2d scaledInput = new Pose2d(
                input.getX() * inputScaler.getX(),
                input.getY() * inputScaler.getY(),
                input.getHeading() * inputScaler.getHeading()
        );
        //convert global input direction to local robot direction
        Pose2d localDir = GetLocalDir(
                new Pose2d(-scaledInput.getY(), scaledInput.getX(), scaledInput.getHeading()),
                bot
        );
        //scale X axis so that irl movement speed on X and Y axis is that same
        bot.smd.setWeightedDrivePower(
                new Pose2d(localDir.getX() * YToXMovementRatio, localDir.getY(), localDir.getHeading())
        );
    }
    private void Controller2(Robot bot) {
        //run arm to position and release cone
        //reset, low, middle, high
        bot.arm.runLiftToPos(gamepad2.b, gamepad2.a, gamepad2.x, gamepad2.y);
        boolean useDriveArm = gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.y;

        //ARM MOVEMENT - won't work while running arm to position
        if (!useDriveArm) bot.arm.driveArm(-gamepad2.left_stick_y);

        //GRIPPER MOVEMENT - bumpers for full range, triggers for 20 deg
        boolean lTrig = gamepad2.left_trigger > 0.5;
        boolean rTrig = gamepad2.right_trigger > 0.5;
        bot.arm.runGripperContinuous(gamepad2.left_bumper, gamepad2.right_bumper);
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

            /*double mag = Math.hypot( hAxis, vAxis );
            double curveMag = LinearBezierY( mag );

            hAxis /= mag;
            vAxis /= mag;

            hAxis *= curveMag;
            vAxis *= curveMag;*/
        }
        else {
            hAxis *= dpadInputScaler;
            vAxis *=-dpadInputScaler;
        }

        rAxis = gamepad1.right_stick_x;
        //rAxis = LinearBezierY( gamepad1.right_stick_x );

        return new Pose2d(hAxis, vAxis, rAxis);
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
    private Pose2d GetLocalDir( Pose2d globalDir, Robot bot ) {
        double crntHeadingRad = Math.toRadians(bot.smd.getPoseEstimate().getHeading());

        Vector2D rotatedVector = new Vector2D(
                globalDir.getX() * Math.cos(crntHeadingRad) - globalDir.getY() * Math.sin(crntHeadingRad),
                globalDir.getX() * Math.sin(crntHeadingRad) + globalDir.getY() * Math.cos(crntHeadingRad)
        );
        return new Pose2d(rotatedVector.getX(), rotatedVector.getY(), globalDir.getHeading());
    }
}