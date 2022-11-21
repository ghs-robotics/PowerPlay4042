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

    private Pose2d inputScaler = new Pose2d(0.8, 0.8, 0.6);
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

            Pose2d movementVector = bot.inputMan.HandleController1Input(gamepad1);

            bot.smd.setWeightedDrivePower(movementVector);

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            bot.inputMan.HandleController2Input(gamepad2);

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            bot.smd.update();
            Pose2d estimatedPos = bot.smd.getPoseEstimate();

            telemetry.addData("PosX: ", estimatedPos.getX());
            telemetry.addData("PosY: ", estimatedPos.getY());
            telemetry.addData("PosHeading: ", estimatedPos.getHeading());

            telemetry.update();
        }
    }
}