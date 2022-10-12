package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class Tele1 extends LinearOpMode {

    //Robot robot = new Robot(hardwareMap, telemetry);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 1           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //driving

            robot.calculateDrivePower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            Pose2d estimate = robot.getPoseEstimate();

            telemetry.addData("x pos:", estimate.getX());
            telemetry.addData("y pos:", estimate.getY());
            telemetry.addData("heading", Math.toDegrees(estimate.getHeading()));

            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

            telemetry.update();
        }
    }
}