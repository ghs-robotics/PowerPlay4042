package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Tele1 extends LinearOpMode {

    //Robot robot = new Robot(hardwareMap, telemetry);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //reset lift at start
            double sec = runtime.seconds();
            boolean release = sec < 3;

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 1           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            //driving

            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);

            telemetry.addData("gamepad2.a", gamepad2.a);
            telemetry.addData("gamepad2.y", gamepad2.y);
            /*telemetry.addData("z axis", angles.firstAngle);
            telemetry.addData("y axis", angles.secondAngle);
            telemetry.addData("x axis", angles.thirdAngle);
            telemetry.addData("shooter power variable", shooterPower); */
            telemetry.update();
        }
    }
}