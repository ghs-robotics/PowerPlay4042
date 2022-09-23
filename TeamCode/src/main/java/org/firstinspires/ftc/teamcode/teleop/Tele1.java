package org.firstinspires.ftc.teamcode.teleop;

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
        Robot drive = new Robot(hardwareMap, telemetry);

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

            //get input
            float hInput = GetAxis( 0 );
            float vInput = GetAxis( 1 );
            float rInput = GetAxis( 2 );

            //driving
            drive.calculateDrivePower(hInput, vInput, rInput);
            //////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////           Controller 2           ////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////           Telemetry           /////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("rotationInput", rInput);
            telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
            telemetry.addData("horizontalInput", hInput);
            telemetry.addData("verticalInput", vInput);
            telemetry.addData("gamepad2.a", gamepad2.a);
            telemetry.addData("gamepad2.y", gamepad2.y);
            /*telemetry.addData("z axis", angles.firstAngle);
            telemetry.addData("y axis", angles.secondAngle);
            telemetry.addData("x axis", angles.thirdAngle);
            telemetry.addData("shooter power variable", shooterPower); */
            telemetry.update();
        }
    }

    private float GetAxis( int axisType ) {
        // 0 = horizontal | 1 = vertical | 2 = rotational
        float axis = 0;
        switch ( axisType ){
            case 0:
                if ( gamepad1.dpad_right ) axis++;
                if ( gamepad1.dpad_left ) axis--;
                if ( axis == 0 ) axis = gamepad1.left_stick_x;
                break;

            case 1:
                if ( gamepad1.dpad_up ) axis++;
                if ( gamepad1.dpad_down ) axis--;
                if ( axis == 0 ) axis = gamepad1.left_stick_y;
                break;

            case 2:
                axis = gamepad1.right_stick_x;
                break;
        }
        return axis;
    }
}