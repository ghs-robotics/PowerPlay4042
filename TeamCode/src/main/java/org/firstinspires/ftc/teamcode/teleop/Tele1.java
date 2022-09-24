package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class Tele1 extends LinearOpMode {
    //Input Variables
    private final float dpadInputScaler = 1; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.1f; // 0.5 = no effect | 0.0 = max effect


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
                if ( axis == 0 ) {
                    axis = gamepad1.left_stick_x;
                    if ( axis < 0 ) axis = -LinearBezierY( -axis );
                    else axis = LinearBezierY( axis );
                }
                else axis *= dpadInputScaler;
                break;

            case 1:
                if ( gamepad1.dpad_up ) axis++;
                if ( gamepad1.dpad_down ) axis--;
                if ( axis == 0 ) {
                    axis = gamepad1.left_stick_y;
                    if ( axis < 0 ) axis = -LinearBezierY( -axis );
                    else axis = LinearBezierY( axis );
                }
                else axis *= dpadInputScaler;
                break;

            case 2:
                axis = gamepad1.right_stick_x;
                if ( axis < 0 ) axis = -LinearBezierY( -axis );
                else axis = LinearBezierY( axis );
                break;
        }
        return axis;
    }
    private float LinearBezierY( float t ){
        //Uses the Y coordinates of 3 points to solve for the Y coordinate along the linear bezier curve at percentage "t"
        float y1 = 0;
        float y2 = bezierP2Y;
        float y3 = 1;

        float oneMinusT = 1 - t;
        return ( oneMinusT * oneMinusT * y1 ) + ( 2 * oneMinusT * t * y2 ) + ( t * t * y3 );
    }
}