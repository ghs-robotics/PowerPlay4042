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
            float hInput = GetAxis( 0 );
            float vInput = GetAxis( 1 );
            float rInput = GetAxis( 2 );

            robot.setWeightedDrivePower(new Pose2d(-hInput, vInput, rInput));

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

            //Reset Pose2D
            if ( gamepad2.a ) {
                Vector2D startPos = robot.TileCords( new Vector2D( 0, 2 ), new Vector2D( 0.5, 1 ) );
                robot.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );
            }

            //MoveTo calls
            if ( gamepad2.x ) {
                targetPos = robot.TileCords( new Vector2D( 4, 5 ), new Vector2D( 0.5, 0.5 ) );
            }
            else if ( gamepad2.y ) {
                targetPos = robot.TileCords( new Vector2D( 1, 1 ), new Vector2D( 0.5, 0.5 ) );
            }
            else if ( gamepad2.b ) {
                targetPos = robot.TileCords( new Vector2D( 3, 2 ), new Vector2D( 0.5, 0.5 ) );
            }

            if ( gamepad2.right_bumper ) {
                robot.MoveToPosLoop( targetPos, robot, telemetry );
            }

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

            telemetry.addData("horizontalInput", hInput);
            telemetry.addData("verticalInput", vInput);
            telemetry.addData("rotationInput", rInput);

            //telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            //telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            //telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

            robot.update();
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
                    axis = LinearBezierY( gamepad1.left_stick_x );
                }
                else axis *= dpadInputScaler;
                break;

            case 1:
                if ( gamepad1.dpad_up ) axis++;
                if ( gamepad1.dpad_down ) axis--;
                if ( axis == 0 ) {
                    axis = LinearBezierY( gamepad1.left_stick_y );
                }
                else axis *= dpadInputScaler;
                break;

            case 2:
                axis = gamepad1.right_stick_x;
                axis = LinearBezierY( axis );
                break;
        }
        return axis;
    }
    private float LinearBezierY( float t ){
        //Uses the Y coordinates of 3 points to solve for the Y coordinate along the linear bezier curve at percentage "t"
        float negativeValue = 1;
        if ( t < 0 ) {
            t *= -1;
            negativeValue = -1;
        }
        if ( t > 1) t = 1;

        float y1 = 0;
        float y2 = bezierP2Y;
        float y3 = 1;

        float oneMinusT = 1 - t;
        return negativeValue * ( ( oneMinusT * oneMinusT * y1 ) + ( 2 * oneMinusT * t * y2 ) + ( t * t * y3 ) );
    }
}