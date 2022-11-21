package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class InputManager {
    private Robot bot;

    private final float dpadInputScaler = 0.35f; // controls the speed of dpad movement as a percentage of the max speed
    private final float bezierP2Y = 0.5f; // 0.5 = no effect | 0.0 = max effect

    private Pose2d inputScaler = new Pose2d(0.8, 0.8, 0.6);
    private double YToXMovementRatio = 0.8;

    public InputManager(Robot bot){
        this.bot = bot;
    }

    public Pose2d HandleController1Input(Gamepad gamepad1) {
        Pose2d input = GetAxisInput(gamepad1);
        Pose2d scaledInput = GetScaledInput(input);
        Pose2d correctedInput = ConvertInputToRobotAxis(scaledInput);
        Pose2d localDir = GetLocalDir(correctedInput);
        Pose2d correctedlocalDir = CorrectForAxesSpeedDifferences(localDir);

        return  correctedlocalDir;
    }
    public void HandleController2Input(Gamepad gamepad2) {
        bot.arm.driveArm(-gamepad2.left_stick_y);

        bot.arm.runGripperContinuous(gamepad2.left_bumper, gamepad2.right_bumper);

        /*//run arm to position and release cone
        //reset, low, middle, high
        bot.arm.runLiftToPos(gamepad2.b, gamepad2.a, gamepad2.x, gamepad2.y);
        boolean useDriveArm = gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.y;

        //ARM MOVEMENT - won't work while running arm to position
        if (!useDriveArm) bot.arm.driveArm(-gamepad2.left_stick_y);*/
    }

    private Pose2d GetAxisInput(Gamepad gamepad1) {
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
    private double LinearBezierY(double t){
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
    private Pose2d GetScaledInput(Pose2d input) {
        return new Pose2d(
            input.getX() * inputScaler.getX(),
            input.getY() * inputScaler.getY(),
            input.getHeading() * inputScaler.getHeading()
        );
    }
    private Pose2d ConvertInputToRobotAxis(Pose2d input) {
        return new Pose2d(-input.getY(), input.getX(), input.getHeading());
    }
    private Pose2d GetLocalDir(Pose2d globalDir) {
        bot.smd.update();
        double crntHeadingRad = bot.smd.getPoseEstimate().getHeading();

        Vector2D rotatedVector = new Vector2D(
                globalDir.getX() * Math.cos(crntHeadingRad) - globalDir.getY() * Math.sin(crntHeadingRad),
                globalDir.getX() * Math.sin(crntHeadingRad) + globalDir.getY() * Math.cos(crntHeadingRad)
        );
        return new Pose2d(rotatedVector.getX(), rotatedVector.getY(), globalDir.getHeading());
    }
    private Pose2d CorrectForAxesSpeedDifferences(Pose2d axes) {
        return new Pose2d(axes.getX() * YToXMovementRatio, axes.getY(), axes.getHeading());
    }
}
