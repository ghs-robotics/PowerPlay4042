package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {

    private int _mainState = 0; // 0 = ReadSleeve | 1 = ConeActions | 2 = Parking
    private int _coneAction = 0; // 0 = place cone at pos #1
    private int _currentStep = 0; // current step in coneAction

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Robot drive = new Robot(hardwareMap, telemetry);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                MainState();
            }
        }
    }
    private void MainState() {
        switch( _mainState ) {
            case 0:

                break;

            case 1:
                ConeActions();
                break;

            case 2:

                break;
        }
    }
    private void ConeActions() {
        switch ( _coneAction ) {
            case 0:
                switch ( _coneAction ) {
                    case 0:

                        break;
                }
                break;
        }
    }
}