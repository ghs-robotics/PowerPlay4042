package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {

    public enum MainStates {
        ReadSleeve,
        ConeActions,
        Parking
    }
    public enum ConeActions {
        PlaceConePos1
    }
    private MainStates _mainState = MainStates.ReadSleeve;
    private ConeActions _coneAction = ConeActions.PlaceConePos1;
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
            case ReadSleeve:

                break;

            case ConeActions:
                ConeActions();
                break;

            case Parking:

                break;
        }
    }
    private void ConeActions() {
        switch ( _coneAction ) {
            case PlaceConePos1:
                switch ( _currentStep ) {
                    case 0:

                        break;
                }
                break;
        }
    }
}
