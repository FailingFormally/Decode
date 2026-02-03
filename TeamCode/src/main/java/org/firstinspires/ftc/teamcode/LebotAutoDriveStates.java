package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.LebotAutoLinearOpMode;

@Autonomous(name = "Auto for States")
public class LebotAutoDriveStates extends LebotAutoLinearOpMode {

    enum AutoRoutine {
        BlueLong, BlueShort, RedLong, RedShort
    }

    private AutoRoutine autoSelected = AutoRoutine.RedShort;

    /**
     * We provide a menu to select the alliance and position.
     * This will determine which auto routine we use.
     */
    @Override
    public void onInitLoop() {
        telemetry.addData("Selected Auto", autoSelected);
        telemetry.addData("Press D-Pad Up", "BlueLong");
        telemetry.addData("Press D-Pad Down", "BlueShort");
        telemetry.addData("Press D-Pad Left", "RedLong");
        telemetry.addData("Press D-Pad Right", "RedShort");
        telemetry.update();

        // Check for controller input to change selection
        if (gamepad1.dpad_up) {
            autoSelected = AutoRoutine.BlueLong;
        } else if (gamepad1.dpad_down) {
            autoSelected = AutoRoutine.BlueShort;
        } else if (gamepad1.dpad_left) {
            autoSelected = AutoRoutine.RedLong;
        } else if (gamepad1.dpad_right) {
            autoSelected = AutoRoutine.RedShort;
        }
    }

    @Override
    public void runRoutine() {
        // We use a switch statement to run the appropriate routine
        // based on what was selected in `onInitLoop`.
        switch(autoSelected) {
            case RedShort:
                runRedShortAuto();
                break;
            case RedLong:
                runRedLongAuto();
                break;
            case BlueShort:
                runBlueShortAuto();
                break;
            case BlueLong:
                runBlueLongAuto();
                break;
        }
    }

    private void runRedShortAuto() {
        driveStraight(DRIVE_SPEED, 48, 0);
        launch(DISTANCE.SHORT);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, 10, 45);
        turnToHeading(TURN_SPEED, 135);
        driveStraight(PICKUP_SPEED, 40, 135);
        driveStraight(DRIVE_SPEED, -40, 135);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -10, 45);
        // turn toward goal again
        turnToHeading(TURN_SPEED, 0);
        launch(DISTANCE.SHORT);
    }

    private void runRedLongAuto() {
        telemetry.addData("I don't know the Auto routine for:", autoSelected);
        telemetry.update();
        sleep(2000);
    }

    /**
     * Same as `runRedShortAuto` but the angles are reversed.
     */
    private void runBlueShortAuto() {
        driveStraight(DRIVE_SPEED, 48, 0);
        launch(DISTANCE.SHORT);
        turnToHeading(TURN_SPEED, -45);
        driveStraight(DRIVE_SPEED, 10, -45);
        turnToHeading(TURN_SPEED, -135);
        driveStraight(PICKUP_SPEED, 40, -135);
        driveStraight(DRIVE_SPEED, -40, -135);
        turnToHeading(TURN_SPEED, -45);
        driveStraight(DRIVE_SPEED, -10, -45);
        // turn toward goal again
        turnToHeading(TURN_SPEED, 0);
        launch(DISTANCE.SHORT);
    }

    private void runBlueLongAuto() {
        telemetry.addData("I don't know the Auto routine for:", autoSelected);
        telemetry.update();
        sleep(2000);
    }
}
