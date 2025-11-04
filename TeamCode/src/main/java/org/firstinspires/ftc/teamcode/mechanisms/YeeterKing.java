package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FlyWheelTesterOpMode;

public class YeeterKing {

    private boolean isYeeting;
    private DcMotorEx yeetWheelLeft;
    private DcMotorEx yeetWheelNotLeft;

    final double tolerance = 0.1;

    private Telemetry telemetry;


    private enum LaunchState {
        /**
         * The default state
         */
        IDLE,

        SPIN_UP,

        LAUNCH,
        /**
         * In this state, we are actively waiting for the launch to occur.
         * From here, we probably need to return to SPIN_UP to get back to launch
         * velocity.
         */
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.IDLE;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        yeetWheelLeft = hardwareMap.get(DcMotorEx.class, "flywheel_left");

        yeetWheelLeft.setDirection(DcMotor.Direction.REVERSE);

        yeetWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        yeetWheelNotLeft = hardwareMap.get(DcMotorEx.class, "flywheel_right");

        yeetWheelNotLeft.setDirection(DcMotor.Direction.FORWARD);

        yeetWheelNotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;

        PIDFCoefficients coefficients = new PIDFCoefficients(300, 0, 0, 10);
        yeetWheelLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        yeetWheelNotLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

    }

    public void stop(){
        yeetWheelLeft.setPower(0);
        yeetWheelNotLeft.setPower(0);
        launchState=LaunchState.IDLE;
    }

    public void launch(boolean shotRequested, double shootVelocity) {

        switch (launchState) {
            case IDLE:
               // telemetry.addData("LaunchState", launchState);
               // telemetry.addData("LaunchVelocity",shootVelocity);
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;

                }

                break;
            case SPIN_UP:
               // telemetry.addData("LaunchState", launchState);
                //telemetry.addData("LaunchVelocity",shootVelocity);
                yeetWheelLeft.setVelocity(shootVelocity);
                yeetWheelNotLeft.setVelocity(shootVelocity);

                // Advance to LAUNCH when both motors are up to speed
                if (yeetWheelLeft.getVelocity() >= (shootVelocity - tolerance) &&
                        yeetWheelNotLeft.getVelocity() >= (shootVelocity - tolerance)) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
               // telemetry.addData("LaunchState", launchState);
               // telemetry.addData("LaunchVelocity",shootVelocity);
                yeetWheelLeft.setVelocity(shootVelocity);
                yeetWheelNotLeft.setVelocity(shootVelocity);
                // For now, if velocity drops on either flyWheel, we probably fired...
                // go back to SPIN_UP
                if (yeetWheelLeft.getVelocity() < (shootVelocity + tolerance) ||
                        yeetWheelNotLeft.getVelocity() < (shootVelocity + tolerance)) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

        }


    }
}