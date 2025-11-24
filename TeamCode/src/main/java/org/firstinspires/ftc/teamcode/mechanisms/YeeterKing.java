package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FlyWheelTesterOpMode;

public class YeeterKing {

    private boolean isYeeting;
    private DcMotorEx yeetWheelLeft;
    private DcMotorEx yeetWheelNotLeft;

    private Servo servo1, servo2;

    private double velocity = 0;

    final double tolerance = 0.1;

    private Telemetry telemetry;

    private ElapsedTime timer;


    private enum LaunchState {
        /**
         * The default state
         */
        IDLE,

        SPIN_UP,

        READY,

        LAUNCH,
        /**
         * In this state, we are actively waiting for the launch to occur.
         * From here, we probably need to return to SPIN_UP to get back to launch
         * velocity.
         */
        LAUNCHING
    }

    private boolean hasLaunched = false;

    private LaunchState launchState = LaunchState.IDLE;

    public void setDirection(DcMotor.Direction direction) {
        if (direction == DcMotor.Direction.FORWARD) {
            yeetWheelLeft.setDirection(DcMotor.Direction.REVERSE);
            yeetWheelNotLeft.setDirection(DcMotor.Direction.FORWARD);
        } else {
            yeetWheelLeft.setDirection(DcMotor.Direction.FORWARD);
            yeetWheelNotLeft.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void open()
    {
        servo1.setPosition(0.6694);
        servo2.setPosition(0.2994);
    }
    public void close()
    {
        servo1.setPosition(0.8794);
        servo2.setPosition(0.2694);
    }
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

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setDirection(Servo.Direction.REVERSE);

        servo2 = hardwareMap.get(Servo.class, "servo2");

        close();
    }

    public void stop(){
        yeetWheelLeft.setPower(0);
        yeetWheelNotLeft.setPower(0);
        launchState=LaunchState.IDLE;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void launch(boolean shotRequested) {

        // If we have already launched, and a new shot is requested,
        // reset hasLaunched to track the next shot
        if (hasLaunched == true && shotRequested == true) {
            hasLaunched = false;
        }

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
                yeetWheelLeft.setVelocity(velocity);
                yeetWheelNotLeft.setVelocity(velocity);

                // Advance to LAUNCH when both motors are up to speed
                if (yeetWheelLeft.getVelocity() >= (velocity - tolerance) &&
                        yeetWheelNotLeft.getVelocity() >= (velocity - tolerance)) {
                    launchState = LaunchState.READY;
                }
                break;

            case READY:
                if(hasLaunched == false){
                    launchState= launchState.LAUNCH;

                }
                break;

            case LAUNCH:
                open();
                timer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                hasLaunched = true;
                if (timer.seconds() > .8) {
                    close();
                    launchState = LaunchState.SPIN_UP;
                }
                break;


        }
    }
}