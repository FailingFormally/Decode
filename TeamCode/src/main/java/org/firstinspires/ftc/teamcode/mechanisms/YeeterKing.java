package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YeeterKing {

    private boolean isYeeting;
    private DcMotor yeetWheelLeft;
    private DcMotor yeetWheelNotLeft;

    public void init (HardwareMap hardwareMap){
        yeetWheelLeft = hardwareMap.dcMotor.get("flywheel_left");

        yeetWheelLeft.setDirection(DcMotor.Direction.FORWARD);

        yeetWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yeetWheelNotLeft = hardwareMap.dcMotor.get("flywheel_right");

        yeetWheelNotLeft.setDirection(DcMotor.Direction.REVERSE);

        yeetWheelNotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public boolean toggle() {
        if (isYeeting) {
            isYeeting = false;
            yeetWheelLeft.setPower(0);
            yeetWheelNotLeft.setPower(0);
        } else {
            isYeeting = true;
            yeetWheelLeft.setPower(1);
            yeetWheelNotLeft.setPower(1);
        }
        return isYeeting;
    }
}

