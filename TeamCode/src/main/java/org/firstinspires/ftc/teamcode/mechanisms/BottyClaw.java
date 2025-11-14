package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BottyClaw {

    private double ARM_SPEED = 0.3;
    private DcMotor arm;
    private Servo claw;

    public void init(HardwareMap hwMap) {
        arm = hwMap.get(DcMotor.class, "Drop");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hwMap.get(Servo.class, "Claw");
    }

    public void up() {
        arm.setPower(ARM_SPEED);
    }

    public void down() {
        arm.setPower(-ARM_SPEED);
    }

    public void stop() {
        arm.setPower(0);
    }

    public void openClaw()
    {
        claw.setPosition(0.14);
    }
    public void closeClaw()
    {
        claw.setPosition(0);
    }





}
