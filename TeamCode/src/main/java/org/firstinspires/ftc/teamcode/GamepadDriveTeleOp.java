package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.YeeterKing;

@TeleOp(name="Gamepad Driving")
public class GamepadDriveTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    Servo servoLeft;
    Servo servoRight;
    Eater eater = new Eater();

    YeeterKing yeeter = new YeeterKing();

    final double FULL_SPEED = 0.5;
    final double NORMAL_SPEED = 0.3;

    boolean turboEnabled = false;






    double getSpeed() {
        if (turboEnabled)
        {
            return FULL_SPEED;
        }
        else
        {
            return NORMAL_SPEED;
        }
    }

    @Override
    public void init() {
        drive.init(hardwareMap);
        eater.init(hardwareMap);
        yeeter.init(hardwareMap, telemetry);
        yeeter.close();

        servoRight = hardwareMap.get(Servo.class, "servo2");
        servoLeft = hardwareMap.get(Servo.class, "servo1");

        servoLeft.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if (gamepad2.rightBumperWasPressed()) {
            eater.toggle();
        }

        eater.setPower(gamepad2.right_trigger);

        telemetry.addData("Eater (Intake) Status", eater.status());

        if (gamepad2.xWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(800);
            yeeter.launch(true);
        }
        if (gamepad2.yWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(1100);
            yeeter.launch(true);
        }
        if (gamepad2.bWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(1500);
            yeeter.launch(true);
        }
        if (gamepad2.aWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.REVERSE);
            yeeter.setVelocity(200);
            yeeter.launch(true);
        }


        if (gamepad2.backWasPressed()) {
            yeeter.stop();
        }

        if (gamepad1.left_bumper) {
            turboEnabled = true;
        }

        if (gamepad1.right_bumper) {
            turboEnabled = false;
        }

        if (gamepad2.left_bumper){
            servoRight.setPosition(0);
            servoLeft.setPosition(0);
        }

        if (gamepad2.right_bumper){
            servoRight.setPosition(.14);
            servoLeft.setPosition(.14);
        }





        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, getSpeed());

        telemetry.update();

    }
}
