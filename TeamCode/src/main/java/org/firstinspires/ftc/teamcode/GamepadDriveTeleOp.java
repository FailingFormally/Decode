package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.YeeterKing;

@TeleOp(name="Gamepad Driving")
public class GamepadDriveTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

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
        yeeter.init(hardwareMap, telemetry);
        yeeter.close();
    }

    @Override
    public void loop() {
        telemetry.addData("LaunchSate",yeeter.getLaunchState());

        if (gamepad2.rightBumperWasPressed()) {
          yeeter.toggleEater();
        }

        yeeter.update();

        if (gamepad2.xWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(800);
            yeeter.launch();
        }
        if (gamepad2.yWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(1100);
            yeeter.launch();
        }
        if (gamepad2.bWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(1500);
            yeeter.launch();
        }
        if (gamepad2.aWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.REVERSE);
            yeeter.setVelocity(200);
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

        if(gamepad1.left_trigger >0){
            yeeter.open();
        }

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, getSpeed());

        telemetry.update();

    }
}
