package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchAllYeeterKing;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="One Controller")
public class OneControlTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    LaunchAllYeeterKing yeeter = new LaunchAllYeeterKing();

    final double NORMAL_SPEED = 0.4;

    @Override
    public void init() {
        drive.init(hardwareMap);
        yeeter.init(hardwareMap, telemetry);
        yeeter.close();

    }
    @Override
    public void start()
    {
        yeeter.setVelocity(900);
        yeeter.spinUp();
    }
    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
          yeeter.turnOnEater();
        }
        if (gamepad1.bWasPressed()) {
            yeeter.turnOffEater();
        }

        yeeter.update();

        if (gamepad1.xWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.MEDIUM);
            yeeter.launch();
        }

        if (gamepad1.backWasPressed()) {
            yeeter.stop();
        }
        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;


        drive.drive(forward, right, rotate, NORMAL_SPEED);
        yeeter.printTelemetry();
        telemetry.update();

    }
}
