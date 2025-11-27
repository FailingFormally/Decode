package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.BottyClaw;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Botty White")
public class BottyTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    AprilTagWebcam webcam = new AprilTagWebcam();
    BottyClaw claw = new BottyClaw();
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
        claw.init(hardwareMap);
        webcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        webcam.update();

        if (!gamepad1.dpad_up && !gamepad1.dpad_down) { claw.stop(); }

        if (gamepad1.dpad_down)
        {
            claw.down();
        }

        if (gamepad1.dpad_up)
        {
            claw.up();
        }

        if (gamepad1.left_bumper) {
            turboEnabled = true;
        }

        if (gamepad1.right_bumper) {
            turboEnabled = false;
        }
        if (gamepad1.bWasPressed()) {
            claw.openClaw();
        }

        if (gamepad1.aWasPressed()) {
            claw.closeClaw();
        }

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, getSpeed());

        if (gamepad1.xWasPressed()) {
            webcam.printAllDetections();
        }

        telemetry.update();

    }

    @Override
    public void stop() {
        claw.closeClaw(); // Make sure the claw is in the closed position
        webcam.stop();
    }
}
