package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Alliance; // RED, BLUE alliances

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchAllYeeterKing;

@TeleOp(name="Driving with launchAll")
public class LaunchAllTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    LaunchAllYeeterKing yeeter = new LaunchAllYeeterKing();

    final double FULL_SPEED = 0.7;
    final double NORMAL_SPEED = 0.5;

    boolean turboEnabled = false;

    Alliance alliance = Alliance.RED;

    Limelight3A limelight;

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

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Default to 0 for red alliance
    }
    @Override
    public void start()
    {
        yeeter.setVelocity(900);
        yeeter.spinUp();
    }
    @Override
    public void loop() {
        telemetry.addData("alliance", alliance);

        if (gamepad2.right_trigger > 0) {
          yeeter.toggleEater();
        }

        yeeter.update();

        if (gamepad2.xWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.SHORT);
            yeeter.launch();
        }
        if (gamepad2.yWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.MEDIUM);
            yeeter.launch();
        }
        if (gamepad2.bWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.LONG);
            yeeter.launch();
        }
        if (gamepad2.aWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.REVERSE);
            yeeter.setVelocity(200);
        }

        if (gamepad2.leftBumperWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.SHORT);
            yeeter.launchAll();
        }
        if (gamepad2.rightBumperWasPressed()) {
            yeeter.setDirection(DcMotor.Direction.FORWARD);
            yeeter.setVelocity(LaunchAllYeeterKing.LONG);
            yeeter.launchAll();
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

        if(gamepad1.right_trigger > 0){
            if(alliance == Alliance.RED){
                alliance = Alliance.BLUE;
                limelight.pipelineSwitch(1);
            }
            else {
                alliance = Alliance.RED;
                limelight.pipelineSwitch(0);
            }
        }

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, getSpeed());
        yeeter.printTelemetry();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();

    }
}
