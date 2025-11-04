package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.YeeterKing;

@TeleOp(name="Gamepad Driving")
public class GamepadDriveTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);
    //Eater eater = new Eater();

    YeeterKing yeeter = new YeeterKing();

    final double FULL_SPEED = 1.0;
    final double NORMAL_SPEED = 0.5;

    boolean turboEnabled = false;


    double getSpeed() {
        if (turboEnabled == true)
        {
            return FULL_SPEED;
        }
        else{
            return NORMAL_SPEED;
        }

    }

    @Override
    public void init() {
        drive.init(hardwareMap);
       // eater.init(hardwareMap);
        yeeter.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (gamepad2.rightBumperWasPressed()) {
            //boolean eaterStatus = eater.toggle();
            //telemetry.addData("Eater (Intake) Status", eaterStatus);
        }


            yeeter.launch(gamepad2.xWasPressed(), 800);
            yeeter.launch(gamepad2.yWasPressed(), 1100);
            yeeter.launch(gamepad2.bWasPressed(), 1500);

            if(gamepad1.backWasPressed()){
                yeeter.stop();
            }



        if (gamepad1.left_bumper) {
            turboEnabled = true;
        }

        if (gamepad1.right_bumper) {
            turboEnabled = false;
        }

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, getSpeed());



        telemetry.update();




    }
}
