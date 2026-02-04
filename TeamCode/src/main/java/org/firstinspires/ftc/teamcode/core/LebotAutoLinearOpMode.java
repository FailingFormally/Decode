package org.firstinspires.ftc.teamcode.core;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchAllYeeterKing;

public abstract class LebotAutoLinearOpMode extends AutoLinearOpMode {

    protected LaunchAllYeeterKing yeeter = new LaunchAllYeeterKing();

    public enum DISTANCE {
        SHORT, MEDIUM, LONG
    }

    /**
     * Translate our local DISTANCE enum to the YeeterKing values.
     * @param distance - ENUM for Long, Medium or Short
     */
    private double getSpeed(DISTANCE distance) {
        double speed = LaunchAllYeeterKing.SHORT; // the default
        switch(distance) {
            case LONG:
                speed = LaunchAllYeeterKing.LONG;
                break;
            case MEDIUM:
                speed = LaunchAllYeeterKing.MEDIUM;
                break;
            case SHORT:
                speed = LaunchAllYeeterKing.SHORT;
                break;
        }
        return speed;
    }

    @Override
    public void initialize() {
        super.initialize();
        yeeter.init(hardwareMap, telemetry);
        yeeter.close();
        yeeter.setVelocity(LaunchAllYeeterKing.LONG);
    }

    public void launch(DISTANCE distance) {
        double speed = getSpeed(distance);
        ElapsedTime timer = new ElapsedTime();
        yeeter.setVelocity(speed);
        yeeter.launchAll();
        timer.reset();

        while  (opModeIsActive() && timer.seconds() < 4) {
            yeeter.update();
        }

       // yeeter.stop();
       // yeeter.turnOffEater();
    }

}
