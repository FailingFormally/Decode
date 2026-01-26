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
     * @param distance
     */
    private getSpeed(DISTANCE distance) {
        switch(distance) {
            case LONG:
                return LaunchAllYeeterKing.LONG;
                break;
            case MEDIUM:
                return LaunchAllYeeterKing.MEDIUM;
                break;
            case SHORT:
                return LaunchAllYeeterKing.SHORT;
                break;
        }
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

        while  (opModeIsActive() && timer.seconds() < 3) {
            yeeter.update();
        }

        yeeter.stop();
        yeeter.turnOffEater();
    }

}
