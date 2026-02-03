package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The "Eater" is the robot intake.
 * aka Devourer of Artifacts, Robo-Kirb, The Rizzler
 */
public class Eater {
    boolean isEating = false;

    private DcMotor eaterMotor;

    public boolean status() { return isEating; }

    public void init(HardwareMap hardwareMap) {
        eaterMotor = hardwareMap.dcMotor.get("intake");
        eaterMotor.setDirection(DcMotor.Direction.FORWARD);
        eaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean toggle() {
        if (isEating) {
            off();
        } else {
            on();
        }

        return isEating;
    }

    public void on() {
        this.on(0.7);
    }

    public void on(double speed) {
        isEating = true;
        eaterMotor.setPower(speed);
    }

    public void off() {
        isEating = false;
        eaterMotor.setPower(0);
    }

    public  void setPower(double power) {
        isEating = true;
        eaterMotor. setPower(power);
    }
}
