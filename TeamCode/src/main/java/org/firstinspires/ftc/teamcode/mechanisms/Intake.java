package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * aka Devourer of Artifacts, Robo-Kirb, The Rizzler
 */
public class Intake {
    boolean isTaking = false;

    private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean toggle() {
        if (isTaking) {
            isTaking = false;
            intakeMotor.setPower(0);
        } else {
            isTaking = true;
            intakeMotor.setPower(1);
        }

        return isTaking;
    }
}
