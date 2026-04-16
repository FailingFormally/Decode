package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Auto Rotate PD tuning")
public class AutoRotateTuningOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);

    Alliance alliance = Alliance.RED;

    Limelight limelight = new Limelight();

    // ==== PD control for aiming ====
    double kP = 0.001;
    double error = 0;
    double lastError = 0;
    double angleTolerance = 0.4;
    double kD = 0.0;
    double curTime = 0;
    double lastTime = 0;

    // ---------- controller based PD tuning ----------
    double [] stepSizes = { 0.1, 0.01, 0.001, 0.0001 };
    int stepIndex = 0;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        drive.init(hardwareMap);
        limelight.init(hardwareMap);
        limelight.setPipeline(0); // Default to 0 for red alliance
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }


    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        telemetry.addData("alliance", alliance);

        // Switch Alliances/Pipelines
        if(gamepad1.right_trigger > 0.1){
            if(alliance == Alliance.RED){
                alliance = Alliance.BLUE;
                limelight.setPipeline(1);
            }
            else {
                alliance = Alliance.RED;
                limelight.setPipeline(0);
            }
        }

        // update P and D on the fly
        // 'B' button cycles through the different step sizes for turning precision
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0
        }

        // Left/Right adjusts P gain
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        // Up/Down adjusts D gain
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            if (gamepad1.left_trigger > 0.2) {

                error = tx;
                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                    lastTime = getRuntime();
                    lastError = 0;
                } else {
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;
                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
            lastError = 0;
            lastTime = getRuntime();
        }

        drive.drive(forward, right, rotate, 0.5);

        telemetry.addLine("---------------------");
        telemetry.addData("Tuning P", "%.4f (dpad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (dpad U/D)", kD);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
