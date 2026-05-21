/*
Copyright 2026 FIRST Tech Challenge Team 10022

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class OffSeasonPrototype1I extends OpMode {
    /* Declare OpMode members. */

    private DcMotor Intake = null;

    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;

    private Limelight3A limelight = null;

    private IMU imu = null;

    @Override
    public void init() {
        Intake = hardwareMap.dcMotor.get("intake");

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        FLMotor = hardwareMap.dcMotor.get("FL");
        BLMotor = hardwareMap.dcMotor.get("BL");
        FRMotor = hardwareMap.dcMotor.get("FR");
        BRMotor = hardwareMap.dcMotor.get("BR");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);

        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            )
        );
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        imu.resetYaw();
        limelight.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Intake.setPower(gamepad1.a?-1:0);


        double speedmultiplier = MathUtils.clamp(((1-gamepad1.left_trigger)/2)+((1-gamepad1.right_trigger)/2),0.25,1);

        boolean following = false;
        double roboYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x; //controls turning

        LLResult result = limelight.getLatestResult(); //april tag code
        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addLine("Limelight Found!");
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                if (fr.getFiducialId() == 9 && gamepad1.a)  {
                   // rx = fr.getTargetPoseRobotSpace().getOrientation().getYaw() / 35; it wasn't one line of code

                    // If Limelight is mounted forward, tx IS your error.
                    // You might need to flip the sign depending on your motor configuration.
                    double headingError = result.getTx();

                    // Normalize error (just in case)
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;

                    // Simple Proportional control (P-loop). Adjust Kp until it snaps to target smoothly.
                    double Kp = 0.04;
                    rx = headingError * Kp;

                    // Optional: Cap rx so it doesn't spin violently
                    rx = Math.max(-0.5, Math.min(0.5, rx));
                }
            }
        } else {
            telemetry.addLine("No Limelight Detected :C");
        }

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedmultiplier;
        double FRMotorPower = ((y - x - rx) / stickTotal) * speedmultiplier;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedmultiplier;
        double BRMotorPower = ((y + x - rx) / stickTotal) * speedmultiplier;

        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower);
        BLMotor.setPower(BLMotorPower);
        BRMotor.setPower(BRMotorPower);

        if (gamepad1.right_stick_button)imu.resetYaw();

        telemetry.addData("SpeedMult", speedmultiplier);
        telemetry.addData("Yaw", roboYaw);
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();
    }
}
