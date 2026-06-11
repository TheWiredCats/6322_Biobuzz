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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import java.util.concurrent.TimeUnit;

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
    //as soon as teleop selected

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;
    private HuskyLens huskyLens = null;

    private Limelight3A limelight = null;

    private IMU imu = null;

    //private Deadline rateLimit = null;

    @Override
    public void init() {
        //runs once as soon as "init" is pressed
        Intake = hardwareMap.dcMotor.get("intake");
        Transfer = hardwareMap.dcMotor.get("transfer");

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        FLMotor = hardwareMap.dcMotor.get("FL");
        BLMotor = hardwareMap.dcMotor.get("BL");
        FRMotor = hardwareMap.dcMotor.get("FR");
        BRMotor = hardwareMap.dcMotor.get("BR");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        //Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        //rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData("HL:", "Problem communicating with " + huskyLens.getDeviceName());
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

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
        //rateLimit.reset();
        Intake.setPower(gamepad1.a?-1:0);
        Transfer.setPower(gamepad1.y?1:0);

        //how low/high the Speed can go with both triggers down/up respectully
        double Minimum = 0.25;
        //maximum must be less than or equal to 1
        double Maximum = 1;

        //Calculates how far the minimum is from the middle of the 2
        // (to know how much each should affect)
        double Difference=(Maximum-Minimum)/2;

        //Readability of code
        double TotalTrigger=gamepad1.right_trigger+gamepad1.left_trigger;

        double speedMultiplier =(Maximum-Difference*TotalTrigger);

        boolean following = true;
        boolean buttonDown = false;
        //drive variables
        double roboYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x; //controls turning

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        if(gamepad1.b)buttonDown=false;
        if(!buttonDown&&!gamepad1.b) {
            buttonDown = true;
            following = !following;
        }
        LLResult result = limelight.getLatestResult(); //april tag code
        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("LL: April tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                if (fr.getFiducialId() == 9 && following)  {
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
                    rx = MathUtils.clamp(rx,-0.5,0.5);
                }
            }
        } else {
            telemetry.addLine("LL: No Detections");
        }

        HuskyLens.Block[] blocks = huskyLens.blocks(); //huskylens code
        telemetry.addData("HL Block Count", blocks.length);

        if (gamepad1.x) for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("HL:", blocks[i].toString());

                // HuskyLens Constants
                final double SCREEN_CENTER_X = 160.0;
                final double HALF_HORIZONTAL_FOV_RAD = Math.toRadians(27.5); // 55 degrees / 2

                // 1. Get your 2D data from HuskyLens
                int blockX = blocks[i].x;

                // 2. Calculate your 3D Z-distance first (from previous step)
                double distanceZ = (2.9 * 4.6) / blocks[i].width;

                // 3. Calculate pixel offset from screen center
                double pixelOffset = blockX - SCREEN_CENTER_X;

                // 4. Normalize the offset (-1.0 to 1.0) and convert to radians
                double angleX = (pixelOffset / SCREEN_CENTER_X) * HALF_HORIZONTAL_FOV_RAD;

                // 5. Calculate final physical X position (in inches or cm depending on your Z unit)
                double positionX = distanceZ * Math.tan(angleX);

                double headingError = positionX;

                // Normalize error (just in case)
                //while (headingError > 180) headingError -= 360;
                //while (headingError < -180) headingError += 360;

                // Simple Proportional control (P-loop). Adjust Kp until it snaps to target smoothly.
                double Kp = 0.04;
                //rx = headingError * Kp;
                rx = headingError * 1.5;

                // Optional: Cap rx so it doesn't spin violently
                rx = MathUtils.clamp(rx,-0.5,0.5);

                telemetry.addData("HL 3D Z (Distance)", distanceZ);
                telemetry.addData("HL 3D X (Lateral)", positionX);

                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 *
                 * These values have Java type int (integer).
                 */
        }

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedMultiplier;
        double FRMotorPower = ((y - x - rx) / stickTotal) * speedMultiplier;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedMultiplier;
        double BRMotorPower = ((y + x - rx) / stickTotal) * speedMultiplier;

        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower);
        BLMotor.setPower(BLMotorPower);
        BRMotor.setPower(BRMotorPower);

        if (gamepad1.right_stick_button)imu.resetYaw();

        telemetry.addData("stickleftX", x);
        telemetry.addData("stickright", rx);
        telemetry.addData("SpeedMult", speedMultiplier);
        telemetry.addData("Yaw", roboYaw);
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL STATS", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("LL Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();
    }
}
