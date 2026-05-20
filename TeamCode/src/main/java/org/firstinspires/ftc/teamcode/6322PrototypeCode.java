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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
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

    private IMU imu = null;

    @Override
    public void init() {
        Intake = hardwareMap.dcMotor.get("intake");

        imu = hardwareMap.get(IMU.class, "imu");

        FLMotor = hardwareMap.dcMotor.get("FL");
        BLMotor = hardwareMap.dcMotor.get("BL");
        FRMotor = hardwareMap.dcMotor.get("FR");
        BRMotor = hardwareMap.dcMotor.get("BR");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        boolean slowspeed = false;
        double speedmult = 1;

        if (gamepad1.a){
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }


        if (gamepad1.rightStickButtonWasReleased() && slowspeed == false) {
            slowspeed = true;
        } else if(gamepad1.rightStickButtonWasReleased() && slowspeed == true) {
            slowspeed = false;
        }

        if (slowspeed){
             speedmult = 0.5;
         } else {
             speedmult = 1 - gamepad1.left_trigger;
             if (1 - gamepad1.left_trigger <= 0.5)
                 speedmult = 0.5;
         }

        double roboYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x; //controls turning

        double x = lx * Math.cos(roboYaw) - ly * Math.sin(-roboYaw);
        double y = lx * Math.sin(-roboYaw) + ly * Math.cos(roboYaw);

        double stickTotal = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double FLMotorPower = ((y + x + rx) / stickTotal) * speedmult;
        double FRMotorPower = ((y - x - rx) / stickTotal) * speedmult;
        double BLMotorPower = ((y - x + rx) / stickTotal) * speedmult;
        double BRMotorPower = ((y + x - rx) / stickTotal) * speedmult;

        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower);
        BLMotor.setPower(BLMotorPower);
        BRMotor.setPower(BRMotorPower);

        if (gamepad1.right_stick_button)
            imu.resetYaw();

        telemetry.addData("Slowmode", slowspeed);
        telemetry.addData("SpeedMult", speedmult);
        telemetry.addData("Yaw", roboYaw);
    }
}
