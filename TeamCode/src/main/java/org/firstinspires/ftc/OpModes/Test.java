package org.firstinspires.ftc.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "test")
public class Test extends OpMode {
    DcMotor left;
    DcMotor right;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Left Power", left.getPower());
        telemetry.addData("Right Power", right.getPower());
        telemetry.addData("Stick", -gamepad1.left_stick_y);
        telemetry.update();
    }
}
