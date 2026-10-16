package org.firstinspires.ftc.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "test auto v1.1")
public class TempAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        DcMotor transfer = hardwareMap.dcMotor.get("transfer");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor shoot = hardwareMap.dcMotor.get("shoot");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setPower(1);
        intake.setPower(1);
        shoot.setPower(1);
        while(!isStopRequested()){
            //wait until its time
            sleep(10);
        }
        transfer.setPower(0);
        intake.setPower(0);
        shoot.setPower(0);
    }
}
