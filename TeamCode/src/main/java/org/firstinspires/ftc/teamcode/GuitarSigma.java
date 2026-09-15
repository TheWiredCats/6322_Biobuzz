package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Guitar Sigma")

public class GuitarSigma extends OpMode {

    private DcMotor Intake;
    private DcMotor Transfer;
    private GoBildaPinpointDriver pinpoint;
    private double frcHeading;
    List<DcMotor> motors;
    private DcMotor Shoot;
    private double speedMult;
    @Override
    public void init() {
        motors = Motors.setupDrivingMotors(this);
        pinpoint = Pinpoint.setUpPinpoint(this);
        frcHeading=pinpoint.getHeading(CONSTANTS.unit.AU);
        List<DcMotor> tempMotors = Motors.setupMotors(this);
        Intake = tempMotors.get(0);
        Transfer = tempMotors.get(1);
        Shoot = tempMotors.get(2);
        speedMult = 0.5+ MathUtils.clamp(gamepad1.right_stick_x, -0.4, 5);
    }

    @Override
    public void loop() {
        double temp = pinpoint.getHeading(CONSTANTS.unit.AU.getUnnormalized());
        pinpoint.update();
        frcHeading +=  gamepad1.back?-frcHeading:Cameras.wrapAngle(CONSTANTS.unit.AU,
                pinpoint.getHeading(CONSTANTS.unit.AU.getUnnormalized()) - temp);

        Intake.setPower(gamepad2.a?1:0);
        Transfer.setPower(gamepad2.y?1:0);
        Shoot.setPower(gamepad2.x?1:0);


        speedMult = 0.5+ MathUtils.clamp(gamepad1.right_stick_x, -0.4, 5);
        double roboYaw = AngleUnit.RADIANS.fromUnit(CONSTANTS.unit.AU, frcHeading);
        double lx = - (gamepad1.a?(gamepad1.b?0:1):gamepad1.b?-1:0);
        double ly = 1.1* (gamepad1.x?(gamepad1.y?0:1):gamepad1.y?-1:0);
        double rx = gamepad1.dpad_down?(gamepad1.dpad_up?0:1):gamepad1.dpad_up?-1:0;
        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double FL = (y + x + rx) * speedMult;
        double BL = (y - x + rx) * speedMult;
        double FR = (y - x - rx) * speedMult;
        double BR = (y + x - rx) * speedMult;
        Motors.setPowers(FL, BL, FR, BR, motors);
    }
}
