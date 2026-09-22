package RobotUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

final class Motors extends Robot{
    private Motors(){
        super(null, null);
    }
    @Override
    protected List<DcMotor> setupMotors(OpMode op){
        List<DcMotor> motors = new ArrayList<>();
        for(int i = 4; i < 7; i++){
            CONSTANTS.MOTOR_CONFIGS motorData = CONSTANTS.MOTOR_CONFIG.get(i);
            if(motors.size()<=i-4)motors.add(i-4, op.hardwareMap.dcMotor.get(motorData.NAME));
            else motors.set(i-4, op.hardwareMap.dcMotor.get(motorData.NAME));
            motors.get(i-4).setMode(motorData.RUN_MODE);
            motors.get(i-4).setZeroPowerBehavior(motorData.BRAKE_MODE);
            motors.get(i-4).setDirection(motorData.DIRECTION);
        }
        return motors;
    }

    @Override
    protected List<DcMotor> setupDrivingMotors(OpMode op) {
        List<DcMotor> motors = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            CONSTANTS.MOTOR_CONFIGS motorData = CONSTANTS.MOTOR_CONFIG.get(i);
            if (motors.size() <= i) motors.add(i, op.hardwareMap.dcMotor.get(motorData.NAME));
            else motors.set(i, op.hardwareMap.dcMotor.get(motorData.NAME));
            motors.get(i).setMode(motorData.RUN_MODE);
            motors.get(i).setZeroPowerBehavior(motorData.BRAKE_MODE);
            motors.get(i).setDirection(motorData.DIRECTION);
        }
        return motors;
    }
}
