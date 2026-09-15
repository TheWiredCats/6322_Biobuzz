package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public final class Motors {
    private Motors(){
        //same as the others
    }
    public static List<DcMotor> setupMotors(OpMode op){
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
    public static List<DcMotor> setupDrivingMotors(OpMode op){
        List<DcMotor> motors = new ArrayList<>();
        for(int i = 0; i < 4; i++){
            CONSTANTS.MOTOR_CONFIGS motorData = CONSTANTS.MOTOR_CONFIG.get(i);
            if(motors.size()<=i)motors.add(i, op.hardwareMap.dcMotor.get(motorData.NAME));
            else motors.set(i, op.hardwareMap.dcMotor.get(motorData.NAME));
            motors.get(i).setMode(motorData.RUN_MODE);
            motors.get(i).setZeroPowerBehavior(motorData.BRAKE_MODE);
            motors.get(i).setDirection(motorData.DIRECTION);
        }
        return motors;
    }

    public static void setPowers(double FL, double BL, double FR, double BR, List<DcMotor> motors){
        //clipping extra could cause some problems and going in a circle
        //so we have to divide 1 by the greatest so we can multiply the rest by that
        //we can only do that if the greatest is over 1 tho

        //check all the powers and divide everything by the greatest one if its over one
        double greatest = Math.max(Math.max(Math.abs(FL),Math.abs(BL)), Math.max(Math.abs(FR),Math.abs(BR)));
        if(greatest>1){
            //if the greatest is over 1 divide everything by the greatest to ensure the max is one
            FL/=greatest;
            BL/=greatest;
            FR/=greatest;
            BR/=greatest;
        }
        double[] powers = new double[]{FL, BL, FR, BR};
        //The motors in the list should always be placed in order
        //FLMotor, BLMotor, FRMotor, BRMotor
        //So we can set their power directly
        for(int i = 0; i < 4; i++){
            motors.get(i).setPower(powers[i]);
        }
    }
}
