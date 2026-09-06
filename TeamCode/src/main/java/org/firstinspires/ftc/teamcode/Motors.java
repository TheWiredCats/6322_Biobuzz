package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class Motors {
    private Motors(){
        //same as the others
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

        //The motors in the list should always be placed in order
        //FLMotor, BLMotor, FRMotor, BRMotor
        //So we can set their power directly
        motors.get(0).setPower(FL);
        motors.get(1).setPower(BL);
        motors.get(2).setPower(FR);
        motors.get(3).setPower(BR);
    }
}
