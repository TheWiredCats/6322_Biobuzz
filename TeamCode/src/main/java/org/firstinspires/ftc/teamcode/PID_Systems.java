package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.opencv.core.Mat;

import java.util.List;

public class PID_Systems {
    private void confirmPosition(LLResult results, GoBildaPinpointDriver pinpoint, CameraConstants cc){
        //only run the rest of the code if we can actually see a tag
        if (results.isValid()) {

            //run through each result found
            for (LLResultTypes.FiducialResult fr : results.getFiducialResults()) {

                // get rid of - 20 after kick off
                // instead of having to write it the long way we can just make a variable to write
                // the short way, also improves computations time
                int id = fr.getFiducialId() - 20;

                if (id >= 0 && id < cc.APRIL_TAG_POSITIONS.length) {
                    //declare and reset the real X and Y
                    double currentX;
                    double currentY;

                    //Tx and Ty from the limelight are graphed in degrees and are also reversed so
                    //we have to make them negative and turn them into radians before we can use them
                    double tx = Math.toRadians(-fr.getTargetXDegrees());
                    double ty = Math.toRadians(-fr.getTargetYDegrees());

                    //tan of something that is too close to 90 starts to make it head towards
                    //infinity, and  FAST
                    if (Math.abs(tx) < (Math.PI / 3) && Math.abs(ty) < (Math.PI / 3)) {

                        //Get the X and Y positions of each tag
                        double apriltagX = cc.APRIL_TAG_POSITIONS[id][0];
                        double apriltagY = cc.APRIL_TAG_POSITIONS[id][1];

                        //how far away the april tag is
                        double ZDifference = cc.APRIL_TAG_HEIGHT / Math.tan(ty);

                        //how far left or right it is, negative is left and right is positive
                        double LRDifference = ZDifference * Math.tan(tx);

                        //We want to do something a lil different depending on which way the tag is
                        //facing
                        switch ((int) cc.APRIL_TAG_POSITIONS[id][2]) {
                            case (0):
                                currentX = apriltagX - ZDifference;
                                currentY = apriltagY - LRDifference;
                                break;
                            case (1):
                                currentX = apriltagX + ZDifference;
                                currentY = apriltagY + LRDifference;
                                break;
                            case (2):
                                currentX = apriltagX + LRDifference;
                                currentY = apriltagY - ZDifference;
                                break;
                            case (3):
                                currentX = apriltagX - LRDifference;
                                currentY = apriltagY + ZDifference;
                                break;
                            default:
                                //if its something thats not in the code then set the position
                                // to what it already is
                                currentX = pinpoint.getPosX(DistanceUnit.INCH) - cc.CAMERA_X_OFFSET;
                                currentY = pinpoint.getPosY(DistanceUnit.INCH) - cc.CAMERA_Y_OFFSET;
                                break;
                        }
                        //set the position to what the tag says we are, and the position to what
                        //it already is
                        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                                (currentX + cc.CAMERA_X_OFFSET),
                                (currentY + cc.CAMERA_Y_OFFSET),
                                AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES)));

                    }
                }
            }
        }
    }
    private void setPowers(double FL, double BL, double FR, double BR,List<DcMotor> motors){
        //clipping extra could cause some problems and going in a circle
        //so we have to divide 1 by the greatest so we can multiply the rest by that
        //we can only do that if the greatest is over 1 tho

        //check all the powers and divide evreything by the greatest one if its over one
        double greatest = Math.max(Math.max(Math.abs(FL),Math.abs(BL)), Math.max(Math.abs(FR),Math.abs(BR)));
        if(greatest>1){
            //if the greatest is over 1 divide evreything by the greatest to ensure the max is one
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
    public void driveTo(DistanceUnit sigma, GoBildaPinpointDriver pinpoint, Limelight3A limelight,
                        List<DcMotor> motors, LinearOpMode ll, CameraConstants cc,
                        double x, double y){

        //PID BS

        //needs testing/tuning
        //multiplier for each part of the PID
        final double KP = 0.125;
        final double KD = 0.1;
        final double KI = 0;

        //dt is a tiny amount of times, it's a mystery tool that will help us later.
        //(cool calculus kids know what's up)
        double dt;

        //Some more mystery tools that will help us later
        double integral =0;
        double readingIntegral;
        double derivative;
        double proportional;

        //dt measuring stuff
        double currentTime;
        double previousTime=System.currentTimeMillis();

        //update it once to get fresh data
        pinpoint.update();

        //This tells us the angel we want to travel in
        double startingHeading = Math.atan2((y-pinpoint.getPosY(sigma)),-(x-pinpoint.getPosX(sigma)));

        //We cant really read 2d direction so we split it into its X and Y direction
        double directionX = Math.sin(startingHeading);
        double directionY = Math.cos(startingHeading);

        //PID LOOP HELL
        //run untill either opmode turns off or untill were both moving less than .5 inches per second
        //and also .5 inches away from the position
        while(ll.opModeIsActive()&&
                ((Math.sqrt(Math.pow(pinpoint.getVelX(sigma),2)+Math.pow(pinpoint.getVelY(sigma),2))>.5)||
                (Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2))>.5))){

            //Confirming current position using limelight
            confirmPosition(limelight.getLatestResult(), pinpoint, cc);

            //the last pinpoint data before we update to the newest
            double previousError = Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+
                    Math.pow(y-pinpoint.getPosY(sigma),2));


            //update pinpoint for some fresh data
            pinpoint.update();

            //For PID we need magnitude and Direction, the heading im gonna use for direction,
            //and for magnitude im just gonna use distance formula

            //The angel of where i am compared to where i want to be going
            double desiredHeading=Math.atan2((y-pinpoint.getPosY(sigma)),-(x-pinpoint.getPosX(sigma)));
            //where i should tell the robot were pointing so we can go where we want to go
            double roboYaw = (pinpoint.getHeading(AngleUnit.RADIANS)-desiredHeading);

            //were using distance formula to find out the absolute value of how far away we are
            //from the target, or if youve taken algebra 2, were finding the magnitude
            double error = Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2));

            //This part tells us how fast to go depending on how far away we CURRENTLY are
            //This is known as the present or proportional part of the pid or in other words
            //P part of the PID
            proportional=error*KP;

            //dt is the length of the loop

            //current time represents the current time
            currentTime=System.currentTimeMillis();

            //we subtract current time by the last time we ran this and then divide by 1000
            //so we could get dt in seconds rather than in miliseconds
            dt=Math.max((currentTime-previousTime)/1000.0, 0.001);

            //change previous time to the old current time so that when it loops previousTime
            //now represents the previous currentTime
            previousTime=currentTime;

            //without the extra part the integral could not decrease, due to the fact that error
            //represents the magnitude of the error, meaning its only the absolute value
            integral+=(((x-pinpoint.getPosX(sigma))*directionX)+(directionY*(y-pinpoint.getPosY(sigma))))*dt;

            //dont want to change the actual integral, bc that would mess up inner calculations
            // so we make a new variable and multiply that one by the KI also it represents the
            //integral, or inherited errors from PAST loops in other words
            // I part of the PID
            readingIntegral=integral*KI;

            //This represents the Derivative, or destiny of the error, it handles
            //where the error WILL become or in other words
            //D part of the PID
            derivative=KD*((error-previousError)/dt);

            //use PID as magnitude
            double output = -(readingIntegral+derivative+proportional);

            //telemetry data being added
            ll.telemetry.addData("PID DATA","KP: %.2f, KI: %.2f, KD: %.2f, " +
                    "error: %.2f, dt in secs: %.2f", KP, KI,KD, error, dt);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f",proportional, readingIntegral, derivative, output );
            ll.telemetry.addData("Position", new Pose2D(DistanceUnit.INCH,pinpoint.getPosX(DistanceUnit.INCH),pinpoint.getPosY(DistanceUnit.INCH),AngleUnit.DEGREES,0));
            ll.telemetry.update();


            //cos represents x but bc shawn dosent know how to place a pinpoint it now represnts Y
            //by that logic sin now reprsents X
            double xPower = 1.1*output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = -output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);

            //We set the power to each motor using this math, and the motors list
            setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower),motors);
        }
        //brake after we get to the x y positions
        setPowers(0,0,0,0,motors);
    }
    public void turnTo(AngleUnit sigma, LinearOpMode ll,GoBildaPinpointDriver pinpoint,
                       List<DcMotor> motors, double desiredHeading){
        //needs tuning
        //multiplier for each part of the PID
        final double KP = 0.125;
        final double KD = 0.1;
        final double KI = 0;

        //dt is the time between loops, its gonna be a very small amount of time
        double dt;
        //set previous time to rn as a default value
        double previousTime=System.currentTimeMillis();
        double currentTime;

        //each part of the PID
        double proportional;
        double integral=0;
        double derivative;

        //update pinpoint before we start loop for fresh data
        pinpoint.update();
        while(ll.opModeIsActive()&&
                ((pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)>0.5)||
                        (Math.abs(pinpoint.getHeading(sigma)-desiredHeading)>0.5))){

            //grab the previous error to use for D
            double previousError = desiredHeading-pinpoint.getHeading(sigma);

            //update the pinpoint for fresh data
            pinpoint.update();

            //error is the current difference between the 2 angels
            double error = desiredHeading-pinpoint.getHeading(sigma);

            //P part of PID represents how much change we still need to do
            //but is often the cause of oscillation when KP is too high
            proportional=error*KP;

            //obvious name is obvious
            currentTime=System.currentTimeMillis();
            //for all but the first loop this measure almost the time it takes to do the entire loop
            //we divide by 1000 so it gives the data to us in seconds
            dt=(currentTime-previousTime)/1000;
            //set the last current time to previous time to be used in the next loop
            previousTime=currentTime;

            //D part of the PID represents how much error is changing, were taking the derivative
            //of the different positions by simply using the limit definition
            derivative=KD*((error-previousError)/dt);

            //I part of the PID represnets how much the error has changed, we take the integral
            //by using simply multiplying by dt and adding over evrey loop
            integral+=error*dt;

            //the output of the PID is represented by the addition of each part of the pid
            //multiplied by their respective multiplier
            double total=(proportional+derivative+(integral*KI));

            //telemetry data for tuning and testing
            ll.telemetry.addData("PID DATA","KP: %.2f, KI: %.2f, KD: %.2f, error: " +
                    "%.2f, dt in secs: %.2f", KP, KI,KD, error, dt);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f",proportional, integral*KI, derivative, total);
            ll.telemetry.addData("Position", new Pose2D(DistanceUnit.INCH,pinpoint.getPosX(DistanceUnit.INCH),pinpoint.getPosY(DistanceUnit.INCH),AngleUnit.DEGREES,0));
            ll.telemetry.update();

            //set the motors to either positive or negative motors
            setPowers(total,-total,total,-total,motors);
        }
        //brake after we arrive at our destination
        setPowers(0,0,0,0,motors);
    }
    public boolean lockOn(LinearOpMode ll, Limelight3A limelight, GoBildaPinpointDriver pinpoint, List<DcMotor> motors, double initialHeading, double shimmy) {
        //Make sure im not trying to shimmy too much or trying to move while i shouldnt be
        if(ll.opModeIsActive()&&Math.abs(shimmy)<50) {

            //if this isnt our first time looping then move a lil to the right or left
            if(shimmy!=0)turnTo(AngleUnit.DEGREES,ll,pinpoint,motors, initialHeading +shimmy);


            //give it a chance to scan after we shimmy
            ll.sleep(50);

            //get latest results from the ll
            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {
                //if its valid head towards it
                turnTo(AngleUnit.DEGREES, ll, pinpoint, motors, pinpoint.getHeading(AngleUnit.DEGREES) - result.getTx());
                return true;
            } else {

                //recursive hehe
                //but in serious, were just gonna repeat the code but move a lil to the right or left
                return lockOn(ll, limelight, pinpoint, motors, initialHeading, (shimmy<0?5-shimmy:(shimmy>0?-shimmy:5)));

            }
        }else if(ll.opModeIsActive()){
            //otherwise turn to where we were at the beginning
            turnTo(AngleUnit.DEGREES, ll, pinpoint, motors, initialHeading);
            return false;
        }
        return false;
    }
    public void lockIn(DistanceUnit sigma,LinearOpMode ll, Limelight3A limelight, GoBildaPinpointDriver pinpoint, CameraConstants cc,List<DcMotor> motors, double Distance){
        //make sure were facing the right way
        if(lockOn(ll,limelight,pinpoint,motors, pinpoint.getHeading(AngleUnit.DEGREES),0)) {

            //declare it outside so we can increase the scope beyond the fore loop
            double ZDistance;

            //get fresh limelight data
            LLResult results = limelight.getLatestResult();

            //just to have some data
            confirmPosition(results, pinpoint, cc);

            //get how many degrees above us it is
            double ty = results.getTy();

            //we make a right triangle to find how far a way we are and use basic trig
            ZDistance = cc.APRIL_TAG_HEIGHT / Math.tan(Math.toRadians(-ty));

            //we subtract the total from how far we want to be from the tag to find
            //out how far we have to move
            double difference = ZDistance - Distance;

            //we have to get the coordinates -y,x because the whole graph is rotated 90 to the left
            double x = pinpoint.getPosX(sigma) + (difference * Math.sin(pinpoint.getHeading(AngleUnit.RADIANS)));
            double y = pinpoint.getPosY(sigma) + (difference * Math.cos(pinpoint.getHeading(AngleUnit.RADIANS)));
            driveTo(sigma, pinpoint, limelight, motors, ll, cc, x, y);
        }
    }
}
