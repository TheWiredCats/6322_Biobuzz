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

import java.util.List;

public final class Driving_Systems {
    private Driving_Systems() {
        //so u don't accidentally make an instance of it and only call it as needed
    }

    private final static PID_Controller pidD = new PID_Controller(PIDModes.DRIVING);
    private final static PID_Controller pidT = new PID_Controller(PIDModes.TURNING);

    private static void driveTo(DistanceUnit sigma, GoBildaPinpointDriver pinpoint, Limelight3A limelight,
                                List<DcMotor> motors, LinearOpMode ll,
                                double x, double y) {

        pidD.reset(x, y);


        //This tells us the angel we want to travel in
        double startingHeading = Math.atan2(-(y - pinpoint.getPosY(sigma)), (x - pinpoint.getPosX(sigma)));

        //We cant really read 2d direction so we split it into its X and Y direction
        double directionX = Math.sin(startingHeading);
        double directionY = Math.cos(startingHeading);

        pinpoint.update();

        pidD.setTime(System.nanoTime(), pidD.getError(pinpoint.getPosX(sigma), pinpoint.getPosY(sigma)));
        //PID LOOP HELL
        //run until either op mode turns off or until we're both moving less than .5 inches per second
        //and also .5 inches away from the position
        while (ll.opModeIsActive() && ((Math.sqrt(Math.pow(pinpoint.getVelX(sigma), 2) +
                Math.pow(pinpoint.getVelY(sigma), 2)) > CONSTANTS.tolerance.DSpeed) || (pidD.getError(
                pinpoint.getPosX(sigma),
                pinpoint.getPosY(sigma)
        )) < CONSTANTS.tolerance.DError)) {

            //Confirming current position using limelight
            try {
                Cameras.confirmPosition(limelight, pinpoint, ll);
            } catch (MonkeyBusiness ignored) {
            }

            //update pinpoint for some fresh data
            pinpoint.update();
            double error = pidD.getError(pinpoint.getPosX(sigma), pinpoint.getPosY(sigma));
            double P = pidD.getP(error);
            double I = pidD.getI((x - pinpoint.getPosX(sigma)) * directionX +
                    (y - pinpoint.getPosY(sigma)) * directionY);
            double D = pidD.getD(error);

            //For PID, we need magnitude and Direction, the heading I'm gonna use for direction,
            //and for magnitude I'm just gonna use distance formula

            //The angel of where I am compared to where I want to be going
            double desiredHeading = Math.atan2((y - pinpoint.getPosY(sigma)), -(x - pinpoint.getPosX(sigma)));
            //where I should tell the robot we're pointing so we can go where we want to go
            double roboYaw = (pinpoint.getHeading(AngleUnit.RADIANS) - desiredHeading);

            //use PID as magnitude
            double output = P + I + D;

            //telemetry data being added
            ll.telemetry.addData("PID DATA", "KP: %.2f, KI: %.2f, KD: %.2f, " +
                            "error: %.2f", CONSTANTS.driverConstants.KP,
                    CONSTANTS.driverConstants.KI,
                    CONSTANTS.driverConstants.KD,
                    error);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f", P, I, D, output);
            Pinpoint.addTelemetry(pinpoint, ll);
            ll.telemetry.update();


            //cos represents x but bc Shawn doesn't know how to place a pinpoint it now represents Y
            //by that logic sin now represents X
            double xPower = 1.1 * output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = -output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);

            //We set the power to each motor using this math, and the motors list
            Motors.setPowers((yPower + xPower), (yPower - xPower), (yPower - xPower), (yPower + xPower), motors);
        }
        //brake after we get to the x y positions
        Motors.setPowers(0, 0, 0, 0, motors);
    }

    private static void turnTo(AngleUnit sigma, LinearOpMode ll, GoBildaPinpointDriver pinpoint,
                               List<DcMotor> motors, double desiredHeading) {

        pidT.reset(desiredHeading);

        //update pinpoint before we start loop for fresh data
        pinpoint.update();

        pidT.setTime(System.nanoTime(), pidT.getError(pinpoint.getHeading(CONSTANTS.unit.AU)));

        while (ll.opModeIsActive() &&
                ((Math.abs((pinpoint.getHeadingVelocity(CONSTANTS.unit.AU.getUnnormalized()))) >
                        CONSTANTS.tolerance.TSpeed) ||
                        (Math.abs(pidT.getError(pinpoint.getHeading(sigma))) >
                                CONSTANTS.unit.AU.fromUnit(sigma, CONSTANTS.tolerance.TError)))) {

            //update the pinpoint for fresh data
            pinpoint.update();

            //error is the current difference between the 2 angels
            double error = Cameras.wrapAngle(sigma, desiredHeading - pinpoint.getHeading(sigma));

            //P part of PID represents how much change we still need to do
            //but is often the cause of oscillation when KP is too high
            double P = pidT.getP(error);

            //D part of the PID represents how much error is changing, we're taking the derivative
            //of the different positions by simply using the limit definition
            double D = pidT.getD(error);

            //I part of the PID represents how much the error has changed, we take the integral
            //by simply multiplying by dt and adding over every loop
            double I = pidT.getI(error);

            //the output of the PID is represented by the addition of each part of the pid
            //multiplied by their respective multiplier
            double total = (P + I + D);

            //telemetry data for tuning and testing
            ll.telemetry.addData("PID DATA", "KP: %.2f, KI: %.2f, KD: %.2f, error: " +
                            "%.2f",
                    CONSTANTS.turningConstants.KP,
                    CONSTANTS.turningConstants.KI,
                    CONSTANTS.turningConstants.KD, error);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f", P, I, D, total);
            Pinpoint.addTelemetry(pinpoint, ll);
            ll.telemetry.update();

            //set the motors to either positive or negative motors
            Motors.setPowers(-total, -total, total, total, motors);
        }
        //brake after we arrive at our destination
        Motors.setPowers(0, 0, 0, 0, motors);
    }
    public static void headTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                              LinearOpMode ll, DistanceUnit sigmaDis, AngleUnit sigmaAng,
                              Pose2D targetPosition){
        driveTo(sigmaDis, pinpoint, limelight, motors, ll, targetPosition.getX(sigmaDis),
                targetPosition.getY(sigmaDis));

        turnTo(sigmaAng, ll, pinpoint, motors, targetPosition.getHeading(sigmaAng));
    }

    /*
    public static void goTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                     LinearOpMode ll, DistanceUnit sigma, int id,
                     double distance)throws NullPointerException{
        //turn the distance from whatever it is to the standardized one
        double convertedDistance = (CONSTANTS.unit.DU).fromUnit(sigma, distance);

        //how far away we should be given a certain angle pretty sure this is right could be wrong tho
        //idrk lol
        double x = CONSTANTS.APRIL_TAG_POSITIONS[id][0] - convertedDistance * Math.sin(
                AngleUnit.RADIANS.fromUnit(CONSTANTS.unit.AU, CONSTANTS.APRIL_TAG_POSITIONS[id][2]));
        //same thing as x just a lil different still should be right do
        double y = CONSTANTS.APRIL_TAG_POSITIONS[id][1] - convertedDistance * Math.cos(
                AngleUnit.RADIANS.fromUnit(CONSTANTS.unit.AU, CONSTANTS.APRIL_TAG_POSITIONS[id][2]));

        //go towards that position, and then look towards the id
        headTo(pinpoint, limelight, motors, ll , CONSTANTS.unit.DU,
                CONSTANTS.unit.AU, x, y, (CONSTANTS.unit.AU==AngleUnit.RADIANS?
                        Math.PI:180) + CONSTANTS.APRIL_TAG_POSITIONS[id][2]);

        //make sure we are looking at it and then move until we are the exact distance away
        //that we want to be
        lockIn(CONSTANTS.unit.DU, ll, limelight, pinpoint, motors, convertedDistance);
    }
    */
    public static void headTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                              LinearOpMode ll, DistanceUnit sigmaDis, AngleUnit sigmaAng,
                              double x, double y, double heading) {
        //then drive to the location we want to go to
        driveTo(sigmaDis, pinpoint, limelight, motors, ll, x, y);

        //first turn to the direction we want to be heading
        turnTo(sigmaAng, ll, pinpoint, motors, heading);
    }

    public static boolean lockOn(LinearOpMode ll, Limelight3A limelight, GoBildaPinpointDriver pinpoint,
                                 List<DcMotor> motors, double initialHeading, double shimmy) {

        //Make sure im not trying to shimmy too much or trying to move while I shouldn't be
        if (ll.opModeIsActive() && Math.abs(shimmy) < 50) {

            //if this isn't our first time looping then move a lil to the right or left
            if (shimmy != 0) turnTo(CONSTANTS.unit.AU, ll, pinpoint, motors,
                    initialHeading + CONSTANTS.unit.AU.fromUnit(AngleUnit.DEGREES, shimmy));

            //increment shimmy by 5 or make it negative
            shimmy = (shimmy < 0 ? 5 - shimmy : (shimmy > 0 ? -shimmy : 5));

            //give it a chance to scan after we shimmy
            ll.sleep(50);

            //get latest results from the ll
            LLResult results = limelight.getLatestResult();

            try {

                //get the closest result as the result we want
                LLResultTypes.FiducialResult result = Cameras.getBiggest(results.getFiducialResults());

                limelight.pipelineSwitch(TagData.tagData.get(result.getFiducialId()-30).value);

                //if its valid head towards it
                turnTo(CONSTANTS.unit.AU, ll, pinpoint, motors,
                        pinpoint.getHeading(CONSTANTS.unit.AU) -
                                CONSTANTS.unit.AU.fromUnit(AngleUnit.DEGREES, result.getTargetXDegrees()));
                return true;
            } catch (MonkeyBusiness e) {
                //recursive hehe
                //but in serious, we're just gonna repeat the code but move a lil to the right or left
                return lockOn(ll, limelight, pinpoint, motors, initialHeading, shimmy);

            }
        } else if (ll.opModeIsActive()) {
            //otherwise turn to where we were at the beginning
            turnTo(CONSTANTS.unit.AU, ll, pinpoint, motors, initialHeading);
            limelight.pipelineSwitch(0);
            return false;
        }
        limelight.pipelineSwitch(0);
        return false;
    }

    private static void lostCause(LinearOpMode ll, Limelight3A limelight,
                                  GoBildaPinpointDriver pinpoint, List<DcMotor> motors, Team team){
        boolean left3rd = pinpoint.getPosY(CONSTANTS.unit.DU) > 48;
        boolean right3rd = pinpoint.getPosY(CONSTANTS.unit.DU) < -48;
        boolean top3rd = pinpoint.getPosX(CONSTANTS.unit.DU) > 48;
        boolean bottom3rd = pinpoint.getPosX(CONSTANTS.unit.DU) < -48;
        if(left3rd){
            if(top3rd){

            }else if(bottom3rd){

            }else{

            }
        }else if(right3rd){
            if(top3rd){

            }else if(bottom3rd){

            }else{

            }
        }else{
            if(top3rd){

            }else if(bottom3rd){

            }else{

            }
        }
    }
    public static void lockIn(DistanceUnit sigma, LinearOpMode ll, Limelight3A limelight,
                              GoBildaPinpointDriver pinpoint, List<DcMotor> motors,
                              double distance) {
        //make sure we're facing the right way
        if (lockOn(ll, limelight, pinpoint, motors, pinpoint.getHeading(CONSTANTS.unit.AU), 0)) {
            double convertedDistance = CONSTANTS.unit.DU.fromUnit(sigma, distance);
            LLResult result = limelight.getLatestResult();
            if(result==null)return;
            LLResultTypes.FiducialResult tags = Cameras.getBiggest(result.getFiducialResults());
            Pose2D targetPos = TagData.getPosition(tags.getFiducialId());

            double deltaX = targetPos.getX(CONSTANTS.unit.DU) - pinpoint.getPosX(CONSTANTS.unit.DU);
            double deltaY = targetPos.getY(CONSTANTS.unit.DU) - pinpoint.getPosY(CONSTANTS.unit.DU);
            double currentMagnitude = Math.hypot(deltaX, deltaY);

            if(currentMagnitude <= 0.00001)return;
            double targetX = pinpoint.getPosX(CONSTANTS.unit.DU) + ((deltaX / currentMagnitude) * (currentMagnitude - convertedDistance));
            double targetY = pinpoint.getPosY(CONSTANTS.unit.DU) + ((deltaY / currentMagnitude) * (currentMagnitude - convertedDistance));
            Pose2D target = new Pose2D(CONSTANTS.unit.DU,
                    targetX,
                    targetY,
                    CONSTANTS.unit.AU,
                    pinpoint.getHeading(CONSTANTS.unit.AU));
            Driving_Systems.headTo(pinpoint, limelight, motors, ll, CONSTANTS.unit.DU,
                    CONSTANTS.unit.AU, target);
        }
    }
}
