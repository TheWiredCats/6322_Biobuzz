package RobotUtil;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class RobotUtil {
    private static Robot robot;
    private static void checker(){if(robot==null)robot=Robot.getInstance();}

    /**
     * <h1>Motor Power Setter</h1>
     * If the max power is greater than 1, then it divides all the other powers by it, essentially capping it between {@code [1,-1]},
     * then sets each of the individual motors to those powers.
     * @param FL Power For Front Left Motor
     * @param BL Power For Back Left Motor
     * @param FR Power For Front Right Motor
     * @param BR Power For Back Right Motor
     * @param motors List of each of the motors
     */
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

    /**
     * <h1>Camera Result Calculator</h1>
     * Runs through every single result, and returns the larget one
     * @param results Latest camera result
     * @return The largest April Tag
     * @throws MonkeyBusiness If no April Tags are detected throw this error
     */
    public static LLResultTypes.FiducialResult getBiggest(List<LLResultTypes.FiducialResult> results)throws MonkeyBusiness {
        if(results.isEmpty())throw new MonkeyBusiness();
        if(results.size()==1)return results.get(0);
        //make a new fiducial result that has nothing in it
        LLResultTypes.FiducialResult result = results.get(0);

        //run through all the April tags and checks for which one is the bigger
        //cuz that means it the closest to the robot and least likely to have errors
        for(LLResultTypes.FiducialResult fr:results){

            //makes result equal to this new result if the new result is greater than the old
            //result, or if there is no old result
            if(result.getTargetArea()<fr.getTargetArea())result = fr;

        }
        //return the result
        return result;
    }

    /**
     * <h1>Angle Calculator</h1>
     * Find the coterminal angle that's closer to 0 then the original angle, and then finds both of the differences, and then finally returns the lesser of the 2 differences
     * @param angle1 The First Angle
     * @param angle2 The Second Angle
     * @return The difference between the 2 angles wrapped into a [-179,180]
     */
    public static double wrapAngle(double angle1, double angle2){
        double adjustedAngle1 = angle1 - Math.copySign(360, angle1);
        return Math.abs(adjustedAngle1 - angle2) < Math.abs(angle1 - angle2)?
                adjustedAngle1 - angle2 : angle1 - angle2;
    }
    public static void confirmPosition()throws MonkeyBusiness {
        checker();
        LinearOpMode ll = robot.getLL();
        Limelight3A limelight = robot.getLimelight();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        LLResult result = limelight.getLatestResult();
        double oldTimestamp = result.getTimestamp();
        LLResultTypes.FiducialResult tags = getBiggest(result.getFiducialResults());
        if(tags.getTargetPoseCameraSpace().getOrientation().getPitch()<-70)throw new MonkeyBusiness();
        limelight.pipelineSwitch(TagData.tagData.get(tags.getFiducialId()-30).value);
        long startTime = System.currentTimeMillis();
        LLResult freshResult = limelight.getLatestResult();
        while (ll.opModeIsActive() && (freshResult.getTimestamp() <= oldTimestamp)) {
            if (System.currentTimeMillis() - startTime > 40)break;
            ll.sleep(2);
            freshResult = limelight.getLatestResult();
        }
        Pose3D position = freshResult.getBotpose_MT2();
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, -position.getPosition().x,
                -position.getPosition().y, AngleUnit.DEGREES,
                wrapAngle(180, -position.getOrientation().getYaw(AngleUnit.DEGREES))));
        limelight.pipelineSwitch(0);
    }
    public static void placementScanner(){
        checker();
        Limelight3A limelight = robot.getLimelight();
        RevBlinkinLedDriver LED = robot.getLed();
        try{
            LLResultTypes.FiducialResult x = RobotUtil.getBiggest(limelight.getLatestResult().getFiducialResults());
            if(Math.abs(x.getTargetXDegrees()) > 7.5) LED.setPattern(CONSTANTS.ledConfig.TARGET_SIGHTED);
            else if(Math.abs(x.getTargetXDegrees()) <= 7.5 && Math.abs(x.getTargetXDegrees()) > 2.5)
                LED.setPattern(CONSTANTS.ledConfig.CLOSE);
            else LED.setPattern(CONSTANTS.ledConfig.ON_POINT);
        } catch (MonkeyBusiness ignored) {
            LED.setPattern(CONSTANTS.ledConfig.NONE_SIGHTED);
        }
    }

    /**
     * An upgrade of the actual pinpoint's .getPosition
     * @see GoBildaPinpointDriver#getPosition() Actual getPosition()
     * @see Robot#setUpPinpoint(OpMode)  Pinpoint Setup
     * @return The Current Position in Units Described By {@linkplain CONSTANTS}
     */
    public static Pose2D getPosition(){
        checker();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        return (new Pose2D(CONSTANTS.unit.DU, pinpoint.getPosX(CONSTANTS.unit.DU),
                pinpoint.getPosY(CONSTANTS.unit.DU), CONSTANTS.unit.AU,
                pinpoint.getHeading(CONSTANTS.unit.AU)));
    }

    /**
     * Adds pinpoint telemetry: Distance Units, Angle Units, X Position, Y Position, Heading
     */
    public static void addTelemetry(){
        checker();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        OpMode op = robot.getOP();
        //adds the distance unit, angle unit, x y positions, and the heading to telemetry
        op.telemetry.addData("Odometry Data", "Distance Unit: %s, Angle Unit: %s, " +
                        "X Pos: %.2f, Y Pos: %.2f, Heading: %.2f",
                CONSTANTS.unit.DU,
                CONSTANTS.unit.AU,
                pinpoint.getPosX(CONSTANTS.unit.DU),
                pinpoint.getPosY(CONSTANTS.unit.DU),
                pinpoint.getHeading(CONSTANTS.unit.AU)
        );
    }
}
