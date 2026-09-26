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
    private final Robot robot;
    private static RobotUtil instance;
    private RobotUtil(Robot robot){
        this.robot = robot;
    }

    protected static RobotUtil initialize(Robot robot){
        return instance==null? instance = new RobotUtil(robot): instance;
    }

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
    public void setPowers(double FL, double BL, double FR, double BR, List<DcMotor> motors){
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
     * @throws MonkeyBusiness If no April Tags are detected throw an advanced error
     * @see MonkeyBusiness#MonkeyBusiness(String, OpMode)  Advanced Error
     * @see #getResult(List) Actual Logic Parts
     */
    public LLResultTypes.FiducialResult getBiggest(List<LLResultTypes.FiducialResult> results)throws MonkeyBusiness {
        OpMode op = robot.getOP();
        if(results.isEmpty())throw new MonkeyBusiness("List Empty", op);
        if(results.size()==1)return results.get(0);
        //return the result
        return getResult(results);
    }

    private LLResultTypes.FiducialResult getResult(List<LLResultTypes.FiducialResult> results) {
        LLResultTypes.FiducialResult result = results.get(0);

        //run through all the April tags and checks for which one is the bigger
        //cuz that means it the closest to the robot and least likely to have errors
        for(LLResultTypes.FiducialResult fr: results){

            //makes result equal to this new result if the new result is greater than the old
            //result, or if there is no old result
            if(result.getTargetArea()<fr.getTargetArea())result = fr;

        }
        return result;
    }

    /**
     * <h1>Angle Calculator</h1>
     * Find the coterminal angle that's closer to 0 then the original angle, and then finds both of the differences, and then finally returns the lesser of the 2 differences
     * @param angle1 The First Angle
     * @param angle2 The Second Angle
     * @return The difference between the 2 angles wrapped into a [-179,180]
     */
    public double wrapAngle(double angle1, double angle2){
        double adjustedAngle1 = angle1 - Math.copySign(360, angle1);
        return Math.abs(adjustedAngle1 - angle2) < Math.abs(angle1 - angle2)?
                adjustedAngle1 - angle2 : angle1 - angle2;
    }

    /**
     * <h1>Limelight/Pinpoint Position Confirmer</h1>
     * <h6><i>Why Be Sure When You Can Be HIV Positive</i></h6>
     * Confirms position by using different pipelines to filter for the 4 nearest tags and then do a lil bit of math to find the position of the robot and then setting the pinpoint to the new position
     * @throws MonkeyBusiness Will throw a base error if no tag is detected, or if the pitch of the tag is less than -70
     */
    public void confirmPosition()throws MonkeyBusiness {
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

    /**
     * <h1>Robot Placement Scanner</h1>
     * <h6><i>No Not <u>There</u> I Meant <u>There</u></i></h6>
     * Scans AprilTag's in front of it and changes the color based on the pre-set led-configs<br>
     * <h5><b><u><i>Warning: Must Be Put In A</i></u></b> {@code while(opModeInInit)} <b><u><i>Loop</i></u></b></h5>
     * @see LinearOpMode#opModeInInit() opModeInInit (boolean)
     */
    public void placementScanner(){
        Limelight3A limelight = robot.getLimelight();
        RevBlinkinLedDriver LED = robot.getLed();
        try{
            LLResultTypes.FiducialResult x = getBiggest(limelight.getLatestResult().getFiducialResults());
            if(Math.abs(x.getTargetXDegrees()) > 7.5) LED.setPattern(CONSTANTS.ledConfig.TARGET_SIGHTED);
            else if(Math.abs(x.getTargetXDegrees()) <= 7.5 && Math.abs(x.getTargetXDegrees()) > 2.5)
                LED.setPattern(CONSTANTS.ledConfig.CLOSE);
            else LED.setPattern(CONSTANTS.ledConfig.ON_POINT);
        } catch (MonkeyBusiness ignored) {
            LED.setPattern(CONSTANTS.ledConfig.NONE_SIGHTED);
        }
    }

    /**
     * <h1>Pinpoint Position Getter</h1>
     * <h6><i>3.4 What, Inches? Feet? Autistic Camels?!</i></h6>
     * An upgrade of the actual pinpoint's .getPosition(), which gets a position with more specific units
     * @see GoBildaPinpointDriver#getPosition() Actual getPosition()
     * @see Robot#setUpPinpoint(OpMode)  Pinpoint Setup
     * @return The Current Position in Units Described By {@linkplain CONSTANTS}
     */
    public Pose2D getPosition()throws MonkeyBusiness{
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        return (new Pose2D(CONSTANTS.unit.DU, pinpoint.getPosX(CONSTANTS.unit.DU),
                pinpoint.getPosY(CONSTANTS.unit.DU), CONSTANTS.unit.AU,
                pinpoint.getHeading(CONSTANTS.unit.AU)));
    }

    /**
     * <h1>Auto Pinpoint Telemetry Adder</h1>
     * <h6><i>I'm Tired of Adding Telemetry Grandpa</i></h6>
     * Adds pinpoint telemetry: Distance Units, Angle Units, X Position, Y Position, Heading
     * @see CONSTANTS Unit Data
     */
    public void addTelemetry(){
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
