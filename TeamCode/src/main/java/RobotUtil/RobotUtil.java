package RobotUtil;

import android.util.ArrayMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Map;

//V3.0
//Removed LED's
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
     */
    public void setPowers(double FL, double BL, double FR, double BR){
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
            robot.drivingMotors.get(i).setPower(powers[i]);
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
     *
     * Find the coterminal angle that's closer to 0 then the original angle, and then finds both of the differences, and then finally returns the lesser of the 2 differences
     * @param a1 The First Angle
     * @param a2 The Second Angle
     * @throws MonkeyBusiness On the off chance theirs an error in the math somehow will throw a super error
     * @return The difference between the 2 angles wrapped into a [-179,180]
     */
    public double wrapAngle(double a1, double a2)throws MonkeyBusiness{
        double normalizedA1 = a1 % 360;
        double normalizedA2 = a2 % 360;
        double result = compareAngles(normalizedA1-360, normalizedA1, normalizedA1+360, normalizedA2);
        return -((int) (100 * result)) / 100.0;
    }
    private double compareAngles(double left, double first, double right, double ref)throws MonkeyBusiness{
        double leftDistance = Math.abs(left - ref);
        double firstDistance = Math.abs(first - ref);
        double rightDistance = Math.abs(right - ref);
        double min = Math.min(Math.min(leftDistance, firstDistance), rightDistance);
        if(min == leftDistance) return left-ref;
        if(min == firstDistance) return first-ref;
        if(min == rightDistance) return right-ref;
        throw new MonkeyBusiness("How Did This Even Happen");
    }

    /**
     * <h1>Limelight/Pinpoint Position Confirmer</h1>
     * <h6><i>Why Be Sure When You Can Be HIV Positive</i></h6>
     * Confirms position by using different pipelines to filter for the 4 nearest tags and then do a lil bit of math to find the position of the robot and then setting the pinpoint to the new position
     * @throws MonkeyBusiness Will throw a base error if no tag is detected, or if the pitch of the tag is less than -70
     */
    public void confirmPosition()throws MonkeyBusiness {
        /*
        LinearOpMode ll = robot.getLL();
        Limelight3A limelight = robot.getLimelight();
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        LLResult result = limelight.getLatestResult();
        double oldTimestamp = result.getTimestamp();
        LLResultTypes.FiducialResult tags = getBiggest(result.getFiducialResults());
        if(tags.getTargetPoseCameraSpace().getOrientation().getPitch()<-25)
            throw new MonkeyBusiness();
        limelight.pipelineSwitch(TagData.tagData.get(tags.getFiducialId() - 30).value);
        long startTime = System.currentTimeMillis();
        LLResult freshResult = limelight.getLatestResult();
        while(ll.opModeIsActive() && (freshResult.getTimestamp()<=oldTimestamp)) {
            if(System.currentTimeMillis() - startTime>40) break;
            ll.sleep(2);
            freshResult = limelight.getLatestResult();
        }
        Pose3D position = freshResult.getBotpose_MT2();
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, -position.getPosition().x,
                                        -position.getPosition().y, AngleUnit.DEGREES,
                                        wrapAngle(180, -position.getOrientation().getYaw(AngleUnit.DEGREES))));
        limelight.pipelineSwitch(0);
         */
    }

    /**
     * <h1>Pinpoint Position Getter</h1>
     * <h6><i>3.4 What, Inches? Feet? Autistic Camels?!</i></h6>
     * An upgrade of the actual pinpoint's .getPosition(), which gets a position with more specific units rather
     * than just millimeters
     * @see GoBildaPinpointDriver#getPosition() Actual getPosition()
     * @return The Current Position in Units Described By {@linkplain Constants Constants File}
     */
    public Pose2D getPosition()throws MonkeyBusiness{
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        return (new Pose2D(Constants.unit.DU, pinpoint.getPosX(Constants.unit.DU),
                           pinpoint.getPosY(Constants.unit.DU), Constants.unit.AU,
                           pinpoint.getHeading(Constants.unit.AU)));
    }

    /**
     * <h1>Auto Pinpoint Telemetry Adder</h1>
     * <h6><i>I'm Tired of Adding Telemetry Grandpa</i></h6>
     * Adds pinpoint telemetry: Distance Units, Angle Units, X Position, Y Position, Heading
     * @see Constants Unit Data
     */
    public void addTelemetry(){
        GoBildaPinpointDriver pinpoint = robot.getPinpoint();
        OpMode op = robot.getOP();
        //adds the distance unit, angle unit, x y positions, and the heading to telemetry
        op.telemetry.addData("Odometry Data", "Distance Unit: %s, Angle Unit: %s, " +
                        "X Pos: %.2f, Y Pos: %.2f, Heading: %.2f",
                             Constants.unit.DU,
                             Constants.unit.AU,
                             pinpoint.getPosX(Constants.unit.DU),
                             pinpoint.getPosY(Constants.unit.DU),
                             pinpoint.getHeading(Constants.unit.AU)
        );
    }

    public void brakeMotors(){
        Map<DcMotor, DcMotor.ZeroPowerBehavior> before= new ArrayMap<>(robot.drivingMotors.size());
        for(DcMotor motor : robot.drivingMotors){
            before.put(motor, motor.getZeroPowerBehavior());
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
        try{Thread.sleep(10);}catch(Exception ignored){}
        for(DcMotor motor : before.keySet()){
            motor.setZeroPowerBehavior(before.get(motor));
        }
    }
}
