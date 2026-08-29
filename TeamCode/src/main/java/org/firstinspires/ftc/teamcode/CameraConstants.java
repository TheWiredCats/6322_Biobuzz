package org.firstinspires.ftc.teamcode;

public class CameraConstants {
    //replace with the position of this year's apriltags in inches
    public final double[][] APRIL_TAG_POSITIONS = {{0,0,-1},
            {0,0,0},
            {67,67,0},
            {100,100,0}
    };
    public final double CAMERA_X_OFFSET =5;
    public final double CAMERA_Y_OFFSET =6;
    final double CAMERA_HEIGHT = 10.375;
    //replace with the height of the center of this year's apriltags
    public final double APRIL_TAG_HEIGHT = 18.3125 - CAMERA_HEIGHT;

}
