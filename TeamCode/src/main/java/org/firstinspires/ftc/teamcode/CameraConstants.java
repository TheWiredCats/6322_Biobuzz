package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public final class CameraConstants {
    //used in case we switch to Radians
    //private final double pi = Math.PI;
    public final List<Object> MEASUREMENTS = List.of(DistanceUnit.INCH, AngleUnit.DEGREES);
    //replace with the position of this year's apriltags in inches
    //except for the first one keep this one empty   V
    public final double[][] APRIL_TAG_POSITIONS = {{0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {12, 0, 180},
            {12 + ((5.5 * Math.sqrt(3)) / 2), 2.75, 45},
            {12 + ((5.5 * Math.sqrt(3)) / 2), -2.75, 135}
    };
    public final double CAMERA_X_OFFSET = 5;
    public final double CAMERA_Y_OFFSET = 6;
    final double CAMERA_HEIGHT = 10.375;
    //replace with the height of the center of this year's apriltags
    public final double APRIL_TAG_HEIGHT = 18.3125 - CAMERA_HEIGHT;

}
