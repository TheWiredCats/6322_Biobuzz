package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;


public enum FieldPosition {
    BU_RU(AprilTagPosition.UP, AprilTagPosition.UP),
    BU_RD(AprilTagPosition.UP, AprilTagPosition.DOWN),
    BD_RU(AprilTagPosition.DOWN, AprilTagPosition.UP),
    BD_RD(AprilTagPosition.DOWN, AprilTagPosition.DOWN);

    private final AprilTagPosition blue;
    private final AprilTagPosition red;

    FieldPosition(AprilTagPosition blue, AprilTagPosition red) {
        this.blue = blue;
        this.red = red;
    }

    public static FieldPosition from(
            AprilTagPosition blue,
            AprilTagPosition red) {

        for (FieldPosition position : values()) {
            if (position.blue == blue && position.red == red) {
                return position;
            }
        }

        return null;
    }

    public void set(Limelight3A limelight, LLFieldMap fieldMap) {
        limelight.uploadFieldmap(fieldMap, 0);
    }
}