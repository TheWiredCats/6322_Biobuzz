package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;

import java.util.List;

public enum Tag_Data {
    DRED(List.of(
            List.of(),
            List.of()
    )),

    NDRED(List.of(
            List.of(),
            List.of()
    )),

    DBLUE(List.of(
            List.of(),
            List.of()
    )),

    NDBLUE(List.of(
            List.of(),
            List.of()
    ));

    private final List<List<LLFieldMap.Fiducial>> tagsData;

    Tag_Data(List<List<LLFieldMap.Fiducial>> tagsData) {
        this.tagsData = tagsData;
    }

    public List<List<LLFieldMap.Fiducial>> getTagsData() {
        return tagsData;
    }
}