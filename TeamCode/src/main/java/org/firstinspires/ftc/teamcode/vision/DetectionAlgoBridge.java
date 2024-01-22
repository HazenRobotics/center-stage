package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.processors.BackdropProcessor;

import java.sql.Array;
import java.util.ArrayList;

public class DetectionAlgoBridge {
    BackdropProcessor b = new BackdropProcessor();
    ColorAndPosDecider decide = new ColorAndPosDecider();

    public DeciderPixel[] outputColors() {
        DeciderPixel[] output = new DeciderPixel[2];
        output[0] = decide.decidePixel(b.getColorGrid(), 0, b.getColorGrid().size() - 1);
        return null;
    }

}