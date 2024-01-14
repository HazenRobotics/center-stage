package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.processors.BackdropProcessor;

import java.sql.Array;
import java.util.ArrayList;

public class DetectionAlgoBridge {
    BackdropProcessor b = new BackdropProcessor();
    ColorAndPosDecider decide = new ColorAndPosDecider();

    public DeciderPixel.Color[] outputColors() {

    }

