package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrains.CoaxialSwerveDrive;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.Random;

public class AprilTagTuner extends OpMode {
    final static double[] STARTING_CONFIG = new double[]{627.866405467, 627.866405467, 367.632040506, 237.041562849};
    final static int[] X_RANGE = {12, 60};
    final static int[] Y_RANGE = {0, -72};
    final static int NUMBER_OF_FAILS_PER_TEST = 10;
    final static int STEP_AMOUNT = 1;


    double[] currentConfig = STARTING_CONFIG;
    ArrayList<double[]> bestConfigs = new ArrayList<>();
    boolean inTuneMode = false;
    boolean inNegativeMode = false;
    int testNumber = 1;
    int configIndex = 0;
    int numberOfFails = 0;
    CoaxialSwerveDrive swerve = new CoaxialSwerveDrive(hardwareMap);
    GamepadEvents gamePad = new GamepadEvents(gamepad1);
    AprilTagUtil util;
    int[] currentPose = {48, -48};

    Random r = new Random();

    @Override
    public void init() {
        util = new AprilTagUtil(hardwareMap);
        bestConfigs.add(STARTING_CONFIG);
        bestConfigs.add(STARTING_CONFIG);

    }

    @Override
    public void loop() {
        if (inTuneMode) {
            if (numberOfFails > NUMBER_OF_FAILS_PER_TEST) {
                if (inNegativeMode) {
                    configIndex++;
                    if (configIndex > 3) {
                        configIndex = 0;
                        testNumber++;
                    }

                }
                inNegativeMode = !inNegativeMode;
                numberOfFails = 0;
                currentConfig=bestConfigs.get(0);
            }
            if (inNegativeMode) currentConfig[configIndex] -= STEP_AMOUNT / testNumber;
            else currentConfig[configIndex] += STEP_AMOUNT / testNumber;
            util.setLensIntrinsics(true, currentConfig);
            double currentConfigResult = calculateDistance(util.getPositionBasedOnTag(), currentPose);
            util.setLensIntrinsics(true, bestConfigs.get(0));
            double bestConfigResult = calculateDistance(util.getPositionBasedOnTag(), currentPose);
            telemetry.addData("Current config result", currentConfigResult);
            telemetry.addData("Best config result", bestConfigResult);
            if (currentConfigResult < bestConfigResult) {
                bestConfigs.set(0, currentConfig);
            } else {
                numberOfFails++;
            }


        } else {
            swerve.drive(-gamePad.left_stick_y, gamePad.left_stick_x, gamePad.right_stick_x);
        }
        if (gamePad.a.onPress()) {
            inTuneMode = !inTuneMode;
        }
        if (gamePad.b.onPress()) {
            inTuneMode = false;
            currentPose = new int[]{r.nextInt(X_RANGE[1] - X_RANGE[0]) + X_RANGE[0], r.nextInt(Y_RANGE[1] - Y_RANGE[0]) + Y_RANGE[0]};
            bestConfigs.add(0, STARTING_CONFIG);
        }
        telemetry.addData("Current test position", currentPose[0] + "," + currentPose[1]);
        telemetry.addData("Test number", testNumber);
        telemetry.addData("Current Test index", configIndex);
        double[] suggestedConfig = averageArrayList(bestConfigs);
        telemetry.addData("Suggested config",suggestedConfig[0]+","+suggestedConfig[1]+","+suggestedConfig[2]+","+suggestedConfig[3] );
        telemetry.addLine("Press A to toggle driving\nPress B to start new test\nPlace the bot in place listed above as current test position.\nWhen the bot is in the correct place press A to start the test");

        telemetry.update();

    }

    public static double calculateDistance(Point3 pointA, int[] pointB) {
        return Math.sqrt(Math.pow((pointA.x - pointB[0]), 2) + Math.pow((pointA.y - pointB[1]), 2));
    }

    public static double[] averageArrayList(ArrayList<double[]> list) {
        // Assume all double arrays have the same length
        int length = list.get(0).length;
        double[] avgArray = new double[length];

        for (double[] arr : list) {
            for (int i = 0; i < length; i++) {
                avgArray[i] += arr[i];
            }
        }

        for (int i = 0; i < length; i++) {
            avgArray[i] /= list.size();
        }

        return avgArray;
    }
}



