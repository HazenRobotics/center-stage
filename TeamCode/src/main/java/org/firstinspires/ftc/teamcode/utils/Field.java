package org.firstinspires.ftc.teamcode.utils;


import org.opencv.core.Point3;

public class Field {
    //in
    //left to right
    final static Point3 APIRL_TAG_BOARD_POSTIONS[] = {
            new Point3(  60.25f, 41.41f,  4f),
            new Point3( 60.25f,  35.41f,  4f),
            new Point3(  60.25f,  29.41f,  4f),
            new Point3(  60.25f,  -29.41f,  4f),
            new Point3(  60.25f,  -35.41f,  4f),
            new Point3(  60.25f,  -41.41f,  4f),
            };
    //Bigs are the 5.5s and 4s are the small
    final static Point3 APIRL_TAG_WALL_POSTIONS[] = {
            new Point3(  -70.25f,  -40.625f,  5.5f),
            new Point3(  -70.25f,  -35.125f,  4f),
            new Point3(  -70.25f,  35.125f,  4f),
            new Point3(  -70.25f,  40.625f,  5.5f),
    };

    final static double TILE_SIZE = 24;
    final static double TILE_CONNECTOR_SIZE = 1;
    final static double TILE_SIZE_EDGE = 24.375;
    final static int LANDING_ZONE_WIDTH = 144;
    final static int LANDING_ZONE_LENGTH = 23;
    final static double[] BACKDROP_HEIGHTS = new double[] {12.375,19,25.75,29.75};
    final static double TRUSS_LENGTH_TEAM = 22.5;
    final static double TRUSS_LENGTH_GROUP = 46.25;
    final static double TRUSS_HEIGHT_GROUP_LOW = 23.5;
    final static double TRUSS_HEIGHT_GROUP_HIGH = 26;
    final static double BACKDROP_PARK_LENGTH = 58.375;
    final static double BACKDROP_PARK_WIDTH = 23.125;
    final static double PIXEL_STACK_LENGTH = 11.5;
    final static double PIXEL_OUTSIDE_LEN = 3;
    final static double PIXEL_INSIDE_LEN = 1.25;
    //count
    final static int PIXEL_ROW = 7;
    final static int PIXEL_COLUMN = 11;
    //April Tags ID
    final static int[] BLUEATID = new int[] {1,2,3};
    final static int[] REDATID = new int[] {4,5,6};
    public enum Pixel {
        WHITE,YELLOW,PURPLE,GREEN,NONE
    }
    //not offset
    static Pixel[][] pixels = new Pixel[PIXEL_COLUMN][PIXEL_ROW];
    public static Point3 getTagPosition(int i) {
        if(i>APIRL_TAG_BOARD_POSTIONS.length-1) {
            return APIRL_TAG_WALL_POSTIONS[i];
        } else {
            return APIRL_TAG_BOARD_POSTIONS[i];
        }
    }


}
