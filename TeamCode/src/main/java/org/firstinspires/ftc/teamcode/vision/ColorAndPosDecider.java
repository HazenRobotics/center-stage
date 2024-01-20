package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.DeciderPixel;

import java.util.ArrayList;

public class ColorAndPosDecider {
    public DeciderPixel decidePixel(ArrayList<DeciderPixel.Color[]> board, int pixelX, int pixelY) {

        DeciderPixel currentPixel = new DeciderPixel(pixelX, pixelY, DeciderPixel.Color.NO_PIXEL);
        currentPixel.pixelX = pixelX;
        currentPixel.pixelY = pixelY;
        currentPixel.color = board.get(pixelY)[pixelX];
        boolean filledUnderneath;

        if (board.get(pixelY).length == 6 && pixelY != 0) {
            filledUnderneath = (board.get(pixelY - 1)[pixelX] != DeciderPixel.Color.NO_PIXEL && board.get(pixelY - 1)[pixelX + 1] != DeciderPixel.Color.NO_PIXEL);
        } else if (board.get(pixelY).length == 7 && pixelY != 0) {
            filledUnderneath = (board.get(pixelY - 1)[pixelX] != DeciderPixel.Color.NO_PIXEL && board.get(pixelY - 1)[pixelX - 1] != DeciderPixel.Color.NO_PIXEL); //if you're encountering weird bugs this is where they are
        } else {
            filledUnderneath = true;
        }

        if (currentPixel.color == DeciderPixel.Color.NO_PIXEL && filledUnderneath) {
            if (!canFormMosaic(board, pixelX, pixelY)) {
                return new DeciderPixel(pixelX, pixelY, DeciderPixel.Color.WHITE);
            } else {
                return new DeciderPixel(
                        pixelX,
                        pixelY,
                        determinePixelColor(generateAdjacentCoordinates(board, pixelX, pixelY)));
            }

        } else if (board.get(pixelY).length != pixelX - 1) {
            return decidePixel(board, pixelX + 1, pixelY);
        } else {
            return decidePixel(board, 0, pixelY + 1);
        }
    }

    private static int determinePixelColor(int[][] pixelCollection) {
        int pixel1 = -1;
        int pixel2 = -1;
        for (int i = 0; i < pixelCollection.length; i++) {
            for (int j = 0; j < 2; j++) {
                if (pixelCollection[i][j] != 0 && (pixel1 == -1)) {
                    pixel1 = pixelCollection[i][j];
                } else if (pixelCollection[i][j] != 0 && (pixel2 == -1)) {
                    pixel2 = pixelCollection[i][j];
                    break;
                }
            }
        }
        if (pixel1 == pixel2) {
            return pixel1;
        } else {
            return (6 - pixel1 - pixel2);
        }
    }

    private static boolean canFormMosaic(ArrayList<DeciderPixel.Color[]> board, int pixelX, int pixelY) {
        return countNonWhitePixels(board, pixelX, pixelY) == 2;
    }

    private static int countNonWhitePixels(ArrayList<DeciderPixel.Color[]> board, int pixelX, int pixelY) {
        int[][] adjacentCoordinates = generateAdjacentCoordinates(board, pixelX, pixelY);
        int nonWhitePixels = 0;
        for (int i = 0; i < adjacentCoordinates.length; i++) {
            if (board.get(pixelY)[pixelX] == DeciderPixel.Color.WHITE) {
                nonWhitePixels++;
            }
        }
        return nonWhitePixels;
    }

    private static int[][] generateAdjacentCoordinates(ArrayList<DeciderPixel.Color[]> board, int pixelX, int pixelY) {
        if (board.get(pixelY).length == 7 && pixelX != 0 && pixelY != 0 && pixelX != 6) {
            return new int[][]{
                    {pixelX + 1, pixelY},
                    {pixelX - 1, pixelY},
                    {pixelX, pixelY - 1},
                    {pixelX - 1, pixelY - 1},
                    {pixelX, pixelY + 1},
                    {pixelX - 1, pixelY + 1}
            };
        } else if (board.get(pixelY).length == 6 && pixelX != 0 && pixelX != 5 && pixelY != 0) {
            return new int[][]{
                    {pixelX + 1, pixelY},
                    {pixelX - 1, pixelY},
                    {pixelX, pixelY - 1},
                    {pixelX + 1, pixelY - 1},
                    {pixelX, pixelY + 1},
                    {pixelX + 1, pixelY + 1}
            };
        } else if (pixelY == 0 && pixelX != 0 && pixelX != 5) {
            return new int[][]{
                    {pixelX + 1, pixelY},
                    {pixelX - 1, pixelY},
                    {pixelX, pixelY + 1},
                    {pixelX + 1, pixelY + 1}
            };
        } else if (pixelY == 0 && pixelX == 0) {
            return new int[][]{
                    {0, 1},
                    {1, 1},
                    {1, 0}
            };
        } else if (pixelY == 0 && pixelX == 5) {
            return new int[][]{
                    {5, 1},
                    {6, 1},
                    {4, 0}
            };
        } else if (board.get(pixelY).length == 7 && pixelX == 0 && pixelY != 0) {
            return new int[][]{
                    {pixelX + 1, pixelY},
                    {pixelX, pixelY - 1},
                    {pixelX, pixelY + 1},
            };
        } else if (board.get(pixelY).length == 6 && pixelX == 0 && pixelY != 0) {
            return new int[][]{
                    {pixelX + 1, pixelY},
                    {pixelX, pixelY - 1},
                    {pixelX + 1, pixelY - 1},
                    {pixelX, pixelY + 1},
                    {pixelX + 1, pixelY + 1}
            };
        } else if (board.get(pixelY).length == 7 && pixelX == 6 && pixelY != 0) {
            return new int[][]{
                    {pixelX - 1, pixelY},
                    {pixelX - 1, pixelY - 1},
                    {pixelX - 1, pixelY + 1},
            };
        }
        return new int[][]{
                {pixelX + 1, pixelY},
                {pixelX - 1, pixelY},
                {pixelX, pixelY - 1},
                {pixelX - 1, pixelY - 1},
                {pixelX, pixelY + 1},
                {pixelX - 1, pixelY + 1}
        };
    }

    private static boolean isAdjacent(ArrayList<DeciderPixel.Color[]> board, int pixelX, int pixelY, int pixelX2,
                                      int pixelY2) {
        if (board.get(pixelY).length == 7 && (pixelY2 == pixelY - 1)) {
            return (pixelX2 == pixelX || pixelX2 == pixelX - 1);
        } else if (board.get(pixelY).length == 6 && (pixelY2 == pixelY - 1)) {
            return (pixelX2 == pixelX || pixelX2 == pixelX + 1);
        }
        return (pixelY == pixelY2 && (pixelX2 == pixelX + 1 || pixelX2 == pixelX - 1));
    }
}
