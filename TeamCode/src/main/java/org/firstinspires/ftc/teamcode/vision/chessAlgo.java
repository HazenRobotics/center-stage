package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;

public class chessAlgo {

    public int calculateMoveScore(ArrayList<Pixel[]> board, int[] pos) {
        return 0;
    }

    public ArrayList<int[]> calculateLegalMoves(ArrayList<Pixel[]> board) {
        ArrayList<int[]> out = new ArrayList<>();
        for (int i = 0; i < board.size(); i++) {
            for (int j = 0; j < board.get(i).length; j++) {
                boolean filledUnderneath = false;
                if (board.get(i).length == 6 && i != 0) {
                    filledUnderneath = (board.get(i - 1)[j].color != DeciderPixel.Color.NO_PIXEL && board.get(i - 1)[j + 1].color != DeciderPixel.Color.NO_PIXEL);
                } else if (board.get(i).length == 7 && i != 0) {
                    filledUnderneath = (board.get(i - 1)[j].color != DeciderPixel.Color.NO_PIXEL && board.get(i - 1)[j - 1].color != DeciderPixel.Color.NO_PIXEL);
                } else {
                    filledUnderneath = true;
                }
                if (filledUnderneath && board.get(i)[j].color == DeciderPixel.Color.NO_PIXEL) {
                    out.add(
                            new int[]{
                                    j,
                                    i
                            }
                    );
                }
            }
        }
        return out;
    }

    private boolean canMosaic(ArrayList<Pixel[]> board, int[] pos) {
        int[][] adj = generateAdjacentCoordinates(board, pos);
        if (countColorPixels(board, pos) == 2) {
            for (int[] ints : adj) {
                if (board.get(ints[1])[ints[0]].color != DeciderPixel.Color.WHITE && board.get(ints[1])[ints[0]].color != DeciderPixel.Color.NO_PIXEL) {
                    return (countColorPixels(board, new int[]{ints[0], ints[1]}) == 1);
                }
            }
        }
        return false;
    }

    private int countWhitePixels(ArrayList<Pixel[]> board, int[] pos) {
        int[][] adjacentCoordinates = generateAdjacentCoordinates(board, pos);
        int whitePixels = 0;
        for (int i = 0; i < adjacentCoordinates.length; i++) {
            if (board.get(pos[1])[pos[0]].color == DeciderPixel.Color.WHITE) {
                whitePixels++;
            }
        }
        return whitePixels;
    }

    private int countColorPixels(ArrayList<Pixel[]> board, int[] pos) {
        int[][] adjacentCoordinates = generateAdjacentCoordinates(board, pos);
        int whitePixels = 0;
        for (int i = 0; i < adjacentCoordinates.length; i++) {
            if (board.get(pos[0])[pos[1]].color != DeciderPixel.Color.WHITE && board.get(pos[0])[pos[1]].color != DeciderPixel.Color.NO_PIXEL) {
                whitePixels++;
            }
        }
        return whitePixels;
    }

    private int[][] generateAdjacentCoordinates(ArrayList<Pixel[]> board, int[] pos) {
        if (board.get(pos[1]).length == 7 && pos[0] != 0 && pos[1] != 0 && pos[0] != 6) {
            return new int[][]{
                    {pos[0] + 1, pos[1]},
                    {pos[0] - 1, pos[1]},
                    {pos[0], pos[1] - 1},
                    {pos[0] - 1, pos[1] - 1},
                    {pos[0], pos[1] + 1},
                    {pos[0] - 1, pos[1] + 1}
            };
        } else if (board.get(pos[1]).length == 6 && pos[0] != 0 && pos[0] != 5 && pos[1] != 0) {
            return new int[][]{
                    {pos[0] + 1, pos[1]},
                    {pos[0] - 1, pos[1]},
                    {pos[0], pos[1] - 1},
                    {pos[0] + 1, pos[1] - 1},
                    {pos[0], pos[1] + 1},
                    {pos[0] + 1, pos[1] + 1}
            };
        } else if (pos[1] == 0 && pos[0] != 0 && pos[0] != 5) {
            return new int[][]{
                    {pos[0] + 1, pos[1]},
                    {pos[0] - 1, pos[1]},
                    {pos[0], pos[1] + 1},
                    {pos[0] + 1, pos[1] + 1}
            };
        } else if (pos[1] == 0 && pos[0] == 0) {
            return new int[][]{
                    {0, 1},
                    {1, 1},
                    {1, 0}
            };
        } else if (pos[1] == 0 && pos[0] == 5) {
            return new int[][]{
                    {5, 1},
                    {6, 1},
                    {4, 0}
            };
        } else if (board.get(pos[1]).length == 7 && pos[0] == 0 && pos[1] != 0) {
            return new int[][]{
                    {pos[0] + 1, pos[1]},
                    {pos[0], pos[1] - 1},
                    {pos[0], pos[1] + 1},
            };
        } else if (board.get(pos[1]).length == 6 && pos[0] == 0 && pos[1] != 0) {
            return new int[][]{
                    {pos[0] + 1, pos[1]},
                    {pos[0], pos[1] - 1},
                    {pos[0] + 1, pos[1] - 1},
                    {pos[0], pos[1] + 1},
                    {pos[0] + 1, pos[1] + 1}
            };
        } else if (board.get(pos[1]).length == 7 && pos[0] == 6 && pos[1] != 0) {
            return new int[][]{
                    {pos[0] - 1, pos[1]},
                    {pos[0] - 1, pos[1] - 1},
                    {pos[0] - 1, pos[1] + 1},
            };
        }
        return new int[][]{
                {pos[0] + 1, pos[1]},
                {pos[0] - 1, pos[1]},
                {pos[0], pos[1] - 1},
                {pos[0] - 1, pos[1] - 1},
                {pos[0], pos[1] + 1},
                {pos[0] - 1, pos[1] + 1}
        };
    }
}
