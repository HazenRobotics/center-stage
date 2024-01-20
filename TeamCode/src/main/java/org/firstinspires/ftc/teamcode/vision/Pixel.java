package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Color;

import org.opencv.core.Rect;

import java.util.ArrayList;

public class Pixel {
    private Rect rect;
    public DeciderPixel.Color color;

    public Pixel(Rect r, DeciderPixel.Color c) {
        rect = r;
        color = c;
    }

    public Pixel(Rect r, ArrayList<Rect> greenRects, ArrayList<Rect> purpleRects, ArrayList<Rect> yellowRects) {
        rect = r;
        color = this.getPixelColor(greenRects, purpleRects, yellowRects);
    }

    public boolean isNear(Rect r2) {
        double centerXDistance = Math.abs((rect.x + rect.width / 2.0) - (r2.x + r2.width / 2.0));
        double centerYDistance = Math.abs((rect.y + rect.height / 2.0) - (r2.y + r2.height / 2.0));
        double widthTolerance = (rect.width + r2.width) / 2.0 + 10;
        double heightTolerance = (rect.height + r2.height) / 2.0 + 10;

        return centerXDistance <= widthTolerance && centerYDistance <= heightTolerance;
    }

    public boolean isTouching(Rect r2) {
        double centerXDistance = Math.abs((rect.x + rect.width / 2.0) - (r2.x + r2.width / 2.0));
        double centerYDistance = Math.abs((rect.y + rect.height / 2.0) - (r2.y + r2.height / 2.0));
        double widthTolerance = (rect.width + r2.width) / 2.0;
        double heightTolerance = (rect.height + r2.height) / 2.0;

        return centerXDistance <= widthTolerance && centerYDistance <= heightTolerance;
    }

    public boolean isTouchingBorder(Rect r2) {
        double leftDistance = Math.abs(rect.x - r2.x);
        double rightDistance = Math.abs((rect.x + rect.width) - (r2.x + r2.width));
        double topDistance = Math.abs(rect.y - r2.y);
        double bottomDistance = Math.abs((rect.y + rect.height) - (r2.y + r2.height));

        return leftDistance <= 10 || rightDistance <= 10 ||
                topDistance <= 10 || bottomDistance <= 10;
    }

    public DeciderPixel.Color getPixelColor(ArrayList<Rect> greenRects, ArrayList<Rect> purpleRects, ArrayList<Rect> yellowRects) {
        for (int i = 0; i < greenRects.size(); i++) {
            if (isTouching(greenRects.get(i))) {
                return DeciderPixel.Color.GREEN;
            }
        }
        for (int i = 0; i < purpleRects.size(); i++) {
            if (isTouching(purpleRects.get(i))) {
                return DeciderPixel.Color.PURPLE;
            }
        }
        for (int i = 0; i < yellowRects.size(); i++) {
            if (isTouching(yellowRects.get(i))) {
                return DeciderPixel.Color.YELLOW;
            }
        }

        return DeciderPixel.Color.WHITE;
    }

    public int getGraphicColor() {
        if (color == DeciderPixel.Color.GREEN) {
            return Color.GREEN;
        }
        if (color == DeciderPixel.Color.PURPLE) {
            return Color.MAGENTA;
        }
        if (color == DeciderPixel.Color.YELLOW) {
            return Color.YELLOW;
        }
        if (color == DeciderPixel.Color.WHITE) {
            return Color.WHITE;
        }
        return Color.BLACK;
    }

    public ArrayList<Pixel> rowNeighbors(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> neighbor = new ArrayList<>();
        for (Pixel currentPixel : pixels) {
            if (((rect.y - (rect.height / 2)) <= (currentPixel.getRect().y + 10)) ||
                    ((rect.y - (rect.height / 2)) >= (currentPixel.getRect().y - 10)) && !currentPixel.getRect().equals(rect)) {
                neighbor.add(currentPixel);
            }
        }
        return neighbor;
    }

    public ArrayList<Pixel> colNeighbors(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> neighbor = new ArrayList<>();
        for (Pixel currentPixel : pixels) {
            if (((rect.x - (rect.width / 2)) <= (currentPixel.getRect().x + 10)) ||
                    ((rect.x - (rect.width / 2)) >= (currentPixel.getRect().x - 10))
                            && !currentPixel.getRect().equals(rect)) {
                neighbor.add(currentPixel);
            }
        }
        return neighbor;
    }

    public Rect getRect() {
        return rect;
    }

    public DeciderPixel.Color getPixelColor() {
        return color;
    }
}
