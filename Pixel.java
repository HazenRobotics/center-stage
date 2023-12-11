import java.util.ArrayList;

public class Pixel {
    public Pixel(int pixelX, int pixelY, Color color) {
        this.pixelX = pixelX;
        this.pixelY = pixelY;
        this.color = color;
    }

    public Pixel(int pixelX, int pixelY, int color) {
        this.pixelX = pixelX;
        this.pixelY = pixelY;
        switch (color) {
            case 0:
                this.color = Color.WHITE;
            case 1:
                this.color = Color.YELLOW;
            case 2:
                this.color = Color.GREEN;
            case 3:
                this.color = Color.PURPLE;
            case 4:
                this.color = Color.NO_PIXEL;
        }
    }

    public int pixelX;
    public int pixelY;
    public Color color;

    public enum Color {
        WHITE(0),
        YELLOW(1),
        GREEN(2),
        PURPLE(3),
        NO_PIXEL(4);

        Color(int i) {
        }
    }
}
