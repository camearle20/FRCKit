package frckit.vision.corner;

public class CornerPair {
    private final Corner left;
    private final Corner right;

    public CornerPair(Corner left, Corner right) {
        this.left = left;
        this.right = right;
    }

    public Corner getLeft() {
        return left;
    }

    public Corner getRight() {
        return right;
    }
}
