package frckit.vision.corner;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class DualTopCornerSupplier implements Supplier<List<Corner>> {
    private final Supplier<List<Corner>> cornerSupplier;

    public DualTopCornerSupplier(Supplier<List<Corner>> cornerSupplier) {
        this.cornerSupplier = cornerSupplier;
    }

    @Override
    public List<Corner> get() {
        List<Corner> corners = cornerSupplier.get();

        if (corners.size() < 2) {
            return Collections.emptyList(); //There are not enough corners
        }

        //Sort corners to identify top left and right corners
        corners.sort(Corner.SORT_BY_X);
        corners.sort(Corner.SORT_BY_Y);

        List<Corner> top = corners.subList(0, 2);

        top.sort(Corner.SORT_BY_X);

        return top;
    }
}
