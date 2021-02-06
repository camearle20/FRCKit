package frckit.vision.corner;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DualTopCornerSupplier implements Supplier<Optional<CornerPair>> {
    private final Supplier<List<Corner>> cornerSupplier;

    public DualTopCornerSupplier(Supplier<List<Corner>> cornerSupplier) {
        this.cornerSupplier = cornerSupplier;
    }

    @Override
    public Optional<CornerPair> get() {
        List<Corner> corners = cornerSupplier.get();

        if (corners.size() < 2) {
            return Optional.empty(); //There are not enough corners
        }

        //Sort corners to identify top left and right corners
        corners.sort(Corner.SORT_BY_X);
        corners.sort(Corner.SORT_BY_Y);

        List<Corner> top = corners.subList(0, 2);

        top.sort(Corner.SORT_BY_X);

        return Optional.of(new CornerPair(top.get(0), top.get(1)));
    }
}
