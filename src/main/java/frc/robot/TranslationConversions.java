package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class TranslationConversions {

    /** Utility class, so constructor is private. */
    private TranslationConversions() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Translation2d xboxToCartesian(Translation2d translation) {
        return cartesianToXbox(translation);
    }

    public static Translation2d cartesianToXbox(Translation2d translation) {
        return new Translation2d(translation.getX(), -translation.getY());
    }

    public static Translation2d xboxToRobot(Translation2d translation) {
        return new Translation2d(-translation.getY(), -translation.getX());
    }

    public static Translation2d robotToXbox(Translation2d translation) {
        return xboxToRobot(translation);
    }

    public static Translation2d robotToCartesian(Translation2d translation) {
        return cartesianToRobot(translation);
    }

    public static Translation2d cartesianToRobot(Translation2d translation) {
        return new Translation2d(translation.getY(), -translation.getX());
    }
}
