package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     */
    public static void drawRobot(Pose2D pose) {
        panelsField.moveCursor(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        panelsField.circle(ROBOT_RADIUS);

        sendPacket();
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}