//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.math.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.math.Velocity;
//
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//public class SOTMUtil {
//    private final Pose goal;
//    private final InterpolatedLookupTable tofLUT;
//    private final InterpolatedLookupTable velocityLUT;
//    public SOTMUtil(Pose goal) {
//        this.goal = goal;
//        tofLUT = Shooter.Companion.getTimeInAirLookupTable();
//        velocityLUT = Shooter.Companion.getVelocityLookupTable();
//    }
//    private double sampleRate(
//            InterpolatedLookupTable map, double distance, double distanceVelocity) {
//        return (map.calculate(distance + distanceVelocity * ShootingConstants.SAMPLING_DT)
//                - map.calculate(distance))
//                / ShootingConstants.SAMPLING_DT;
//    }
//    public ShootingConstants.ShooterOutputs calculateShooterOutputs(Pose turretPose, Velocity currentSpeeds, Vector robotAcceleration, double angularVelocity, double dt, Alliance alliance) {
//        double tof = tofLUT.calculate(turretPose.distance(goal));
//
//        Pose virtualGoal = new Pose(goal.x()-currentSpeeds.vx*tof, goal.y()-currentSpeeds.vy*tof);
//
//        double dx = virtualGoal.x() - turretPose.x();
//        double dy = virtualGoal.y() - turretPose.y();
//        double r2 = dx * dx + dy * dy;
//        double distance = Math.sqrt(r2);
//
//        // Derivatives of dx and dy over time
//        double dxDot = currentSpeeds.vx;
//        double dyDot = currentSpeeds.vy;
//
//        // d/dt of atan2(-dx, dy)
//        double turretAngleRate = -(dy * (-dxDot) - (-dx) * dyDot) / r2;
//
//        // subtract robot angular velocity since turret angle is robot-relative
//        double wantedTurretVelocity = turretAngleRate - angularVelocity;
//        // MathUtil.lerp(Math.toRadians(0), Math.toRadians(-3.5), (distance - 70) / 160);
//        double wantedTurretAngle = Math.atan2(-(virtualGoal.x() - turretPose.x()), virtualGoal.y() - turretPose.y()) - turretPose.getHeading() + Math.toRadians(90) + turretOffset;
//
//        // take derivative with respect to time of distance (sqrt(dx^2 + dy^2))
//        // double distanceRate = -(dx * dxDot + dy * dyDot) / distance;
//        double distanceRate = (dx * dxDot + dy * dyDot) / distance;
//
//        double wantedWheelSpeed = velocityLUT.calculate(distance) * ShootingConstants.wheelSpeedMultiplier;
//        double wantedWheelAcceleration = sampleRate(velocityLUT, distance, distanceRate);
//
//        return new ShootingConstants.ShooterOutputs(
//                wantedTurretAngle,
//                wantedTurretVelocity,
//                wantedWheelSpeed,
//                wantedWheelAcceleration
//        );
//    }
//}