package frc.robot.analytics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class ChargeStation implements Statistic<ChargeStation.ChargeStationInfo> {
    private static final String DRIVER_STATION = "DriverStation";
    private static final String ALLIANCE_STATION = "AllianceStation";
    private static final String MATCH_TIME = "MatchTime";
    private static final String AUTONOMOUS = "Autonomous";
    private static final String ENABLED = "Enabled";

    private static final String DASHBOARD_INPUTS = "DashboardInputs";
    private static final String AUTO_SELECTOR = "AutoSelector";

    private static final String GYRO = "Gyro";
    private static final String PITCH_POSITION_DEG = "PitchPositionDeg";

    private static final String REAL_OUTPUTS = "RealOutputs";

    private static final String ODOMETRY = "Odometry";
    private static final String ROBOT_2D = "Robot2d";

    private static boolean isRedAlliance(final LogTable entry) {
        // taken from...
        // https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/shared/log/LogUtil.ts#L137-L156
        return entry.getSubtable(DRIVER_STATION).get(ALLIANCE_STATION, 3L) <= 2;
    }

    private static boolean poseInArea(
            final Pose2d pose2d,
            final Translation2d blPoint,
            final Translation2d trPoint
    ) {
        return pose2d.getX() > blPoint.getX()
                && pose2d.getX() < trPoint.getX()
                && pose2d.getY() > blPoint.getY()
                && pose2d.getY() < trPoint.getY();
    }

    private static Pose2d flipPose(final Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(16.54175, 8.0137),
                new Rotation2d(Math.PI)
        ));
    }

    public static class ChargeStationInfo {
        private boolean maybeBalancePath;
        private boolean attemptedCharge;

        public ChargeStationInfo() {}

        public boolean isMaybeBalancePath() {
            return maybeBalancePath;
        }
        public void setMaybeBalancePath(boolean maybeBalancePath) {
            this.maybeBalancePath = maybeBalancePath;
        }
        public boolean isAttemptedCharge() {
            return attemptedCharge;
        }
        public void setAttemptedCharge(boolean attemptedCharge) {
            this.attemptedCharge = attemptedCharge;
        }
    }

    @Override
    public List<ChargeStationInfo> compute(final List<WPILOGReader> readers) {
        final List<ChargeStationInfo> chargeStationInfoByLog = new ArrayList<>(readers.size());

        for (final WPILOGReader reader : readers) {
            final LogTable entry = new LogTable(0);
            reader.start();

            final ChargeStationInfo chargeStationInfo = new ChargeStationInfo();
            while (reader.updateTable(entry)) {
                final boolean isRed = isRedAlliance(entry);

                final LogTable driverstation = entry.getSubtable(DRIVER_STATION);

                final long matchTime = driverstation.get(MATCH_TIME, 0L);

                final boolean isEnabled = driverstation.get(ENABLED, false);
                final boolean logAutonomous = driverstation.get(AUTONOMOUS, false);

                final boolean isAutonomous = logAutonomous && isEnabled;
                final boolean isEndgame = !logAutonomous && isEnabled && matchTime <= 30;

                final double pitchPositionDeg = entry.getSubtable(GYRO)
                        .get(PITCH_POSITION_DEG, Double.POSITIVE_INFINITY);

                final LogTable realOutputs = entry.getSubtable(REAL_OUTPUTS);
                final Pose2d robotOdometryPose2d = realOutputs.getSubtable(ODOMETRY)
                        .get(ROBOT_2D, new Pose2d());
                final Pose2d blueSidePose = isRed ? flipPose(robotOdometryPose2d) : robotOdometryPose2d;

                final boolean isOnChargeStation = poseInArea(
                        blueSidePose,
                        new Translation2d(2.92, 1.50),
                        new Translation2d(4.86, 4.01)
                );
                // >= 12 deg should mean that we're trying to balance
                final boolean isAttemptingChargeStation = isOnChargeStation && Math.abs(pitchPositionDeg) >= 12;
                // 4 deg of freedom on the charge station
                final boolean isBalancedOnChargeStation = isOnChargeStation && Math.abs(pitchPositionDeg) <= 4;

                // handle autonomous balance
                if (isAutonomous && matchTime <= 1) {
                    // end of auto, 1 second left

                    final LogTable dashboardInputs = entry.getSubtable(DASHBOARD_INPUTS);
                    final String autoSelector = dashboardInputs.get(AUTO_SELECTOR, "");

                    // maybe we can catch a path that has "charge" or "bal" or "balance" in it
                    final boolean maybeBalancePath = Stream.of("charge", "bal")
                            .anyMatch(key -> autoSelector.toLowerCase().contains(key));

                    if (maybeBalancePath && isAttemptingChargeStation && !chargeStationInfo.isAttemptedCharge()) {
                        chargeStationInfo.setAttemptedCharge(true);
                    }
                }
            }
        }

        return chargeStationInfoByLog;
    }
}
