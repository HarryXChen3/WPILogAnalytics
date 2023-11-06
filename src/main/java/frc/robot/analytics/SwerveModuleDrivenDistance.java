package frc.robot.analytics;

import frc.robot.Constants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class SwerveModuleDrivenDistance implements Statistic<Map<String, Double>> {
    private static final String SWERVE_INPUTS = "Swerve";
    private static final List<String> MODULE_INPUTS = List.of(
            "Module_FrontLeft",
            "Module_FrontRight",
            "Module_BackLeft",
            "Module_BackRight"
    );
    private static final String DRIVE_POSITION_ROTS = "DrivePositionRots";

    private static double rotationsToLinearDistance(final double rotations) {
        return rotations * Constants.Modules.WHEEL_CIRCUMFERENCE;
    }

    public static Stream<DoubleStream> asStreamOfDoubleStreams(final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog) {
        return moduleDrivenDistanceMetersMapsByLog.stream()
                .map(map -> map.values().stream().mapToDouble(Double::doubleValue));
    }

    public static List<Double> averageByLog(final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog) {
        return asStreamOfDoubleStreams(moduleDrivenDistanceMetersMapsByLog)
                .map(stream -> stream.average().orElseThrow())
                .toList();
    }

    public static double sumOfAveragesByLog(final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog) {
        return averageByLog(moduleDrivenDistanceMetersMapsByLog).stream()
                .mapToDouble(Double::doubleValue)
                .sum();
    }

    public static Map<String, Double> sumByLog(final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog) {
        return moduleDrivenDistanceMetersMapsByLog.stream()
                .flatMap(moduleMap -> moduleMap.entrySet().stream())
                .collect(Collectors.groupingBy(Map.Entry::getKey, Collectors.summingDouble(Map.Entry::getValue)));
    }

    @Override
    public List<Map<String, Double>> compute(final List<WPILOGReader> readers) {
        final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog = new ArrayList<>(readers.size());

        for (final WPILOGReader reader : readers) {
            final LogTable entry = new LogTable(0);
            reader.start();

            final Map<String, Double> moduleLastDistanceMetersMap = new HashMap<>(MODULE_INPUTS.size());
            final Map<String, Double> moduleDrivenDistanceMetersMap = new HashMap<>(MODULE_INPUTS.size());

            while (reader.updateTable(entry)) {
                final LogTable swerve = entry.getSubtable(SWERVE_INPUTS);

                for (final String moduleTableName : MODULE_INPUTS) {
                    final LogTable moduleTable = swerve.getSubtable(moduleTableName);

                    final double nextPositionRots = moduleTable.get(DRIVE_POSITION_ROTS, -1d);
                    if (nextPositionRots == -1) {
                        continue;
                    }

                    final double nextDistanceMeters = rotationsToLinearDistance(nextPositionRots);
                    final Double lastDistanceDoubleObject =
                            moduleLastDistanceMetersMap.put(moduleTableName, nextDistanceMeters);
                    final double lastDistanceMeters = lastDistanceDoubleObject == null ? 0d : lastDistanceDoubleObject;

                    final double drivenDistanceMeters = Math.abs(nextDistanceMeters - lastDistanceMeters);
                    moduleDrivenDistanceMetersMap.compute(
                            moduleTableName,
                            (key, last) -> last != null ? last + drivenDistanceMeters : drivenDistanceMeters
                    );
                }
            }

            moduleDrivenDistanceMetersMapsByLog.add(moduleDrivenDistanceMetersMap);
        }

        return moduleDrivenDistanceMetersMapsByLog;
    }
}
