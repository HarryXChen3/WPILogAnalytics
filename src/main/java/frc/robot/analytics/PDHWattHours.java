package frc.robot.analytics;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import java.util.ArrayList;
import java.util.List;

public class PDHWattHours implements Statistic<Double> {
    private static final String PDH = "PowerDistribution";
    private static final String SYSTEM_STATS = "SystemStats";
    private static final String VOLTAGE = "BatteryVoltage";
    private static final String TOTAL_CURRENT = "TotalCurrent";

    private static double microsecondsToHours(final double microseconds) {
        return (microseconds / 1e+6) / 3600;
    }

    @Override
    public List<Double> compute(final List<WPILOGReader> readers) {
        final List<Double> wattHoursByLog = new ArrayList<>(readers.size());

        for (final WPILOGReader reader : readers) {
            final LogTable entry = new LogTable(0);
            reader.start();

            double initialTimestampMicroseconds = -1;
            double totalWatts = 0;
            while (reader.updateTable(entry)) {
                if (initialTimestampMicroseconds == -1) {
                    initialTimestampMicroseconds = entry.getTimestamp();
                }

                final LogTable pdh = entry.getSubtable(PDH);
                final LogTable systemStats = entry.getSubtable(SYSTEM_STATS);
                final double discreteWatts = systemStats.get(VOLTAGE, 0d) * pdh.get(TOTAL_CURRENT, 0d);

                totalWatts += discreteWatts;
            }

            wattHoursByLog.add(totalWatts * microsecondsToHours(entry.getTimestamp() - initialTimestampMicroseconds));
        }

        return wattHoursByLog;
    }
}