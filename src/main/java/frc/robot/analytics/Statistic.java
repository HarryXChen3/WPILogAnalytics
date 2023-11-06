package frc.robot.analytics;

import org.littletonrobotics.junction.wpilog.WPILOGReader;

import java.util.List;

@FunctionalInterface
public interface Statistic<T> {
    List<T> compute(final List<WPILOGReader> readers);
}
