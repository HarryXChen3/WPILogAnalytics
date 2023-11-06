// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.analytics.PDHWattHours;
import frc.robot.analytics.Statistic;
import frc.robot.analytics.SwerveModuleDrivenDistance;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.io.File;
import java.nio.file.FileSystem;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

public class Robot extends LoggedRobot {
    public static final String WPILOG_EXTENSION = "wpilog";

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        this.robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        this.autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    private String getFileExtension(final File file) {
        final String name = file.getName();
        final int lastIndexOf = name.lastIndexOf(".");

        return lastIndexOf == -1 ? "" : name.substring(lastIndexOf);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        final FileNameExtensionFilter extensionFilter = new FileNameExtensionFilter(
                "WPILOG", WPILOG_EXTENSION
        );

        final String extensionsAsStrings = Arrays.stream(extensionFilter.getExtensions())
                .map(s -> "*." + s)
                .collect(Collectors.joining(";"));

        final FileDialog dialog = new FileDialog((Frame) null, "WPILOG", FileDialog.LOAD);
        final List<File> selectedFiles = new ArrayList<>();
        try {
            dialog.setDirectory(Filesystem.getOperatingDirectory().getAbsolutePath());
            dialog.setMultipleMode(true);
            dialog.setFile(extensionsAsStrings);
            dialog.setFilenameFilter((file, name) -> extensionFilter.accept(file));
            dialog.setVisible(true);

            final File[] files = dialog.getFiles();
            for (final File file : files) {
                if (file.isFile() && getFileExtension(file).equalsIgnoreCase("." + WPILOG_EXTENSION)) {
                    selectedFiles.add(file);
                }
            }
        } finally {
            dialog.setVisible(false);
            dialog.dispose();
        }

        final List<WPILOGReader> readers = selectedFiles.stream()
                .map(file -> new WPILOGReader(file.getAbsolutePath()))
                .toList();

        final Statistic<Map<String, Double>> statistic = new SwerveModuleDrivenDistance();
        final List<Map<String, Double>> moduleDrivenDistanceMetersMapsByLog = statistic.compute(readers);

        final Map<String, Double> sumByLog = SwerveModuleDrivenDistance.sumByLog(moduleDrivenDistanceMetersMapsByLog);

        System.out.printf(
                "Total distance driven (averaged over modules): %.2fm%n",
                SwerveModuleDrivenDistance.sumOfAveragesByLog(moduleDrivenDistanceMetersMapsByLog)
        );
        System.out.printf(
                "Distances driven (by module, in meters): %s%n",
                Arrays.toString(sumByLog.entrySet().toArray())
        );
        System.out.printf(
                "Total distance driven (of all modules): %.2fm%n",
                sumByLog.values().stream().mapToDouble(Double::doubleValue).sum()
        );

        final Statistic<Double> wattHours = new PDHWattHours();
        final List<Double> wattHoursByLog = wattHours.compute(readers);

        System.out.printf("%s%n", Arrays.toString(wattHoursByLog.toArray()));
    }

    @Override
    public void testPeriodic() {

    }
}
