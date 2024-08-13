package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Logger {
    //./adb pull /sdcard/FIRST/java/src/runtimeLogs/teleop.txt C:\Users\jerem\Documents\Coding\Digital-Duks-2024-2025\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\latestLog
    public enum LOG_TYPE {
        INFO,
        WARNING,
        ERROR
    }

    public static class LogEntry {
        public final String message;
        public final LOG_TYPE type;
        public final double runTime;

        public LogEntry(String _message, LOG_TYPE _type) {
            message = _message;
            type = _type;
            runTime = TimeManager.getTime(false);
        }
    }

    private static List<LogEntry> logEntries = new ArrayList<>();

    public static void addEntry(LogEntry logEntry) {
        logEntries.add(logEntry);
    }

    @SuppressLint("NewApi")
    public static void writeLog(boolean auto) {
        new File(DukConstants.DEBUG.LOG_DIRECTORY).mkdir();
        String filePath = DukConstants.DEBUG.LOG_DIRECTORY + (auto ? "/autonomous.txt" : "/teleop.txt");

        try(FileWriter writer = new FileWriter(filePath)) {
            writer.write(String.format("Epoch Time: %1$s\n", System.currentTimeMillis() * 0.001));
            for (LogEntry entry : logEntries) {
                writer.write(String.format(Locale.US, "(%3.3f|%2$s)\t%3$s\n", entry.runTime, entry.type, entry.message));
            }
        } catch (IOException e) {
            e.printStackTrace();
            DashboardInterface.immediateError("Encountered IOException while writing log file!", e);
        }
        logEntries.clear();
    }
}
