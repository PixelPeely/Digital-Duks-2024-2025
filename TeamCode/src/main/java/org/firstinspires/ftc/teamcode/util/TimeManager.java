package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;

//All measurements are in seconds
public class TimeManager {
    private static double initTime;
    private static double currentTime = -1;
    private static double deltaTime = 1;

    private static List<Predicate<Double>> tasks = new ArrayList<>();
    private static List<Predicate<Double>> recentTasks = new ArrayList<>();

    //Called at the end of every tick
    public static void onTick(double time) {
        if (currentTime != -1) deltaTime = time - currentTime;
        else initTime = time;
        currentTime = time;
        tasks.addAll(recentTasks);
        recentTasks.clear();
        tasks.removeIf(task -> task.test(getTime(false)));
    }

    public static double getTime(boolean includeInit) {
        return currentTime - (includeInit ? 0 : initTime);
    }

    //Time it took to run the LAST cycle
    public static double getDeltaTime() {
        return deltaTime;
    }

    //TODO substitute for a command scheduler?
    public static void hookTick(Predicate<Double> task) {
        recentTasks.add(task);
    }

    public static void hookFuture(double offset, Predicate<Double> task) {
        final double scheduleTime = getTime(false);
        recentTasks.add(time -> (time > scheduleTime + offset && task.test(time)));
    }

    public static void hookPeriodic(double interval, Predicate<Double> task) {
        AtomicInteger iterations = new AtomicInteger(0);
        final double scheduleTime = getTime(false);
        recentTasks.add(time -> {
            if (time > scheduleTime + interval * iterations.get()) {
                iterations.getAndIncrement();
                return task.test(time);
            }
            return false;
        });
    }

    public static void cancelTask(Predicate<Double> task) {
        tasks.removeIf(t -> t.equals(task));
        recentTasks.removeIf(t -> t.equals(task));
    }

    public static void reset() {
        currentTime = -1;
        deltaTime = 1;
        tasks.clear();
    }
}
