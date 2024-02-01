package frc.robot;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LiveDoubleBinding {
    DoubleSubscriber valueSubscriber;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shuffleboardTable = inst.getTable("Shuffleboard");

    public LiveDoubleBinding(String tabName, String key, Double defaultValue, Consumer<NetworkTableEvent> listener) {

        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kNumberSlider);
        valueSubscriber = shuffleboardTable.getDoubleTopic(tabName + "/" + key).subscribe(defaultValue);

        if (listener != null) {
            inst.addListener(
                    valueSubscriber,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    listener);
        }
    }

    public LiveDoubleBinding(String tabName, String key, Double defaultValue) {
        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kNumberSlider);
        valueSubscriber = shuffleboardTable.getDoubleTopic(key).subscribe(defaultValue);
    }

    DoubleSubscriber getSubscriber() {
        return valueSubscriber;
    }

    double getDouble() {
        return valueSubscriber.get();
    }

    double getDouble(double defaultValue) {
        return valueSubscriber.get(defaultValue);
    }
}