package frc.robot.libs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

public class DiagnosticTable {
    NetworkTable table;
    HashMap<String, NetworkTableEntry> entryMap;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public DiagnosticTable(String tabKey) {
        table = inst.getTable(tabKey);
        entryMap = new HashMap<>();
    }

    public void putNumber(String key, double value) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultDouble(value);
        }
        entryMap.get(key).setDouble(value);
    }

    public double getNumber(String key, double defaultValue) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultDouble(defaultValue);
        }
        return entryMap.get(key).getDouble(defaultValue);
    }

    public void putBoolean(String key, boolean value) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultBoolean(value);
        }
        entryMap.get(key).setBoolean(value);
    }

    public boolean getBoolean(String key, boolean defaultValue) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultBoolean(defaultValue);
        }
        return entryMap.get(key).getBoolean(defaultValue);
    }

    public void putString(String key, String value) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultString(value);
        }
        entryMap.get(key).setString(value);
    }

    public String getString(String key, String defaultValue) {
        if(!entryMap.containsKey(key)) {
            entryMap.put(key, table.getEntry(key));
            entryMap.get(key).setDefaultString(defaultValue);
        }
        return entryMap.get(key).getString(defaultValue);    }

}
