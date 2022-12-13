package com._604robotics.quixsam.mathematics;

import java.util.TreeMap;

import edu.wpi.first.math.Pair;

public class DoubleInterpolatableTreeMap<T> {
  private final TreeMap<Double, Interpolatable<T>> treeMap =
      new TreeMap<Double, Interpolatable<T>>();

  public DoubleInterpolatableTreeMap() {}

  public DoubleInterpolatableTreeMap(Pair<Double, Interpolatable<T>>... values) {
    set(values);
  }

  public void set(double key, Interpolatable<T> value) {
    treeMap.put(key, value);
  }

  public void set(Pair<Double, Interpolatable<T>>... values) {
    for (Pair<Double, Interpolatable<T>> value : values) {
      treeMap.put(value.getFirst(), value.getSecond());
    }
  }

  public void clear() {
    treeMap.clear();
  }

  public T get(double key) {
    if (treeMap.isEmpty()) return null;

    var value = treeMap.get(key);
    if (value != null) return value.get();

    // Returns the entry with the least key that is greater than or equal to the key.
    var topBound = treeMap.ceilingEntry(key);
    // Returns the entry with the greatest key that is less than or equal to the key.
    var bottomBound = treeMap.floorEntry(key);

    if (topBound == null) {
      return bottomBound.getValue().get();
    } else if (bottomBound == null) {
      return topBound.getValue().get();
    } else {
      return bottomBound
          .getValue()
          .interpolate(
              topBound.getValue(),
              (key - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey()));
    }
  }
}
