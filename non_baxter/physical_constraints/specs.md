# Xylophone Representation

The following is used to represent a xylophone's keys:

1. Choose an origin (generally the bottom left-hand corner of the xylophone).
2. Measure the xy-distance from that origin to the center of each key.

Measurements are reported as an array of (note,x,y) triplets (hashmaps in Python). For example,

```python
[
  {
    'note': 'A',
    'x': 10
    'y': 10
  }, ...
]
```

