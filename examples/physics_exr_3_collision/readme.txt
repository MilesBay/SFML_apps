The Separating Axis Theorem (SAT), as used for convex polygon collision in Physics.cpp, boils down to:

Gather axes to test — for each edge of both polygons, get a perpendicular (normal) direction. These are the candidate "separating axes."
Project both shapes onto each axis — for each axis, project every vertex of both polygons onto it and keep the min/max, giving each shape a 1D interval on that axis.
Check for a gap — if the two intervals don't overlap on any axis, that axis separates the shapes → no collision, stop early.
If every axis overlaps — the shapes are colliding. Track the axis with the smallest overlap — that's the Minimum Translation Vector (MTV), used to push the shapes apart.
Resolve — use that axis + overlap amount to separate the bodies and apply collision response (impulses, etc.).
Rule of thumb: one gap on any axis = no collision; overlap on all axes = collision, and the smallest overlap tells you how to push them apart.