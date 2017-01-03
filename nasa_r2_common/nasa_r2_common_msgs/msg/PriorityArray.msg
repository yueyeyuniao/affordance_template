# axis_priorities must be 0, 1, or 6 elements
# empty defaults to high, 1 sets all axes equal priority, 6 sets each axis (x, y, z, r, p, y)
uint8[]  axis_priorities

uint8 IGNORE          = 0   # Exclude from trajectory
uint8 CRITICAL        = 1   # Most important priority in trajectory
uint8 HIGH            = 2   # Second most important priority in trajectory
uint8 MEDIUM          = 3   # Medium prioirty in trajectory
uint8 LOW             = 4   # Low priority in trajectory
uint8 OPT             = 5   # Optimize trajectory (solve for, but do not iterate to achieve)

#uint8 IGNORE = 0
#uint8 LOW = 1
#uint8 MEDIUM = 128
#uint8 HIGH = 254
#uint8 CRITICAL = 255
