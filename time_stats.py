import numpy as np

# filename = "high_priority_perf_governor.txt"
# filename = "higher_freq.txt"
filename = "std_sleep_2.txt"


# Load the timestamps from the file
with open(filename, "r") as file:
    timestamps = np.array([float(line.strip()) for line in file])

# Calculate differences between sequential timestamps
differences = np.diff(timestamps)

# Calculate mean and standard deviation
mean_diff = np.mean(differences)
stddev_diff = np.std(differences)

# Calculate variance and coefficient of variation
variance_diff = np.var(differences)
cv_diff = stddev_diff / mean_diff if mean_diff else 0

print(f"Mean difference: {mean_diff}")
print(f"Standard deviation of difference: {stddev_diff}")
print(f"Variance of difference: {variance_diff}")
print(f"Coefficient of Variation: {cv_diff}")
print(f"Max difference: {np.max(differences)}")
print(f"Min difference: {np.min(differences)}")
