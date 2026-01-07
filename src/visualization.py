import matplotlib.pyplot as plt
import pandas as pd

# --- Ground truth trajectory ---
truth = pd.read_csv("robot.csv")
tx = truth["x"]
ty = truth["y"]

# --- True landmarks ---
landmarks = pd.read_csv("landmarks.csv")
lx = landmarks["x"]
ly = landmarks["y"]

# --- EKF-SLAM estimate ---
ex, ey = [], []
final_landmarks = []

with open("ekf_slam_state.csv") as f:
    for line in f:
        vals = [float(v) for v in line.strip().split(",")]
        ex.append(vals[0])
        ey.append(vals[1])
        final_landmarks = vals[3:]  # overwrite → last timestep only

# Convert EKF landmarks (last state)
ekf_lx = final_landmarks[0::2]
ekf_ly = final_landmarks[1::2]

# --- Plot ---
plt.figure(figsize=(6, 6))

plt.plot(tx, ty, "k--", label="True trajectory")
plt.plot(ex, ey, "r", label="EKF trajectory")

plt.scatter(lx, ly, c="green", marker="*", s=100, label="True landmarks")
plt.scatter(ekf_lx, ekf_ly, c="red", marker="x", label="EKF landmarks")

plt.axis("equal")
plt.legend()
plt.title("EKF-SLAM: Truth vs Estimate")

plt.savefig("ekf_slam_result.png")
plt.show()
