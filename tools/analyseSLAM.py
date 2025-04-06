#!/usr/bin/env python3
import pandas as pd
import numpy as np
import argparse

def analyze_odometry(csv_path):
    # 读取 CSV 文件
    df = pd.read_csv(csv_path, header=None)

    # 位置列：x, y, z（第5,6,7列，索引从0开始是4,5,6）
    positions = df.iloc[:, 4:7].values
    position_std = np.std(positions, axis=0)

    # 姿态列：四元数 x, y, z, w（第8,9,10,11列，索引是7,8,9,10）
    quaternions = df.iloc[:, 7:11].values
    orientation_std = np.std(quaternions, axis=0)

    print("📍 位置标准差 (meters):")
    print(f"  X: {position_std[0]:.6f}")
    print(f"  Y: {position_std[1]:.6f}")
    print(f"  Z: {position_std[2]:.6f}")

    print("\n🧭 姿态标准差 (quaternion):")
    print(f"  qx: {orientation_std[0]:.6e}")
    print(f"  qy: {orientation_std[1]:.6e}")
    print(f"  qz: {orientation_std[2]:.6e}")
    print(f"  qw: {orientation_std[3]:.6e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze SLAM odometry CSV file.")
    parser.add_argument("csv_path", type=str, help="Path to SLAM odometry CSV file")
    args = parser.parse_args()

    analyze_odometry(args.csv_path)
