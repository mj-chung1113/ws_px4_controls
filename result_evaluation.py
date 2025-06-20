import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
# === 고정 참값 테이블 (ID 기준 정렬) ===
ground_truths = {
    0: (-93.408805847167969, 67.470802307128906, 3.853132963180542),
    1: (-74.44219970703125, 73.996116638183594, 23.234735488891602),
    2: (-64.03082275390625, 79.127532958984375, 8.4990978240966797),
    3: (-81.793121337890625, 112.42037963867188, 3.8079538345336914),
    4: (-96.923896789550781, 104.27999877929688, 8.5504398345947266),
    5: (-108.13301086425781, 99.353309631347656, 23.136356353759766),
}

input_csv_path = 'marker_location.csv'
output_csv_path = 'evaluation_results.csv'

def evaluate_csv_predictions(input_csv_path, output_csv_path):
    predictions = []
    with open(input_csv_path, 'r') as f:
        reader = csv.reader(f)
        predictions = [row for row in reader if row]

    results = []
    pred_points = []
    true_points = []

    for row in predictions:
        if len(row) < 4:
            continue

        try:
            x_pred, y_pred, z_pred, id_str = map(float, row)
        except ValueError:
            continue

        id_val = int(id_str)
        if id_val not in ground_truths:
            print(f"ID {id_val} not found in ground truth. Skipping.")
            continue

        x_true, y_true, z_true = ground_truths[id_val]
        dx = x_pred - x_true
        dy = y_pred - y_true
        dz = z_pred - z_true

        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        success = "성공" if dist <= 1.5 else "실패"

        rel_err_x = abs(dx) / abs(x_true) * 100 if x_true else 0
        rel_err_y = abs(dy) / abs(y_true) * 100 if y_true else 0
        rel_err_z = abs(dz) / abs(z_true) * 100 if z_true else 0

        acc_x = 100 - rel_err_x
        acc_y = 100 - rel_err_y
        acc_z = 100 - rel_err_z

        results.append([
            id_val,
            x_pred, y_pred, z_pred,
            x_true, y_true, z_true,
            rel_err_x, rel_err_y, rel_err_z,
            acc_x, acc_y, acc_z,
            dist, success
        ])

        pred_points.append([x_pred, y_pred, z_pred])
        true_points.append([x_true, y_true, z_true])

    # 정렬
    results.sort(key=lambda r: r[0])

    file_exists = os.path.exists(output_csv_path)
    with open(output_csv_path, 'a', newline='') as f_out:
        writer = csv.writer(f_out)

        if not file_exists:
            writer.writerow([
                "id",
                "x_pred", "y_pred", "z_pred",
                "x_true", "y_true", "z_true",
                "rel_err_x(%)", "rel_err_y(%)", "rel_err_z(%)",
                "acc_x(%)", "acc_y(%)", "acc_z(%)",
                "3D_error(m)", "성공여부"
            ])

        writer.writerows(results)

    print(f"✅ 결과가 저장되었습니다: {output_csv_path}")

    # # 입력 CSV 초기화 (예측값 삭제)
    # with open(input_csv_path, 'w') as f:
    #     pass

    # 3D 시각화
    pred_points = np.array(pred_points)
    true_points = np.array(true_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*pred_points.T, color='red', label='Predicted')
    ax.scatter(*true_points.T, color='blue', label='Ground Truth')

    for p, t in zip(pred_points, true_points):
        ax.plot([p[0], t[0]], [p[1], t[1]], [p[2], t[2]], color='gray', linestyle='--', linewidth=1)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Prediction vs Ground Truth")
    ax.legend()
    plt.tight_layout()
    plt.show()

# 실행
evaluate_csv_predictions(input_csv_path, output_csv_path)
