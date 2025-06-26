import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# === 순서 기반 고정 참값 테이블 ===
ground_truths_ordered = [
    (-97.398834228515625, 68.428794860839844, 3.8531584739685059),
    (-61.844890594482422, 80.143608093261719, 7.8816661834716797),
    (-70.514152526855469, 74.3680419921875, 15.418899536132812),
    (-71.1409912109375, 74.0306396484375, 20.173868179321289),
    (-72.541465759277344, 74.878143310546875, 23.489242553710938),
    (-105.07278442382812, 100.77210998535156, 23.53996467590332),
    (-103.56498718261719, 99.733726501464844, 19.807872772216797),
    (-101.25800323486328, 100, 12.22029972076416),
    (-96.923896789550781, 104.27999877929688, 8.5504398345947266),
    (-78.855506896972656, 109.81887817382812, 3.8079309463500977),
]

input_csv_path = 'marker_location.csv'
output_csv_path = 'evaluation_results.csv'

def evaluate_csv_predictions(input_csv_path, output_csv_path):
    with open(input_csv_path, 'r') as f:
        reader = csv.reader(f)
        predictions = [list(map(float, row)) for row in reader if len(row) >= 3]

    results = []
    pred_points = []
    true_points = []

    for idx, row in enumerate(predictions):
        if idx >= len(ground_truths_ordered):
            print(f"🔺 예측 수({len(predictions)})가 참값 수({len(ground_truths_ordered)})보다 많습니다. 중단합니다.")
            break

        x_pred, y_pred, z_pred = row[:3]
        x_true, y_true, z_true = ground_truths_ordered[idx]

        dx, dy, dz = x_pred - x_true, y_pred - y_true, z_pred - z_true
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        success = "성공" if dist <= 1.5 else "실패"

        rel_err_x = abs(dx) / abs(x_true) * 100 if x_true else 0
        rel_err_y = abs(dy) / abs(y_true) * 100 if y_true else 0
        rel_err_z = abs(dz) / abs(z_true) * 100 if z_true else 0

        acc_x = 100 - rel_err_x
        acc_y = 100 - rel_err_y
        acc_z = 100 - rel_err_z

        results.append([
            idx,
            x_pred, y_pred, z_pred,
            x_true, y_true, z_true,
            rel_err_x, rel_err_y, rel_err_z,
            acc_x, acc_y, acc_z,
            dist, success
        ])

        pred_points.append([x_pred, y_pred, z_pred])
        true_points.append([x_true, y_true, z_true])

    file_exists = os.path.exists(output_csv_path)
    with open(output_csv_path, 'a', newline='') as f_out:
        writer = csv.writer(f_out)
        if not file_exists:
            writer.writerow([
                "index",
                "x_pred", "y_pred", "z_pred",
                "x_true", "y_true", "z_true",
                "rel_err_x(%)", "rel_err_y(%)", "rel_err_z(%)",
                "acc_x(%)", "acc_y(%)", "acc_z(%)",
                "3D_error(m)", "성공여부"
            ])
        writer.writerows(results)

    print(f"✅ 결과가 저장되었습니다: {output_csv_path}")

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
