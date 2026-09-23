import pandas as pd
import numpy as np


CSV_PATH = "calibration_data.csv"


def safe_mean(values):
    values = [v for v in values if not np.isnan(v)]
    if len(values) == 0:
        return None
    return sum(values) / len(values)


def analyze_angular(df):
    angular_modes = ["ANGULAR_POSITIVE", "ANGULAR_NEGATIVE"]
    results = []

    for mode in angular_modes:
        data = df[df["mode"] == mode].copy()
        data = data[data["target_found"] == 1]
        data = data.dropna(subset=["alpha_rad", "w_cmd", "elapsed_trial"])

        if len(data) < 5:
            continue

        t0 = data["elapsed_trial"].iloc[0]
        tf = data["elapsed_trial"].iloc[-1]
        dt = tf - t0

        alpha0 = data["alpha_rad"].iloc[0]
        alphaf = data["alpha_rad"].iloc[-1]

        w = data["w_cmd"].iloc[0]

        if abs(w) < 1e-6 or dt <= 0:
            continue

        delta_alpha = alphaf - alpha0

        # Modelo esperado aproximado:
        # alpha(k+1) = alpha(k) - K_alpha * w * dt
        # Si K_alpha queda cerca de 1, el modelo alpha_dot = -w es razonable.
        k_alpha = -delta_alpha / (w * dt)

        results.append(k_alpha)

        print("\n--- Analisis angular ---")
        print("Modo:", mode)
        print("alpha inicial:", alpha0)
        print("alpha final:", alphaf)
        print("delta alpha:", delta_alpha)
        print("w_cmd:", w)
        print("dt:", dt)
        print("K_alpha estimado:", k_alpha)

    return safe_mean(results)


def analyze_linear(df):
    data = df[df["mode"] == "LINEAR_FORWARD"].copy()
    data = data[data["target_found"] == 1]
    data = data.dropna(subset=["rho_m", "v_cmd", "elapsed_trial"])

    if len(data) < 5:
        return None

    t0 = data["elapsed_trial"].iloc[0]
    tf = data["elapsed_trial"].iloc[-1]
    dt = tf - t0

    rho0 = data["rho_m"].iloc[0]
    rhof = data["rho_m"].iloc[-1]

    v = data["v_cmd"].iloc[0]

    if abs(v) < 1e-6 or dt <= 0:
        return None

    delta_rho = rhof - rho0

    # Modelo esperado:
    # rho(k+1) = rho(k) - K_rho * v * dt
    # Si K_rho queda cerca de 1, el modelo rho_dot = -v es razonable.
    k_rho = -delta_rho / (v * dt)

    print("\n--- Analisis lineal ---")
    print("rho inicial:", rho0)
    print("rho final:", rhof)
    print("delta rho:", delta_rho)
    print("v_cmd:", v)
    print("dt:", dt)
    print("K_rho estimado:", k_rho)

    return k_rho


def analyze_pixel_area_model(df):
    print("\n--- Modelo visual simple por pixeles y area ---")

    # Relacion w -> e_x
    angular = df[df["mode"].isin(["ANGULAR_POSITIVE", "ANGULAR_NEGATIVE"])].copy()
    angular = angular[angular["target_found"] == 1]
    angular = angular.dropna(subset=["error_x_px", "w_cmd", "elapsed_trial"])

    k_w_values = []

    for mode in angular["mode"].unique():
        data = angular[angular["mode"] == mode]

        if len(data) < 5:
            continue

        t0 = data["elapsed_trial"].iloc[0]
        tf = data["elapsed_trial"].iloc[-1]
        dt = tf - t0

        ex0 = data["error_x_px"].iloc[0]
        exf = data["error_x_px"].iloc[-1]
        w = data["w_cmd"].iloc[0]

        if abs(w) < 1e-6 or dt <= 0:
            continue

        k_w = (exf - ex0) / (w * dt)
        k_w_values.append(k_w)

    k_w_mean = safe_mean(k_w_values)

    if k_w_mean is not None:
        print("K_w_to_ex estimado:", k_w_mean, "px/rad")
    else:
        print("No se pudo estimar K_w_to_ex.")

    # Relacion v -> area
    linear = df[df["mode"] == "LINEAR_FORWARD"].copy()
    linear = linear[linear["target_found"] == 1]
    linear = linear.dropna(subset=["area", "v_cmd", "elapsed_trial"])

    if len(linear) >= 5:
        t0 = linear["elapsed_trial"].iloc[0]
        tf = linear["elapsed_trial"].iloc[-1]
        dt = tf - t0

        a0 = linear["area"].iloc[0]
        af = linear["area"].iloc[-1]
        v = linear["v_cmd"].iloc[0]

        if abs(v) > 1e-6 and dt > 0:
            k_v_area = (af - a0) / (v * dt)
            print("K_v_to_area estimado:", k_v_area, "px^2/m")
        else:
            print("No se pudo estimar K_v_to_area.")
    else:
        print("No se pudo estimar K_v_to_area.")


def main():
    df = pd.read_csv(CSV_PATH)

    print("Archivo cargado:", CSV_PATH)
    print("Filas:", len(df))

    k_alpha = analyze_angular(df)
    k_rho = analyze_linear(df)

    print("\n==============================")
    print("RESULTADOS PARA MODELO MPC REAL")
    print("==============================")

    if k_alpha is not None:
        print("K_alpha promedio:", k_alpha)
    else:
        print("No se pudo estimar K_alpha.")

    if k_rho is not None:
        print("K_rho:", k_rho)
    else:
        print("No se pudo estimar K_rho.")

    print("\nModelo sugerido:")
    print("rho(k+1) = rho(k) - K_rho * v(k) * dt")
    print("alpha(k+1) = alpha(k) - K_alpha * w(k) * dt")

    analyze_pixel_area_model(df)


if __name__ == "__main__":
    main()