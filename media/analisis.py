import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# =========================================================
# CONFIGURACIÓN
# =========================================================

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else "mpc_results.csv"

STATE_COLORS = {
    "WAIT_COMMAND":     "#aaaaaa",
    "TRACK_TARGET":     "#2196F3",
    "COLLECTING":       "#4CAF50",
    "WAIT_NEXT_ACTION": "#FF9800",
    "GO_TO_WAYPOINT":   "#9C27B0",
    "RETURN_HOME":      "#F44336",
    "FINISHED":         "#009688",
}

# Referencia deseada de distancia
rho_ref = 0.08

# =========================================================
# CARGA DE DATOS
# =========================================================

df = pd.read_csv(CSV_PATH)

t = df["time"]

state_colors_mapped = df["state"].map(STATE_COLORS).fillna("#cccccc")

print(f"✔ {len(df)} filas cargadas de '{CSV_PATH}'")
print(f"✔ Estados presentes: {df['state'].unique().tolist()}")

# =========================================================
# PREPROCESAMIENTO
# =========================================================

rho_plot = df["rho"].copy()
rho_plot[rho_plot < 0] = np.nan

rho_error = rho_plot - rho_ref

alpha_plot = df["alpha"].copy()
alpha_plot[df["rho"] < 0] = np.nan

cost_plot = df["cost"].copy()
cost_plot[cost_plot <= 0] = np.nan

# =========================================================
# LEYENDA GLOBAL DE ESTADOS
# =========================================================

legend_patches = [
    mpatches.Patch(
        color=c,
        label=s,
        alpha=0.7
    )
    for s, c in STATE_COLORS.items()
    if s in df["state"].values
]

# =========================================================
# HELPER: FONDO POR ESTADOS
# =========================================================

def add_state_background(ax):

    prev_state = df["state"].iloc[0]
    start_t = t.iloc[0]

    for i in range(1, len(df)):

        cur_state = df["state"].iloc[i]

        if cur_state != prev_state or i == len(df) - 1:

            color = STATE_COLORS.get(prev_state, "#cccccc")

            ax.axvspan(
                start_t,
                t.iloc[i],
                alpha=0.08,
                color=color,
                linewidth=0
            )

            start_t = t.iloc[i]
            prev_state = cur_state

# =========================================================
# HELPER: FORMATO GENERAL
# =========================================================

def apply_common_format(ax):

    ax.grid(True, alpha=0.3)

    ax.tick_params(labelsize=10)

# =========================================================
# 1. TRAYECTORIA X-Y
# =========================================================

fig1, ax1 = plt.subplots(figsize=(8, 6))

sc = ax1.scatter(
    df["x"],
    df["y"],
    c=t,
    cmap="viridis",
    s=8,
    zorder=3
)

ax1.plot(
    df["x"].iloc[0],
    df["y"].iloc[0],
    "go",
    ms=10,
    label="Inicio",
    zorder=5
)

ax1.plot(
    df["x"].iloc[-1],
    df["y"].iloc[-1],
    "rs",
    ms=10,
    label="Fin",
    zorder=5
)

cb = plt.colorbar(sc, ax=ax1, pad=0.02)

cb.set_label("Tiempo (s)", fontsize=10)

ax1.set_xlabel("x (m)", fontsize=11)
ax1.set_ylabel("y (m)", fontsize=11)

ax1.set_title(
    "Trayectoria X–Y",
    fontsize=14,
    fontweight="bold"
)

ax1.legend(fontsize=9)

ax1.set_aspect("equal", adjustable="datalim")

apply_common_format(ax1)

fig1.savefig(
    "01_trajectory_xy.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 2. POSICIÓN vs TIEMPO
# =========================================================

fig2, ax2 = plt.subplots(figsize=(9, 5))

add_state_background(ax2)

ax2.plot(
    t,
    df["x"],
    label="x (m)",
    color="#1565C0",
    lw=1.8
)

ax2.plot(
    t,
    df["y"],
    label="y (m)",
    color="#AD1457",
    lw=1.8
)

ax2.set_xlabel("Tiempo (s)", fontsize=11)
ax2.set_ylabel("Posición (m)", fontsize=11)

ax2.set_title(
    "Posición vs Tiempo",
    fontsize=14,
    fontweight="bold"
)

ax2.legend(fontsize=9)

apply_common_format(ax2)
ax2.set_xlim(0, 200)

fig2.savefig(
    "02_position_vs_time.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 3. ORIENTACIÓN
# =========================================================

fig3, ax3 = plt.subplots(figsize=(9, 5))

add_state_background(ax3)

ax3.plot(
    t,
    np.degrees(df["theta"]),
    color="#6A1B9A",
    lw=1.8
)

ax3.set_xlabel("Tiempo (s)", fontsize=11)
ax3.set_ylabel("θ (°)", fontsize=11)

ax3.set_title(
    "Orientación vs Tiempo",
    fontsize=14,
    fontweight="bold"
)

apply_common_format(ax3)
ax3.set_xlim(0, 200)

fig3.savefig(
    "03_orientation_vs_time.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 4. COMANDOS DE VELOCIDAD
# =========================================================

fig4, ax4 = plt.subplots(figsize=(9, 5))

add_state_background(ax4)

ax4.plot(
    t,
    df["v_cmd"],
    label="v_cmd (m/s)",
    color="#00838F",
    lw=1.8
)

ax4.plot(
    t,
    df["w_cmd"],
    label="w_cmd (rad/s)",
    color="#E65100",
    lw=1.8
)

ax4.set_xlabel("Tiempo (s)", fontsize=11)
ax4.set_ylabel("Comando", fontsize=11)

ax4.set_title(
    "Comandos de Velocidad",
    fontsize=14,
    fontweight="bold"
)

ax4.legend(fontsize=9)

apply_common_format(ax4)
ax4.set_xlim(0, 200)

fig4.savefig(
    "04_velocity_commands.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 5. ERROR DE DISTANCIA
# =========================================================

fig5, ax5 = plt.subplots(figsize=(9, 5))

add_state_background(ax5)

ax5.plot(
    t,
    rho_error,
    color="#C62828",
    lw=1.8
)

ax5.axhline(
    0,
    color="gray",
    lw=1.0,
    ls="--"
)

ax5.set_xlabel("Tiempo (s)", fontsize=11)
ax5.set_ylabel(r"$e_{\rho}$ (m)", fontsize=11)

ax5.set_title(
    "Error de Distancia",
    fontsize=14,
    fontweight="bold"
)

apply_common_format(ax5)
ax5.set_xlim(0, 200)

fig5.savefig(
    "05_distance_error.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 6. ERROR ANGULAR
# =========================================================

fig6, ax6 = plt.subplots(figsize=(9, 5))

add_state_background(ax6)

ax6.plot(
    t,
    np.degrees(alpha_plot),
    color="#1565C0",
    lw=1.8
)

ax6.axhline(
    0,
    color="gray",
    lw=1.0,
    ls="--"
)

ax6.set_xlabel("Tiempo (s)", fontsize=11)
ax6.set_ylabel(r"$e_{\alpha}$ (°)", fontsize=11)

ax6.set_title(
    "Error Angular",
    fontsize=14,
    fontweight="bold"
)

apply_common_format(ax6)
ax6.set_xlim(0, 200)

fig6.savefig(
    "06_angular_error.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 7. COSTO MPC
# =========================================================

fig7, ax7 = plt.subplots(figsize=(9, 5))

add_state_background(ax7)

ax7.plot(
    t,
    cost_plot,
    color="#B71C1C",
    lw=1.8
)

ax7.set_xlabel("Tiempo (s)", fontsize=11)
ax7.set_ylabel("Costo", fontsize=11)

ax7.set_title(
    "Costo MPC vs Tiempo",
    fontsize=14,
    fontweight="bold"
)

apply_common_format(ax7)
ax7.set_xlim(0, 200)

fig7.savefig(
    "07_mpc_cost.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 8. ESTADO DEL ROBOT
# =========================================================

fig8, ax8 = plt.subplots(figsize=(10, 5))

state_ids = {
    s: i for i, s in enumerate(df["state"].unique())
}

state_num = df["state"].map(state_ids)

ax8.scatter(
    t,
    state_num,
    c=state_colors_mapped,
    s=8,
    zorder=3
)

ax8.set_yticks(list(state_ids.values()))

ax8.set_yticklabels(
    list(state_ids.keys()),
    fontsize=9
)

ax8.set_xlabel("Tiempo (s)", fontsize=11)

ax8.set_title(
    "Estado del Robot vs Tiempo",
    fontsize=14,
    fontweight="bold"
)

ax8.grid(True, alpha=0.3, axis="x")
ax8.set_xlim(0, 200)

fig8.legend(
    handles=legend_patches,
    loc="upper right",
    fontsize=8,
    framealpha=0.9
)

fig8.savefig(
    "08_robot_state.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# 9. DETECCIÓN DE OBJETIVO
# =========================================================

fig9, ax9 = plt.subplots(figsize=(9, 5))

add_state_background(ax9)

ax9.fill_between(
    t,
    df["target_detected"],
    step="post",
    alpha=0.6,
    color="#0097A7"
)

ax9.set_ylim(-0.05, 1.2)

ax9.set_xlabel("Tiempo (s)", fontsize=11)
ax9.set_ylabel("Detectado (0/1)", fontsize=11)

ax9.set_title(
    "Detección de Objetivo",
    fontsize=14,
    fontweight="bold"
)

apply_common_format(ax9)
ax9.set_xlim(0, 200)

fig9.savefig(
    "09_target_detection.png",
    dpi=300,
    bbox_inches="tight"
)

# =========================================================
# MOSTRAR TODAS
# =========================================================

plt.show()