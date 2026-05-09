import numpy as np
import matplotlib.pyplot as plt

# Calibration data
reading = np.array([2000, 2450, 2650, 3250, 3500, 3950, 4050])
force   = np.array([55,   70,   75,   130,  150,  250,  300])

# Power-law fit: F = a * r^n
# To refit on new data with proper least squares in linear space:
#   from scipy.optimize import curve_fit
#   (a, n), _ = curve_fit(lambda r, a, n: a*r**n, reading, force, p0=[1e-9, 3])
a, n = 1.075e-9, 3.164

r_fit = np.linspace(1800, 4200, 400)
F_fit = a * r_fit**n

residuals = force - a * reading**n
rmse    = np.sqrt(np.mean(residuals**2))
max_err = np.max(np.abs(residuals))

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 16,
    'axes.titlesize': 22,
    'axes.titleweight': 'bold',
    'axes.labelsize': 18,
    'axes.labelweight': 'medium',
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'legend.fontsize': 15,
})

fig, ax = plt.subplots(figsize=(12, 7))
ax.plot(r_fit, F_fit, color='#D85A30', linewidth=6,
        label=f'Fit:  F = {a:.2e} · r$^{{{n:.2f}}}$', zorder=2)
ax.scatter(reading, force, color='#378ADD', edgecolor='#0F3D75',
           linewidth=3, s=320, zorder=3, label='Measurements')

ax.set_xlabel('Analog reading (0 – 4095)', labelpad=12, fontweight='bold')
ax.set_ylabel('Applied force (g)', labelpad=12, fontweight='bold')
ax.set_xlim(1800, 4200)
ax.set_ylim(0, 350)
ax.grid(alpha=0.4, linestyle='--', linewidth=1.4)
ax.set_axisbelow(True)
ax.legend(loc='upper left', frameon=False, handlelength=2.5,
          borderpad=0.6, labelspacing=0.9)
ax.spines[['top', 'right']].set_visible(False)
ax.spines[['left', 'bottom']].set_linewidth(2.6)
ax.tick_params(width=2.2, length=8)

stats = f'RMSE      {rmse:.1f} g\nMax err   {max_err:.1f} g\nn = {len(reading)}'
ax.text(0.97, 0.05, stats, transform=ax.transAxes, fontsize=14,
        ha='right', va='bottom', family='monospace',
        bbox=dict(boxstyle='round,pad=0.8', facecolor='white',
                  edgecolor='#888888', linewidth=2.2))

plt.tight_layout()
plt.savefig('fsr_calibration.png', dpi=220, bbox_inches='tight',
            facecolor='white')
plt.show()
