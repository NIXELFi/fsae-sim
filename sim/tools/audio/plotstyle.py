"""Shared matplotlib styling for the audio tools (Agg backend, PNG output)."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Fixed categorical order: identity follows the entity, never its rank.
REAL = "#2a78d6"      # the recording
SIM = "#eb6834"       # the sim render
REF = "#1baf7a"       # a known reference (sidecar rpm, synthetic truth)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#d9d8d4"
SPEC_CMAP = "magma"

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb",
    "axes.facecolor": "#fcfcfb",
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK2,
    "axes.titlecolor": INK,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "lines.linewidth": 1.6,
    "font.size": 9,
    "axes.titlesize": 10,
    "legend.frameon": False,
    "savefig.dpi": 130,
    "savefig.bbox": "tight",
})


def spectrogram_ax(ax, f, t, P, fmin=40, fmax=8000, dyn_db=70, title=None, ref_db=None):
    """Log-frequency spectrogram in dB with a fixed dynamic range below the
    clip's own 99.5th percentile (or ref_db, to put two clips on one scale)."""
    import numpy as np
    m = (f >= fmin) & (f <= fmax)
    L = 10 * np.log10(P[m] + 1e-20)
    top = np.percentile(L, 99.5) if ref_db is None else ref_db
    im = ax.pcolormesh(t, f[m], L, shading="auto", cmap=SPEC_CMAP, vmin=top - dyn_db, vmax=top,
                       rasterized=True)
    ax.set_yscale("log")
    ax.set_ylim(fmin, fmax)
    ax.set_ylabel("Hz")
    ax.grid(False)
    if title:
        ax.set_title(title, loc="left")
    return im, top
