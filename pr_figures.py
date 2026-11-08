import glob, json, math, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch
from pymavlink import mavutil
RUNS, OUT = sys.argv[1], sys.argv[2]
MASTER, PR = '#c0392b', '#1f6fb2'
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})

# figure: geometry of the drift and the factor
fig, (ax, bx) = plt.subplots(1, 2, figsize=(11, 4.6), gridspec_kw={'width_ratios': [1, 1.15]})
ax.set_aspect('equal'); ax.axis('off')
theta = math.radians(70); dalpha = 30
k = 1 - math.cos(theta)
def compass(deg):  # 0 = north (up), clockwise positive
    r = math.radians(deg); return math.sin(r), math.cos(r)
ax.add_patch(plt.Circle((0, 0), 1, fill=False, ls=':', color='0.6'))
ax.text(0, 1.08, 'N', ha='center', color='0.4')
for deg, alpha, label in (90, 0.35, 'lean direction\nbefore'), (90 + dalpha, 1.0, 'lean direction\nafter'):
    x, y = compass(deg)
    ax.add_patch(FancyArrowPatch((0, 0), (x, y), arrowstyle='-|>', mutation_scale=16, color=PR, alpha=alpha, lw=2))
    ax.text(1.12 * x, 1.12 * y, label, color=PR, alpha=max(alpha, 0.6), ha='left', va='center', fontsize=10)
ax.add_patch(Arc((0, 0), 1.0, 1.0, theta1=90 - 90 - dalpha, theta2=90 - 90, color=PR, lw=1.5))
x, y = compass(90 + dalpha / 2); ax.text(0.62 * x, 0.62 * y, 'Δα = 30°', color=PR, ha='left', va='center')
for deg, alpha, label, ha in (0, 0.35, 'heading before', 'right'), (k * dalpha, 1.0, 'heading after', 'left'):
    x, y = compass(deg)
    ax.add_patch(FancyArrowPatch((0, 0), (0.8 * x, 0.8 * y), arrowstyle='-|>', mutation_scale=16, color=MASTER, alpha=alpha, lw=2))
    ax.text(0.8 * x + (-0.06 if ha == 'right' else 0.06), 0.8 * y - 0.02, label, color=MASTER, alpha=max(alpha, 0.6), ha=ha, va='center', fontsize=10)
ax.add_patch(Arc((0, 0), 0.9, 0.9, theta1=90 - k * dalpha, theta2=90, color=MASTER, lw=1.5))
ax.text(0.2, 0.25, 'Δψ = %.0f°' % (k * dalpha), color=MASTER, ha='left', fontsize=10)
ax.set_xlim(-1.25, 1.75); ax.set_ylim(-1.0, 1.25)
ax.set_title('Top view at θ = 70° lean, no yaw rate: heading is level yaw ψ', fontsize=11)
lean = np.linspace(0, 90, 181)
bx.plot(lean, 1 - np.cos(np.radians(lean)), color=MASTER, lw=2)
for d in 30, 45, 70:
    v = 1 - math.cos(math.radians(d)); bx.plot(d, v, 'o', color=MASTER)
    bx.annotate('%d°: %.2f' % (d, v), (d, v), textcoords='offset points', xytext=(-62, 4))
bx.set_xlabel('lean θ (degrees)'); bx.set_ylabel('Δψ / Δα')
bx.set_title('Heading turned per degree the lean direction turns: 1 − cos θ', fontsize=11)
bx.set_xlim(0, 90); bx.set_ylim(0, 1); bx.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(OUT + '/level-yaw-drift-geometry.png', dpi=130); plt.close(fig)

# figure: master vs this PR, roll stick held at 70 degrees
def wrap(a): return (a + 180) % 360 - 180
series = {'master': [], 'pr': []}
for js in sorted(glob.glob(RUNS + '/freestyle-*.json')):
    build = 'master' if '-master-' in js.split('/')[-1] else 'pr'
    mk = dict(json.load(open(js))['windows']['roll_hold']['marks'])
    t0, t1 = mk['roll'], mk['release']
    log = mavutil.mavlink_connection(js[:-5] + '.BIN')
    vel, yaw = [], []
    while True:
        m = log.recv_match(type=['XKF1', 'ATT'])
        if m is None: break
        t = m.TimeUS * 1e-6
        if not t0 - 1 <= t <= t1 + 1: continue
        if m.get_type() == 'XKF1':
            if m.C == 0: vel.append((t, m.VN, m.VE))
        else: yaw.append((t, m.Yaw))
    vel, yaw = np.array(vel), np.array(yaw)
    grid = np.arange(0, t1 - t0, 0.1)
    nose = np.interp(t0 + grid, yaw[:, 0], np.unwrap(np.radians(yaw[:, 1])))
    track = np.unwrap(np.arctan2(vel[:, 2], vel[:, 1]))
    track = np.interp(t0 + grid, vel[:, 0], track)
    stick = nose[0] + math.copysign(math.pi / 2, wrap(math.degrees(np.interp(t0 + 2, vel[:, 0], np.unwrap(np.arctan2(vel[:, 2], vel[:, 1]))) - nose[0])))
    speed = np.interp(t0 + grid, vel[:, 0], np.hypot(vel[:, 1], vel[:, 2]))
    terr = np.degrees(track - stick); terr[speed < 3] = np.nan
    series[build].append((grid, terr, np.degrees(nose - nose[0])))
fig, axs = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
for build, colour, label in ('master', MASTER, 'master (LOIT_OPTIONS 0)'), ('pr', PR, 'this PR (LOIT_OPTIONS bit 1)'):
    runs = series[build]; g = runs[0][0]; n = min(len(r[0]) for r in runs)
    for ax, idx in (axs[0], 1), (axs[1], 2):
        stack = np.array([r[idx][:n] for r in runs])
        ax.plot(g[:n], np.nanmedian(stack, 0), color=colour, lw=2, label=label + ', median of %d' % len(runs))
        ax.fill_between(g[:n], np.nanmin(stack, 0), np.nanmax(stack, 0), color=colour, alpha=0.2, lw=0)
axs[0].set_ylabel('ground track minus\nstick direction (°)'); axs[1].set_ylabel('nose turned (°)')
axs[1].set_xlabel('seconds since full roll stick, yaw stick centred')
axs[0].set_title('Freestyle SITL, LOIT_ANG_MAX 70, ATC_RATE_FF_ENAB 1: roll stick held 15 s (band: min to max of 3 runs)', fontsize=11)
for ax in axs: ax.grid(alpha=0.3); ax.axhline(0, color='0.5', lw=0.8)
axs[0].legend(loc='upper right', frameon=False)
fig.tight_layout(); fig.savefig(OUT + '/loiter-roll-stick-master-vs-pr.png', dpi=130); plt.close(fig)
for b, runs in series.items():
    print(b, 'track worst %.1f, nose worst %.1f' % (max(np.nanmax(np.abs(r[1])) for r in runs), max(np.nanmax(np.abs(r[2])) for r in runs)))
