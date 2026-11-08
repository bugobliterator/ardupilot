"""PR 35061 animation: a copter at 70 deg lean whose lean direction turns 90 deg with no body yaw rate.
Its level yaw (heading) follows by (1 - cos theta) of the turn. Usage: pr_animation.py <out.gif>"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

THETA = np.radians(70); TURN = np.radians(90); FRAMES = 72; HOLD = 18
LEAN, HEAD = '#1f6fb2', '#c0392b'

def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])
def conj(q): return q * np.r_[1, -1, -1, -1]
def rot(q, v): return qmul(qmul(q, np.r_[0, v]), conj(q))[1:]
def level_yaw(q): return 2 * np.arctan2(q[3], q[0])
def thrust_ned(q): return rot(q, np.array([0, 0, -1.0]))  # thrust is along body -z
def azimuth(v): return np.arctan2(v[1], v[0])  # NED: clockwise from north

# start: heading north, leaning right (east) by THETA; integrate body rates with no body-z part
q = np.array([np.cos(THETA/2), np.sin(THETA/2), 0, 0])  # roll right about body x
states, dt, rate = [], 2e-4, 1.0
alpha0, psi0 = azimuth(thrust_ned(q)), level_yaw(q)
turned = 0.0; per_frame = TURN / FRAMES; next_mark = 0.0
while turned <= TURN + 1e-9:
    if turned >= next_mark - 1e-12:
        states.append(q.copy()); next_mark += per_frame
    z = thrust_ned(q)
    w_e = np.cross(z, np.cross(np.array([0, 0, rate]), z))  # NED +z (down): turns the thrust axis clockwise seen from above
    w_b = rot(conj(q), w_e); w_b[2] = 0.0
    q = q + 0.5 * qmul(q, np.r_[0, w_b]) * dt; q /= np.linalg.norm(q)
    turned = np.unwrap([alpha0, azimuth(thrust_ned(q))])[1] - alpha0
states += [states[-1]] * HOLD

def to_plot(v): return np.array([v[1], v[0], -v[2]])  # NED -> east, north, up
ARMS = [np.radians(a) for a in (45, -45, 135, -135)]
fig = plt.figure(figsize=(10, 4.8))
ax3 = fig.add_subplot(1, 2, 1, projection='3d'); axt = fig.add_subplot(1, 2, 2)
def draw(i):
    q = states[i]
    ax3.cla(); axt.cla()
    ax3.set_xlim(-1, 1); ax3.set_ylim(-1, 1); ax3.set_zlim(-0.6, 1.0); ax3.set_box_aspect((1, 1, 0.8))
    ax3.view_init(elev=28, azim=-60); ax3.set_xticks([]); ax3.set_yticks([]); ax3.set_zticks([])
    ax3.set_xlabel('east'); ax3.set_ylabel('north')
    for pane in ax3.xaxis.pane, ax3.yaxis.pane, ax3.zaxis.pane: pane.set_visible(False)
    ax3.grid(False)
    ring = np.linspace(0, 2*np.pi, 60)
    ax3.plot(np.cos(ring)*0.9, np.sin(ring)*0.9, -0.6, color='0.8', lw=0.8)
    for a in ARMS:
        tip = to_plot(rot(q, 0.6 * np.array([np.cos(a), np.sin(a), 0])))
        ax3.plot([0, tip[0]], [0, tip[1]], [0, tip[2]], color='0.25', lw=3)
        ax3.scatter(*tip, s=60, color='0.25')
    nose = to_plot(rot(q, np.array([0.75, 0, 0])))
    ax3.plot([0, nose[0]], [0, nose[1]], [0, nose[2]], color=HEAD, lw=3)
    thr = to_plot(thrust_ned(q))
    ax3.quiver(0, 0, 0, *thr, color=LEAN, lw=2, arrow_length_ratio=0.15)
    ax3.plot([0, thr[0]], [0, thr[1]], [-0.6, -0.6], color=LEAN, lw=1.5, ls='--')  # lean direction shadow
    alpha = np.unwrap([alpha0, azimuth(thrust_ned(q))])[1]; psi = np.unwrap([psi0, level_yaw(q)])[1]
    # theta: arc from vertical to the thrust axis
    up = np.array([0, 0, 1.0]); s = np.linspace(0, 1, 20)
    arc = np.array([(1 - t) * up + t * thr for t in s]); arc = 0.45 * arc / np.linalg.norm(arc, axis=1)[:, None]
    ax3.plot([0, 0], [0, 0], [0, 0.95], color='0.5', lw=1, ls=':')
    ax3.plot(arc[:, 0], arc[:, 1], arc[:, 2], color='k', lw=1.5)
    mid = arc[len(arc) // 2] * 1.25; ax3.text(*mid, 'θ', fontsize=14, color='k')
    # alpha and psi: bearings from north on the ground ring
    g = -0.6
    ax3.plot([0, 0], [0, 0.95], [g, g], color='0.5', lw=1); ax3.text(0, 1.0, g, 'N', color='0.4', fontsize=9)
    for end, radius, colour, label in (alpha, 0.55, LEAN, 'α'), (psi, 0.32, HEAD, 'ψ'):
        b = np.linspace(0, end, 30)
        ax3.plot(radius * np.sin(b), radius * np.cos(b), g, color=colour, lw=1.8)
        ax3.text(1.2 * radius * np.sin(end / 2), 1.2 * radius * np.cos(end / 2), g, label, color=colour, fontsize=14)
    ax3.plot([0, 0.9 * np.sin(psi)], [0, 0.9 * np.cos(psi)], [g, g], color=HEAD, lw=1.5, ls='--')  # heading on the ground
    ax3.set_title('Copter leaning θ = 70°: nose red, thrust blue', fontsize=11)
    # top view
    d_alpha, d_psi = np.degrees(alpha - alpha0), np.degrees(psi - psi0)
    axt.set_aspect('equal'); axt.axis('off'); axt.set_xlim(-1.3, 2.1); axt.set_ylim(-1.35, 1.25)
    axt.plot(np.cos(ring), np.sin(ring), ':', color='0.6'); axt.text(0, 1.08, 'N', ha='center', color='0.4')
    for ang, colour, label in (alpha, LEAN, 'lean direction α'), (psi, HEAD, 'heading ψ (level yaw)'):
        e, n = np.sin(ang), np.cos(ang)
        axt.annotate('', xy=(0.9*e, 0.9*n), xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color=colour, lw=2.5))
        axt.text(1.0*e, 1.0*n, label, color=colour, ha='left' if e > 0.1 else 'center', va='bottom', fontsize=10)
    axt.text(-1.3, -1.12, f'Δα = {d_alpha:5.1f}°    Δψ = {d_psi:5.1f}°', fontsize=11, family='monospace')
    axt.text(-1.3, -1.32, f'(1 − cos 70°) · Δα = {0.658*d_alpha:5.1f}°', fontsize=11, family='monospace', color=HEAD)
    axt.set_title('Top view: no yaw stick, no body yaw rate', fontsize=11)
anim = FuncAnimation(fig, draw, frames=len(states))
fig.tight_layout()
anim.save(sys.argv[1], writer=PillowWriter(fps=18), dpi=80)
