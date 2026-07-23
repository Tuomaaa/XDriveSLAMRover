# Week 6 — particle impoverishment in the weakly-observed dimension

Two replays of the **same** 60-second synthetic run, through the **same**
filter code, differing only in `SIGMA_ROT_FLOOR` in `ros2_ws/src/mcl.py`.

| | `heading-collapse-before.gif` | `heading-collapse-after.gif` |
|---|---|---|
| `SIGMA_ROT_FLOOR` | 0.002 rad (old) | **0.010 rad (current)** |
| Heading spread at 7s | 0.18 deg — collapsed | 0.52 deg — holding |
| Heading error, final | **+33.6 deg** | +1.2 deg |
| Position RMS | 10.2 cm | **0.8 cm** |
| vs. dead reckoning | 1.6x better | **20.2x better** |

Reproduce either one:

```bash
python3 mcl_wander_sim.py --gif after.gif
python3 mcl_wander_sim.py --gif before.gif --rot-floor 0.002
python3 mcl_wander_sim.py --sweep
```

## Reading the panels

Three panels, because no one of them shows the whole story.

**Left — overview.** The dotted grey line is the true path; red is the MCL
estimate, green is raw dead reckoning. In **after**, red sits on the dotted
line and traces it repeatably while green spirals away as 55 degrees of
accumulated yaw drift compounds. In **before**, red drifts *alongside*
green.

That is the tell, and it is why the failure case is worth keeping: it does
not look like a filter going haywire. It looks like a filter quietly giving
up and echoing the odometry it was supposed to correct. The estimate stays
smooth, stays inside the box, and stays plausible — while being 27 cm and
34 degrees wrong at the end.

**Top right — particle cloud**, a ±6 cm window centred on the *true* pose.
At box scale a converged cloud is about 1% of the frame and is simply
invisible, which is why it gets its own panel. In **after** the cloud stays
wrapped around the black cross with the red estimate inside it. In
**before** the panel goes blank and annotates how far away the cloud has
gone — by the end it is nowhere near the truth.

**Bottom right — position error vs. time.** The most direct answer to "is
it working". Red flat near zero; green climbing. In the failure case red
tracks green instead of staying flat, which is the same story the overview
tells, quantified.

Note that all three panels need ground truth, which a real recording does
not have. `mcl_offline.py` renders the single-panel view you get on real
data; this richer view lives in `mcl_wander_sim.py` because only the
synthetic run knows the answer.

## Why it happened

Heading is the weakly-observed dimension in this setup. Both ultrasonic
beams reference walls through the same origin corner, so at any *single*
pose the family `(x·cos θ, y·cos θ, θ)` predicts bit-identical ranges — the
measurement is rank-2 over a 3-DOF state, and heading lies in its null
direction.

Over a *trajectory* that degeneracy breaks: a wrong heading rotates the
body-frame odometry delta the wrong way, the particle travels in the wrong
direction, and the next measurement contradicts it. Heading is recoverable.

But recovering it requires surviving particles that hold the alternative
heading — and because heading is the weakly-observed dimension, resampling
every cycle strips diversity there **first**. At the 0.002 floor the cloud
committed to a heading within about 7 seconds. After that there was nothing
left to select from, and the error grew without bound.

Raising the one constant restores enough heading diversity for the
measurement update to keep choosing. Behaviour is flat from 0.005 to 0.020,
so 0.010 sits mid-plateau rather than on an edge.

## The testing lesson

The `mcl.py` parked-cloud test passed at **both** floors, and it was not a
weak test — it was aimed at the wrong regime. A stationary rover's heading
barely affects its predicted ranges, so nothing prunes the heading spread
while parked and it looks healthy at any floor. Only sustained *motion*
exposes the collapse.

`run_heading_tracking_test` in `ros2_ws/src/mcl_test.py` now covers this: an
arc with continuous rotation, which fails at 19.9 degrees of heading error
under the old floor.

The broader point for this project: the bug survived a full task-by-task
review and a whole-branch review, and was only caught by running a
realistic-length scenario with known ground truth. Short unit scenarios
verify that the mechanics are right; they do not reveal what a filter does
after a minute of compounding error.
