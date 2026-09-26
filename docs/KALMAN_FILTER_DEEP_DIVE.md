# Kalman Filter Deep Dive

This document focuses on the mathematics implemented in `kalman_filter.py` and connects each line to standard estimation theory.

## 1. State definition

Kalman Katcher uses

[
mathbf{x}_k =
egin{bmatrix}
x_k & y_k & v_{x,k} & v_{y,k} & a_{y,k}
end{bmatrix}^T
]

The camera directly measures only `x` and `y`.

The model assumes:
- horizontal velocity is approximately constant between impacts,
- vertical acceleration is approximately constant,
- external impacts are handled separately as discrete bounce rules.

## 2. Discrete-time process model

For sampling period (Delta t):

[
x_{k+1} = x_k + v_{x,k}Delta t
]

[
y_{k+1} = y_k + v_{y,k}Delta t + rac{1}{2}a_{y,k}Delta t^2
]

[
v_{x,k+1}=v_{x,k}
]

[
v_{y,k+1}=v_{y,k}+a_{y,k}Delta t
]

[
a_{y,k+1}=a_{y,k}
]

This becomes:

[
mathbf{x}_{k+1}=Fmathbf{x}_k + mathbf{u}
]

with (mathbf{u}=0).

The repository uses 40 FPS, therefore:

[
Delta t = 0.025s
]

## 3. Observation model

The detector returns a measured center:

[
mathbf{z}_k =
egin{bmatrix}
x_m \\
y_m
end{bmatrix}
]

and

[
mathbf{z}_k = Hmathbf{x}_k + mathbf{v}_k
]

where

[
H =
egin{bmatrix}
1&0&0&0&0\\
0&1&0&0&0
end{bmatrix}
]

and measurement noise (mathbf{v}_k) is modeled with covariance `R`.

## 4. Probabilistic interpretation

A linear Kalman filter assumes a linear-Gaussian model:

[
x_k sim mathcal{N}(mu_k,P_k)
]

The state is not represented as one unquestionable point. It is represented as a Gaussian belief.

- (mu_k): best state estimate
- (P_k): uncertainty around that estimate

A new camera observation supplies another probabilistic constraint. The filter computes the posterior belief obtained by combining prior/model information with measurement information.

## 5. Innovation

The code computes:

```python
self.y = Z.T - (self.H @ x)
```

Mathematically:

[
	ilde{y}_k = z_k - Hx_k
]

If the innovation is near zero, the camera approximately agrees with the state belief.

A large innovation means either:
- the estimate is wrong,
- the measurement is noisy,
- the target association is wrong,
- or the dynamics changed unexpectedly.

In production systems, innovation statistics are often monitored for fault detection and data-association gating.

## 6. Innovation covariance

```python
S = H @ P @ H.T + R
```

[
S_k = HP_kH^T + R
]

This answers:

> How uncertain should I expect the measurement residual to be?

Both state uncertainty and sensor uncertainty contribute.

## 7. Kalman gain

```python
K = P @ H.T @ inv(S)
```

[
K_k=P_kH^TS_k^{-1}
]

The gain is not an arbitrary tuning coefficient. It is derived from the uncertainty model.

If measurement uncertainty becomes large relative to prior uncertainty, the gain generally shrinks.

If the prior is uncertain while measurement is trusted, measurement correction becomes stronger.

## 8. State correction

[
x_k^+ = x_k^- + K_k(z_k-Hx_k^-)
]

This is:

```python
x = x + K @ y
```

The correction is proportional to both:
- disagreement with measurement,
- how trustworthy that disagreement is.

## 9. Covariance correction

[
P_k^+ = (I-KH)P_k^-
]

A successful measurement usually reduces uncertainty in observed state dimensions and, through covariance coupling, may also reduce uncertainty in hidden variables.

A numerically more robust production implementation sometimes uses the Joseph form:

[
P^+=(I-KH)P^-(I-KH)^T + KRK^T
]

## 10. Time prediction

[
x_{k+1}^- = Fx_k^+ + u
]

[
P_{k+1}^- = FP_k^+F^T + Q
]

The covariance expands because even a good current estimate becomes less certain as it is propagated through an imperfect model.

This is exactly what `Q` expresses.

## 11. Q versus R intuition

A useful mental experiment:

### Increase R
You are saying “the detector is less reliable.”

Expected behavior:
- state changes less in response to individual detections,
- estimate becomes smoother,
- estimator may lag real motion changes.

### Increase Q
You are saying “the motion model is less reliable.”

Expected behavior:
- state is more willing to deviate from the dynamics model,
- measurements influence the estimate more over time,
- estimate can adapt faster but may become noisier.

Tuning a Kalman filter is largely the art of making uncertainty assumptions match reality.

## 12. Initial covariance

The initial state uses guessed:
- `vx = 0`
- `vy = -800`
- `ay = 5000`

Those are priors, not measurements.

The covariance communicates how strongly those priors should be trusted.

Velocity uncertainty is initialized much larger than position uncertainty. That is logically consistent: the first camera observation gives a location but cannot, by itself, reveal reliable velocity.

## 13. Measurement-update-first ordering

Many textbooks present each cycle as:

```text
predict → measure → update
```

This repository's `kf()` function performs:

```text
measurement update → one-step prediction
```

Those are not fundamentally contradictory; they differ in which time index is considered the function input and output.

Here, the input state can be interpreted as a prior belief for the current observation. The function incorporates the observation and then returns the next predicted belief.

## 14. Bounce dynamics are outside the linear Kalman model

The Kalman transition matrix itself does not encode impacts.

The future-trajectory simulation explicitly checks geometric boundaries and flips velocity signs with damping.

That makes the overall model **piecewise/hybrid**.

Between impacts:
- linear dynamics

At impacts:
- discrete nonlinear event rule

This is common in physical systems.

## 15. Why not an Extended Kalman Filter?

An EKF is useful when transition or measurement equations are nonlinear and need local linearization.

This project's core transition and observation equations are already linear.

The nonlinearity is mostly handled outside the estimator:
- Hough perception,
- association,
- bounce event logic,
- line intersection,
- actuator lookup.

So a standard linear Kalman filter is a sensible estimator for the chosen state model.

## 16. Why not a particle filter?

Particle filters are useful for:
- strongly nonlinear systems,
- non-Gaussian uncertainty,
- multimodal beliefs.

Kalman Katcher's state is low-dimensional and approximately continuous with a single dominant hypothesis per track. A particle filter would add computational complexity without an obvious need.

## 17. Covariance-aware association improvement

The tracker currently associates by Euclidean distance.

A more estimator-aware method would use the predicted measurement covariance:

[
S=HPH^T+R
]

and Mahalanobis distance:

[
d^2=(z-Hx)^TS^{-1}(z-Hx)
]

This naturally accounts for direction-dependent uncertainty.

A detection outside a statistical gate can be rejected rather than forced onto a track.

## 18. Numerical considerations

For a small 2×2 innovation matrix, direct inversion works, but production numerical code commonly prefers solving a linear system rather than explicitly computing an inverse.

Other production considerations include:
- enforcing symmetry of `P`,
- checking positive semi-definiteness,
- Joseph covariance update,
- timestamp-based `dt`,
- unit normalization,
- innovation consistency checks.

## 19. Interview summary

Be able to explain this in 30 seconds:

> The camera gives noisy x-y measurements. I represent each ball with a five-dimensional kinematic state containing position, velocity, and vertical acceleration. A linear transition model predicts how that state evolves, while the observation matrix maps it back to image position. The Kalman filter uses the innovation and covariance-derived Kalman gain to fuse the current camera observation with the prior state belief, updates state uncertainty, then propagates the corrected state forward. I use the resulting state to simulate the ball trajectory and predict where it will cross the catcher line.
