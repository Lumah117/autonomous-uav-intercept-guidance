# Guidance Mathematics
1. Coordinate frames & geometry
1.1 GPS → local Cartesian (NED)
Equation
You convert latitude/longitude differences into meters:
ΔN=R⋅(ϕ-ϕ_0)
ΔE=R⋅(λ-λ_0)cos⁡(ϕ_0)

Where in code
north = dlat * R
east  = dlon * R * cos(lat0)
Why this is used
	Guidance maths is much easier in Cartesian space
	For distances ≲ a few km, Earth curvature error is negligible
	This is the standard flat-Earth approximation used in UAV guidance
Design reasoning
	You don’t need full ECEF/WGS-84 transforms
	This is lightweight, deterministic, and numerically stable
________________________________________
1.2 Distance and bearing
Equations
Distance:
d=√(N^2+E^2 )

Bearing:
ψ="atan2"(E,N)

Where in code
distance = hypot(Nt, Et)
bearing  = atan2(Et, Nt)
Why
	Distance tells you which regime you’re in (intercept vs orbit)
	Bearing is the baseline pursuit direction
	atan2 avoids quadrant ambiguity
________________________________________
1.3 Line-of-sight (LOS) unit vector
Equation
r ̂=1/d [█(N@E)]

Where
uN = Nt / distance
uE = Et / distance
Why
	LOS unit vectors are fundamental to pursuit guidance
	Lets you project velocities along or across the target direction
________________________________________
2. Speed logic
2.1 Target-relative speed advantage
Equation
v_c="clamp"(k⋅v_t,v_min,v_max)

Where:
	k=1.3
Where
commanded_speed = clamp(target_speed * SPEED_MARGIN, ...)
Why
	You must be faster than the target to intercept
	Using a ratio instead of fixed speed adapts automatically
	Clamp ensures flight envelope safety
Design reasoning
	Prevents oscillation
	Avoids “chasing your own tail” at equal speeds
________________________________________
3. Intercept guidance (the interesting bit)
3.1 Pure pursuit (baseline)
Pure pursuit would be:
v ⃗_cmd∥r ̂

You don’t use this alone — and that’s a good thing.
________________________________________
3.2 Velocity decomposition (lead pursuit)
Step 1: Target velocity vector
v ⃗_t=[█(v_tN@v_tE )]

Step 2: Radial projection
v_(t,∥)=v ⃗_t⋅r ̂

Step 3: Perpendicular component
v ⃗_(t,⊥)=v ⃗_t-v_(t,∥) r ̂

Where
vt_dot_r = vtN*rN + vtE*rE
vt_perp  = vt - vt_dot_r*r
Why this matters
	Radial velocity just changes range
	Perpendicular velocity causes miss distance
	This is classic lead-pursuit logic
________________________________________
3.3 Lead-biased pursuit vector
Equation
d ⃗=r ̂+k_lead  v ⃗_(t,⊥)/v_c 

Where
leadN = rN + LEAD_GAIN * vt_perp_N / commanded_speed
leadE = rE + LEAD_GAIN * vt_perp_E / commanded_speed
Why this is elegant
	You are not predicting exact intercept point
	You gently bias toward where the target is moving
	Scaling by speed keeps it dimensionless and stable
Design reasoning
	Avoids aggressive PN guidance
	Very forgiving with noisy velocity estimates
	Smooth convergence
________________________________________
4. Orbit guidance
4.1 Tangential velocity for circular motion
Equation
For a circle, velocity must be perpendicular to radius:
v ⃗=[█(-ωrsin⁡θ@ωrcos⁡θ)]

Where
vN = -ORBIT_DIR * v * sin(angle)
vE =  ORBIT_DIR * v * cos(angle)
Why
	This directly commands tangential motion
	No radial component ⇒ stable orbit
	Extremely robust numerically
________________________________________
4.2 Centripetal acceleration limit
Physics
a_c=v^2/r⇒v_max=√(a_max r)

Where
max_orbit_speed = sqrt(MAX_CENTRIPETAL_ACC * ORBIT_RADIUS)
Why this is critical
	Prevents stall, excessive bank angle, or spiral dive
	Converts airframe limits into a speed constraint
	Makes the orbit physically flyable
________________________________________
5. Turn-rate limiting (human-visible smoothness)
Equation
Δψ_max=ψ ̇_max⋅Δt

Where
delta = clamp(desired - last, ±max_delta)
Why
	Prevents step changes in yaw
	Matches real fixed-wing dynamics
	Removes “robotic snapping” in demos
________________________________________
6. Closing speed & time-to-intercept
6.1 Relative velocity
Equation
v ⃗_rel=v ⃗_own-v ⃗_target

Projection onto LOS
v_close=v ⃗_rel⋅r ̂

Where
closing_speed = v_rel_N*uN + v_rel_E*uE
Why
	Only LOS component reduces range
	Lateral velocity doesn’t help interception
________________________________________
6.2 Time-to-intercept estimate
Equation
t=(d-r_orbit)/v_close 

Why this is used
	Gives operator intuition
	Lets you verify guidance health
	Not used for control (important!)
________________________________________
7. Overall guidance philosophy (the “why”)
You can summarise it like this:
“This is not a controller and not a predictor.
It’s a set of geometry-driven guidance laws that produces smooth, flyable velocity commands, constrained by physics, and supervised by a human-in-the-loop autopilot.”
Key properties
	No integrators → no wind-up
	No singularities
	No dependency on perfect target state
	Physically bounded at all times
________________________________________
8. If you want a one-sentence defence
If any colleague challenges my maths:
“Every equation either comes directly from vector geometry, circular motion physics, or classical pursuit guidance — and every constraint exists to keep the solution flyable, observable, and safe.”

