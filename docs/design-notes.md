# Design notes for Av Scenario Runner

## Method selection

### The scenario format

Scenarios are concrete scenarios in the sense of Menzel, Bagschik and Maurer (2018): every
parameter is fixed, nothing is sampled, and running the same document twice gives the same answer.
The structure follows ASAM OpenSCENARIO XML 1.2.0: a road, an ego entity with an initial state,
actor entities carrying either a scripted manoeuvre or a reactive behaviour, a stop trigger, and
the checks the run must satisfy.

The serialisation is TOML rather than XML. The reason is not aesthetic. `tomllib` is in the
standard library, so scenario parsing adds no dependency, and TOML has exactly one obvious way to
express a table, an array of tables, and a typed scalar, which removes an entire class of schema
question before it is asked. The cost is that TOML has no schema language, so all validation is
hand written. That cost is paid deliberately, because the validation this repository needs is not
type checking but domain checking: a lane index inside the road, a schedule whose times increase,
an assertion whose bounds are ordered, a controller name the pipeline actually implements.

Three validation decisions are worth stating explicitly.

Unknown keys are errors, not warnings. A document containing `lane_with = 3.5` instead of
`lane_width = 3.5` is a document that does not test what its author believes it tests. Accepting it
and using the default produces a green result that means nothing, which is the precise failure this
repository exists to prevent. The permitted key set is listed in every rejection message.

Errors carry a location. `tomllib` discards position information once a document has parsed, so an
error found during domain validation has no line number of its own. `model/locate.py` recovers one
by scanning the source for the table header and key that the dotted field path names, and walks up
the path to the nearest locatable ancestor when the exact field is inside an inline table. The
result is `speed_limit_compliance.toml:9: ego.controller: unknown controller 'mpc_lateral'`, which
an editor can jump to. A best effort locator that is sometimes one level coarse is much better than
no location at all.

The format is versioned, and versioning is exercised rather than declared. Version 1.0 is the
original shape; 1.1 adds the optional `[perception]` table. A 1.0 document loads unchanged and
receives the default noise free perception model, and `scenarios/speed_limit_compliance.toml` is
kept at 1.0 in the shipped suite so that this path is exercised on every run rather than only in a
test. A 1.0 document that uses a 1.1 field is rejected by name and told which version introduced
it. A version outside the supported set is rejected with the set listed.

### The assertion vocabulary

Eight assertion kinds are implemented: `no_collision`, `min_time_to_collision`,
`min_time_headway`, `longitudinal_acceleration`, `lateral_acceleration`, `speed_limit`,
`goal_reached`, and `min_distance`. Each returns the worst value it observed, the time at which it
occurred, the bound it was tested against, and a one line detail. This is not decoration. The
difference between a scenario that passes with four metres of clearance and one that passes with
two centimetres does not appear in a verdict, and it is exactly the difference that predicts
whether a small change to the controller will break the scenario next week. Every report in this
repository prints the worst value for passing assertions as well as failing ones for that reason.

The comparison direction is fixed and uniform: lower bounds are inclusive, upper bounds are
inclusive, and `no_collision` is the single strict comparison, because zero clearance is contact
rather than a near miss. Every one of these boundaries is tested at equality in
`tests/test_algorithm.py`, because a comparison operator that is off by one direction is invisible
anywhere except at the boundary.

Time to collision follows Hayward (1972): the time until contact under constant velocity. The
implementation generalises it to two dimensions by solving, for each pair of covering discs, the
smallest non-negative root of `|p + v t| = R`, and taking the minimum over pairs and over actors.
The two dimensional form matters. A purely longitudinal time to collision, computed from a gap and
a closing speed within a lane, cannot score a cut in until the cutting vehicle is already in the
lane, which is after the moment that made the manoeuvre dangerous.

Time headway is the same root solved with the other vehicle held stationary, so it is the time the
ego needs to reach the space that vehicle occupies now. Both are in the vocabulary because Vogel
(2003) compares them and finds that they identify different situations, and the two failure modes
are easy to state. Time to collision is a function of the relative motion and goes to infinity the
moment the closing speed does, so a vehicle followed two metres behind at a matched speed scores
infinity; headway divides the gap by the ego's own speed and scores it anyway. The converse holds
as well: a vehicle approached from two hundred metres back has a comfortable headway and a short
time to collision. Neither ordering can be recovered from the other, so a scenario that cares about
following distance declares `min_time_headway` and a scenario that cares about closing declares
`min_time_to_collision`.

The corridor that headway searches is the disc cover corridor rather than a lane. Its half width is
the sum of the two disc radii, 2.442 m for two default footprints, so a vehicle in the neighbouring
lane of a 3.5 m road is not the ego's leader and a cutting vehicle becomes one partway through the
manoeuvre rather than at the lane line. That is the same conservatism, and the same lateness, that
clearance already carries, which is the point: the two metrics should differ about the road, not
about the geometry. A vehicle behind the ego is never reached by an ego holding its heading, and
neither is any vehicle while the ego is stopped, so both report infinity. The second of those is
the conventional reading of a quantity that divides by speed, and it is why a scenario that ends
with the ego stationary behind an obstacle needs `min_distance` rather than `min_time_headway`.

Footprints are covered by three equal discs along the vehicle axis, following Ziegler and Stiller
(2010). Clearance is then the minimum disc pair distance less the two radii, which is zero at
contact and negative on overlap, and time to collision is the same query solved forward in time.
The approximation is conservative: for the default 4.6 m by 1.9 m footprint the disc cover extends
0.454 m beyond each bumper and 0.271 m beyond each side, so a reported clearance of 1.090 m is a
true bumper to bumper gap of about 2.0 m. Being conservative in this direction is the right sign
for a safety check, and the constant is documented so that a threshold can be set knowingly. This
is also why the browser playback draws the true rectangles rather than the discs: the picture
should show the physical situation, and the assertion should use the conservative bound.

### The simulator

The ego is a kinematic bicycle with the reference point at the rear axle, integrated with a fixed
step classical Runge-Kutta scheme. Kong, Pfeiffer, Schildbach and Borrelli (2015) show this model
tracks a dynamic model closely at the speeds and lateral accelerations these scenarios reach, and
it stays well conditioned at standstill, which a tyre model does not. The step is fixed rather than
adaptive because an adaptive step makes the number of samples a function of the trajectory, and a
regression baseline over a variable length trace is far harder to reason about.

Actors move in road coordinates rather than being steered. A scripted actor's trajectory is a
closed form function of time: a piecewise constant acceleration schedule integrated exactly,
including the instant a braking actor reaches standstill, and a smoothstep lateral profile for a
lane change, which starts and ends with zero lateral rate. This means the provocation a scenario
applies is fixed by the document and cannot be perturbed by the controller it is meant to provoke.
A reactive actor uses the Intelligent Driver Model against the nearest vehicle ahead in its lane,
including the ego, which is what `scenarios/follower_tailgate.toml` uses to create a rear conflict.

Two reference controllers are provided. `idm_lane_keep` combines IDM car following with pure
pursuit lane keeping and is what a scenario is normally written against. `constant_speed` holds its
desired speed and never reacts to anything ahead. The second exists so that the shipped suite
contains a scenario that genuinely fails, with a real collision at a real time, rather than a suite
that only ever prints green and is therefore untested as a detector.

Determinism is a property of the design, not an accident. The loop reads no clock, consults no
environment, and runs no iterative solver. With the default noise free perception model the seed
has no effect at all and two runs agree bit for bit. When `[perception]` declares a non-zero
standard deviation the seed becomes load bearing, and `tests/test_pipeline.py` checks all three
facts: identical across seeds without noise, identical for a repeated seed with noise, and
different across seeds with noise. The last of these is what stops the determinism test from being
vacuous.

## What a regression baseline may pin

This is the design question the repository exists to answer, so it is set out at length.

A regression test earns its place by failing when behaviour changes and passing otherwise, on any
machine that runs it. A baseline that records a raw floating point number taken from late in a
simulated run does not have that property, and the failure mode is well known: a value taken from
an iterative solve that did not fully converge is not reproducible at all, and differences in
reduction order inside a linear algebra library grow, over many steps, into answers that differ in
the third significant figure. A test that fails for reasons unrelated to the code under test is
worse than no test, because people learn to rerun it until it passes and then stop reading it.

This repository is less exposed than a solver based one. There is no optimisation, no iterative
convergence, and no matrix factorisation anywhere in the simulation path; the per step arithmetic
is a handful of scalar operations. It is still not safe to pin a late trajectory value, for three
separate reasons.

First, transcendental functions are not bit reproducible across platforms. `sin`, `cos`, `tan`, and
`atan2` are not required to be correctly rounded by IEEE 754, and different C libraries return
results that differ in the last unit in the last place. Every one of the 400 or so steps in a
typical scenario here calls several of them, and the difference compounds.

Second, and more damaging, several of the reported quantities are threshold crossings. The time at
which the goal is reached is the first sample whose arc length exceeds `goal_s`. A difference of
one part in `10^15` in the arc length at the sample nearest the crossing moves the reported time by
a whole step, which is 0.05 s, or a full percent of a typical figure. The same applies to the time
at which the worst clearance occurred whenever the metric is nearly flat around its minimum, which
is the normal case for a vehicle settling into a following equilibrium. The quantity is not
continuous in the inputs, so no tolerance chosen in advance is both tight enough to be meaningful
and loose enough to be reliable.

Third, the run length itself is a threshold crossing when a scenario terminates on collision or on
goal, so even the number of samples is not obviously stable across platforms.

The baseline in `baselines/reference_suite.json` is therefore the complete recorded result, and
`tests/test_regression.py` reads from it only the quantities that are reproducible:

- the verdict of every scenario, exactly;
- the verdict of every assertion, matched by name, exactly;
- the number of scenarios, the number of assertions in each, and the pass and fail counts, exactly;
- the bound string each assertion was tested against, exactly, so that loosening a threshold to
  make a scenario pass shows up as a diff rather than as a silently green run;
- the sign and decade of every worst observed value, through `algorithm.compare.magnitude_class`,
  exactly.

and pins raw values numerically for one restricted set:

- assertions whose worst sample was taken in the first two seconds of the run, at a relative
  tolerance of `1e-9`. Two seconds is forty steps; a last bit difference in a transcendental
  function cannot have grown to one part in `10^9` in forty steps of a well conditioned explicit
  integration, and one part in `10^9` is far tighter than any behavioural change worth catching.

The magnitude class is deliberately coarse: `-3.2` and `-4.7` are both `-1e0`, while `-3.2` and
`-32.0` are not. It catches a change that alters an answer by a factor, which is what a real
regression looks like, and ignores a change in the fourth significant figure, which is what a
platform difference looks like. It has one honest weakness, which is stated in the docstring: a
value sitting within a rounding error of a decade boundary can flip class. That is why the baseline
is recorded from a real run and read, rather than generated blind, and why none of the pinned
values in the shipped suite sits near a boundary.

The comparison tool exposes the same distinction to a user rather than only to the tests. A change
of verdict, or a scenario or assertion that disappeared, is a regression and sets the exit code. A
metric that moved while its verdict held is reported with the size of the movement and does not set
the exit code, because it may be a deliberate tuning change and it may be the last quiet step
before a failure, and only the person reading it can tell which. A scenario that was renamed
without changing is neither: it is matched to its baseline entry by its results and reported as a
rename, for the reasons set out under closed limitations below.

## Rejected alternatives

### XML with an OpenSCENARIO schema

Adopting OpenSCENARIO XML directly would have made scenarios portable to commercial simulators and
removed the need to invent a format. It was rejected on two grounds. The standard is large, and a
partial implementation of a standard is worse than an honest small format because a reader assumes
the parts that are missing are merely unused rather than absent. More concretely, parsing XML with
schema validation requires a dependency, and the constraint here is the standard library.
`tomllib` gives strict parsing for free; what it does not give, domain validation, would have had
to be written by hand under either choice.

### A behaviour tree or a small scripting language for actors

A scripted actor here is an acceleration schedule and at most one lane change. An embedded
behaviour language would express far more. It was rejected because an actor that can branch on the
ego's state is an actor whose provocation depends on the controller under test, which makes the
scenario a different experiment for every candidate controller and makes a baseline much harder to
interpret. The reactive actor exists for the cases where coupling is the point, and it is exactly
one well documented model rather than an open ended language.

### A dynamic bicycle model with tyre forces

A dynamic model would score lateral behaviour more accurately at high lateral acceleration. It was
rejected because it is singular at low speed, which several of these scenarios reach when the ego
stops behind an obstacle, and because Kong et al. (2015) find the kinematic model adequate in the
regime these scenarios occupy. The cost is stated in the limitations below.

### Polygon collision checking

Separating axis tests on the true rectangles would remove the conservatism of the disc cover. It
was rejected because the disc form gives clearance and time to collision from the same closed form
query, and because a two dimensional time to collision on polygons has no equally simple closed
form. The conservatism is a constant, is documented, and points in the safe direction.

### Implementing responsibility sensitive safety as an assertion

RSS (Shalev-Shwartz, Shammah and Shashua, 2017) defines longitudinal and lateral safe distances
from stated worst case assumptions about reaction time and braking, and would be a stronger check
than a fixed minimum distance. It was not implemented because doing it properly means committing to
a parameter set and to a right of way model, both of which are policy choices rather than
engineering ones, and a half implemented RSS would give a number that looks authoritative and is
not. `min_distance` is presented as what it is, a fixed geometric threshold.

### Pinning the full trajectory in the regression baseline

Recording every state at every step and comparing with a tolerance was the obvious first design. It
was rejected for the reasons set out at length above. The middle option, pinning the worst values
with a loose relative tolerance such as one percent, was also rejected: a one percent tolerance is
too loose to catch a real behavioural change in a metric like peak deceleration, and still too
tight for a threshold crossing time that can move by a whole step.

### A single exit code rule

The obvious rule is that any failing assertion fails the suite. That rule cannot coexist with
shipping scenarios that are supposed to fail, and shipping them is not optional, because a harness
that has never been observed to detect a failure is a harness with no evidence behind it. The
alternative of putting the failing scenarios in a separate directory excluded from CI was rejected,
because it makes them second class and they stop being run. Instead each scenario declares its
`expected_outcome` and the runner offers both rules: the literal rule for an ordinary suite, and
the expectation rule for this one.

## Closed limitations

This section records what used to be in the list below, so that a reader can see which problems
were fixed rather than only which remain.

### Renaming a scenario read as a removal plus an addition

The comparison tool matched scenarios and assertions by name and by nothing else, so renaming
`merge_from_ramp` to `ramp_merge` produced a `scenario_removed` line, an unrelated `scenario_added`
line, and exit code 1, because a scenario that disappears is a regression. Nothing had changed
except a string, and the tool reported the loudest thing it can report.

It is now matched by content, which is how a version control system detects a rename.
`algorithm/compare.match_renames` pairs a scenario that left the suite with a scenario that joined
it when every assertion they carry agrees in name, kind, verdict and worst observed value to the
comparison tolerance. A pair is accepted only when each side is the other's single candidate. The
result is one informational `scenario_renamed` line, no regression, and exit code 0. The same rule
runs one level down for a single assertion renamed inside a scenario it stayed in.

The cost is a false positive that is stated rather than hidden. A genuine removal that happens to
coincide with a genuine addition reproducing every worst observed value to one part in `10^6` is
reported as a rename, which understates it. That is a narrow window: the two scenarios would have
to declare the same assertion names with the same verdicts and agree numerically on every one of
them. The ambiguity rule closes the wider hole, since two candidates on either side are left
reported as they were rather than guessed at, and the case is covered by
`tests/test_algorithm.py::test_an_ambiguous_rename_is_not_guessed`. The matching is also a
comparison of results, not of documents, so a rename that accompanies a real behavioural change is
not a rename here and stays reported as a removal and an addition.

What remains unchanged is the underlying fact that a stored baseline holds names and results and
nothing else. There is no identity in a scenario document that survives a rename, and this rule
recovers one from the results rather than introducing one. Giving each scenario a stable
identifier in its own document would be the stronger fix, and it was not taken, because an
identifier that has to be written by hand is an identifier that will be copied along with the file
it was pasted from.

## Known limitations

The ego's leader search treats a vehicle as in lane when the lateral separation is less than half
the sum of the two widths plus a 0.2 m margin. A vehicle straddling the lane line is therefore not
seen as a leader until it is most of the way across. This is a perception model, not a controller
bug, and it is a fair representation of a naive one, but it means the harness scores a cut in later
than a real system with lane level tracking would. It is why `scenarios/merge_from_ramp.toml` has
to place the merging vehicle 50 m ahead rather than 30 m for the scenario to be survivable; an
earlier version at 30 m produced a collision. Making the awareness band a declared property of the
ego rather than a constant in `pipeline/simulator.py` is the obvious change, and it is not
sufficient on its own. Widening the band to the width of a lane plus a vehicle, which is what lane
level tracking would see, moves the detection of the merging vehicle earlier by a fraction of a
second, and the braking demand at 30 m still saturates well outside the comfort band the scenario
asserts. The only band that restores the 30 m placement is one wider than a lane, at which point
the ego is reacting to a vehicle that is entirely in the next lane and the model is no longer a
perception model at all. Closing this properly means giving the actor a predicted lane occupancy
rather than a current one, which is a planner input rather than a constant, and that is why the
entry is still here.

The kinematic bicycle has no tyre model, so lateral acceleration above roughly 4 m/s^2 is not
physically meaningful and the lateral acceleration assertion should not be read as a grip check
there. The reported value is exact for the model, which is `v^2 tan(delta) / L`, and the curved
scenario reproduces `v^2 / R` to four figures, but the model itself stops representing a real
vehicle before the number stops being computable.

There is no lane change planner for the ego. Faced with a stopped obstacle the reference controller
stops behind it, which is why `scenarios/stopped_obstacle.toml` asserts a comfortable stop rather
than an avoidance manoeuvre. A scenario that can only be passed by changing lanes cannot currently
be passed at all.

The road is a single straight or a single constant curvature arc. There are no clothoid
transitions, no junctions, no grade, and no varying curvature. A scenario needing an intersection
cannot be written.

There is no pedestrian, cyclist, or static obstacle type. Everything is a rectangle with a bicycle
footprint, and a stopped obstacle is modelled as a vehicle with zero speed.

Perception noise is a Gaussian perturbation of the measured gap and leader speed. There is no
occlusion, no false negative, no latency, and no track loss, all of which dominate real perception
failures.

## What passing this suite does and does not mean

Passing this suite is evidence about the ten scenarios in it. It is not evidence about the
controller in general, about scenarios that were not written, or about the distribution of
situations the controller would encounter in service. A controller that passes every scenario here
can fail on the first situation nobody thought to write down, and nothing in this repository would
have predicted it.

Scenario coverage is the unsolved problem, and it is unsolved in the field rather than merely
unsolved here. Kalra and Paddock (2016) show that demonstrating autonomous vehicle safety by
accumulating road miles requires hundreds of millions of miles, and sometimes hundreds of billions,
which is why scenario based assessment exists at all. But scenario based assessment substitutes one
open question for another: on road testing asks how many miles are enough, and scenario testing
asks which scenarios are enough. Riedmaier et al. (2020) survey the approaches to generating and
selecting scenarios, and none of them yields a completeness argument. There is no accepted metric
that says a scenario set covers a domain, and this repository does not provide one.

What a suite like this actually provides is narrower and still worth having: a change to the
controller that breaks one of these ten specific situations is caught before it is merged, with the
worst value and the time it occurred attached, and the same ten situations are scored the same way
every time. That is regression detection. It is not a safety case, and it should never be presented
as one.
