/**
 * The trace schema written by `scenario_runner.analysis.export`.
 *
 * The version is checked before anything is drawn. A viewer that renders a
 * document it does not understand produces a picture that looks authoritative
 * and is wrong, which is worse than refusing to draw.
 */

export const SUPPORTED_TRACE_VERSION = "1.0";

/** A value that may be absent because it was not finite in the source run. */
export type MaybeNumber = number | null;

export interface Shape {
  readonly length: number;
  readonly width: number;
  readonly front_offset: number;
  readonly rear_offset: number;
}

export interface Body {
  readonly id: string;
  readonly role: "ego" | "actor";
  readonly shape: Shape;
  readonly x: MaybeNumber[];
  readonly y: MaybeNumber[];
  readonly yaw: MaybeNumber[];
  readonly speed: MaybeNumber[];
}

export interface Road {
  readonly kind: string;
  readonly lanes: number;
  readonly lane_width: number;
  readonly length: number;
  readonly curvature: number;
  readonly speed_limit: number;
}

export interface Metrics {
  readonly clearance: MaybeNumber[];
  readonly time_to_collision: MaybeNumber[];
  readonly ego_accel: MaybeNumber[];
  readonly ego_lateral_accel: MaybeNumber[];
  readonly ego_s: MaybeNumber[];
}

export interface AssertionVerdict {
  readonly name: string;
  readonly kind: string;
  readonly passed: boolean;
  readonly worst_value: MaybeNumber;
  readonly worst_time: MaybeNumber;
  readonly unit: string;
  readonly bound: string;
}

export interface Trace {
  readonly trace_version: string;
  readonly scenario: string;
  readonly dt: number;
  readonly seed: number;
  readonly terminated: string;
  readonly collided: boolean;
  readonly road: Road;
  readonly time: MaybeNumber[];
  readonly bodies: Body[];
  readonly metrics: Metrics;
  readonly assertions: AssertionVerdict[];
}

export class TraceError extends Error {}

function requireField(value: unknown, path: string): unknown {
  if (value === undefined || value === null) {
    throw new TraceError(`trace is missing the required field ${path}`);
  }
  return value;
}

/**
 * Validate a parsed JSON document and narrow it to `Trace`.
 *
 * Only the structure the renderer relies on is checked. The aim is a clear
 * message at load time rather than an exhaustive schema check.
 */
export function parseTrace(raw: unknown): Trace {
  if (typeof raw !== "object" || raw === null) {
    throw new TraceError("trace is not a JSON object");
  }
  const candidate = raw as Record<string, unknown>;
  const version = requireField(candidate.trace_version, "trace_version");
  if (version !== SUPPORTED_TRACE_VERSION) {
    throw new TraceError(
      `unsupported trace_version ${String(version)}; this viewer reads ${SUPPORTED_TRACE_VERSION}`,
    );
  }
  requireField(candidate.road, "road");
  requireField(candidate.time, "time");
  const bodies = requireField(candidate.bodies, "bodies");
  if (!Array.isArray(bodies) || bodies.length === 0) {
    throw new TraceError("trace contains no bodies");
  }
  const trace = candidate as unknown as Trace;
  const steps = trace.time.length;
  for (const body of trace.bodies) {
    if (body.x.length !== steps || body.y.length !== steps || body.yaw.length !== steps) {
      throw new TraceError(`body ${body.id} has a series of the wrong length`);
    }
  }
  return trace;
}

/** The ego body, which every trace has exactly one of. */
export function egoOf(trace: Trace): Body {
  const ego = trace.bodies.find((body) => body.role === "ego");
  if (ego === undefined) {
    throw new TraceError("trace has no ego body");
  }
  return ego;
}

/** Read a series value at a step, falling back to zero when it was not finite. */
export function at(series: MaybeNumber[], step: number): number {
  const value = series[step];
  return value === null || value === undefined ? 0 : value;
}
