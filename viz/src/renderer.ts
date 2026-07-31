/**
 * Canvas rendering of one step of a trace.
 *
 * The camera follows the ego and rotates with it, so a curved road reads the
 * same as a straight one and the viewer never has to chase the vehicle across
 * the canvas. Distances are in metres throughout; the only pixel arithmetic is
 * the single world to screen transform applied at the start of each frame.
 */

import { footprint, frenetToCartesian, laneBoundary } from "./geometry.js";
import type { Body, Trace } from "./types.js";
import { at, egoOf } from "./types.js";

const EGO_FILL = "#2f6fb2";
const EGO_EDGE = "#123c66";
const ACTOR_FILL = "#c2603f";
const ACTOR_EDGE = "#7a331c";
const CONTACT_FILL = "#b3202a";

export interface ViewOptions {
  /** Pixels per metre. */
  readonly scale: number;
  /** Draw the path each body has taken so far. */
  readonly showTrails: boolean;
}

function devicePixels(canvas: HTMLCanvasElement): { width: number; height: number } {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(canvas.clientWidth * ratio));
  const height = Math.max(1, Math.round(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height };
}

function strokePolyline(
  context: CanvasRenderingContext2D,
  points: { x: number; y: number }[],
): void {
  if (points.length === 0) {
    return;
  }
  context.beginPath();
  context.moveTo(points[0]!.x, points[0]!.y);
  for (let index = 1; index < points.length; index += 1) {
    context.lineTo(points[index]!.x, points[index]!.y);
  }
  context.stroke();
}

function drawRoad(context: CanvasRenderingContext2D, trace: Trace): void {
  const road = trace.road;
  context.lineWidth = 0.12;
  for (let boundary = 0; boundary <= road.lanes; boundary += 1) {
    const outer = boundary === 0 || boundary === road.lanes;
    context.strokeStyle = outer ? "#8a8f98" : "#c9cdd4";
    context.setLineDash(outer ? [] : [3, 3]);
    strokePolyline(context, laneBoundary(road, boundary));
  }
  context.setLineDash([]);

  context.strokeStyle = "#e4e7ec";
  context.lineWidth = 0.06;
  for (let s = 0; s <= road.length; s += 25) {
    const inner = frenetToCartesian(road, s, -road.lane_width / 2);
    const outerPoint = frenetToCartesian(road, s, farSideOffset(road) + road.lane_width / 2);
    context.beginPath();
    context.moveTo(inner.x, inner.y);
    context.lineTo(outerPoint.x, outerPoint.y);
    context.stroke();
  }
}

/** Lateral offset of the centre of the leftmost lane. */
function farSideOffset(road: Trace["road"]): number {
  return (road.lanes - 1) * road.lane_width;
}

function drawBody(
  context: CanvasRenderingContext2D,
  body: Body,
  step: number,
  contact: boolean,
): void {
  const corners = footprint(at(body.x, step), at(body.y, step), at(body.yaw, step), body.shape);
  const isEgo = body.role === "ego";
  context.beginPath();
  context.moveTo(corners[0]!.x, corners[0]!.y);
  for (let index = 1; index < corners.length; index += 1) {
    context.lineTo(corners[index]!.x, corners[index]!.y);
  }
  context.closePath();
  context.fillStyle = contact ? CONTACT_FILL : isEgo ? EGO_FILL : ACTOR_FILL;
  context.fill();
  context.lineWidth = 0.08;
  context.strokeStyle = isEgo ? EGO_EDGE : ACTOR_EDGE;
  context.stroke();

  const nose = corners[1]!;
  const tip = corners[2]!;
  context.beginPath();
  context.moveTo((nose.x + tip.x) / 2, (nose.y + tip.y) / 2);
  context.lineTo(at(body.x, step), at(body.y, step));
  context.lineWidth = 0.05;
  context.strokeStyle = "#ffffff";
  context.stroke();
}

function drawTrail(context: CanvasRenderingContext2D, body: Body, step: number): void {
  const points: { x: number; y: number }[] = [];
  for (let index = 0; index <= step; index += 1) {
    points.push({ x: at(body.x, index), y: at(body.y, index) });
  }
  context.lineWidth = 0.1;
  context.strokeStyle = body.role === "ego" ? "rgba(47,111,178,0.45)" : "rgba(194,96,63,0.35)";
  strokePolyline(context, points);
}

/** Draw one frame of `trace` at `step` into `canvas`. */
export function render(
  canvas: HTMLCanvasElement,
  trace: Trace,
  step: number,
  options: ViewOptions,
): void {
  const context = canvas.getContext("2d");
  if (context === null) {
    return;
  }
  const { width, height } = devicePixels(canvas);
  context.setTransform(1, 0, 0, 1, 0, 0);
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#f7f8fa";
  context.fillRect(0, 0, width, height);

  const ego = egoOf(trace);
  const egoX = at(ego.x, step);
  const egoY = at(ego.y, step);
  const egoYaw = at(ego.yaw, step);

  context.save();
  // World to screen: metres to pixels, y upward, ego at one third from the left.
  context.translate(width / 3, height / 2);
  context.scale(options.scale, -options.scale);
  context.rotate(-egoYaw);
  context.translate(-egoX, -egoY);

  drawRoad(context, trace);
  if (options.showTrails) {
    for (const body of trace.bodies) {
      drawTrail(context, body, step);
    }
  }
  const clearance = trace.metrics.clearance[step];
  const contact = clearance !== null && clearance !== undefined && clearance <= 0;
  for (const body of trace.bodies) {
    if (body.role !== "ego") {
      drawBody(context, body, step, contact);
    }
  }
  drawBody(context, ego, step, contact);
  context.restore();
}
