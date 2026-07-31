/**
 * Road frame geometry, mirroring `scenario_runner.model.geometry`.
 *
 * Kept deliberately small and duplicated rather than shipped from Python,
 * because the alternative is a build step that couples the browser bundle to
 * the Python package. The two implementations are three lines of trigonometry
 * each and are checked against one another by eye on the rendered lane
 * boundaries: a mismatch is immediately visible as a road that does not follow
 * the vehicles.
 */

import type { Road, Shape } from "./types.js";

export interface Point {
  readonly x: number;
  readonly y: number;
}

/** Cartesian position of the road point at arc length `s` and lateral offset `d`. */
export function frenetToCartesian(road: Road, s: number, d: number): Point {
  if (road.curvature === 0) {
    return { x: s, y: d };
  }
  const radius = 1 / road.curvature;
  const theta = s * road.curvature;
  const effective = radius - d;
  return {
    x: effective * Math.sin(theta),
    y: radius - effective * Math.cos(theta),
  };
}

/** Lateral offset of the centre of a lane. Lane 0 is the rightmost lane. */
export function laneOffset(road: Road, lane: number): number {
  return lane * road.lane_width;
}

/** Polyline for one lane boundary, sampled along the road. */
export function laneBoundary(road: Road, boundary: number, samples = 240): Point[] {
  const offset = laneOffset(road, boundary) - road.lane_width / 2;
  const points: Point[] = [];
  for (let index = 0; index <= samples; index += 1) {
    const s = (road.length * index) / samples;
    points.push(frenetToCartesian(road, s, offset));
  }
  return points;
}

/** The four corners of a vehicle footprint in world coordinates. */
export function footprint(x: number, y: number, yaw: number, shape: Shape): Point[] {
  const cos = Math.cos(yaw);
  const sin = Math.sin(yaw);
  const half = shape.width / 2;
  const local: [number, number][] = [
    [shape.rear_offset, -half],
    [shape.front_offset, -half],
    [shape.front_offset, half],
    [shape.rear_offset, half],
  ];
  return local.map(([along, across]) => ({
    x: x + along * cos - across * sin,
    y: y + along * sin + across * cos,
  }));
}
