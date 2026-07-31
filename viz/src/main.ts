/**
 * DOM wiring for the scenario playback.
 *
 * A trace can arrive two ways: fetched from `traces/` when the page is served
 * over HTTP, or opened from disk with the file picker, which works from a
 * `file://` URL where fetch is blocked. Neither path contacts the network.
 */

import { Player } from "./player.js";
import { render } from "./renderer.js";
import type { MaybeNumber, Trace } from "./types.js";
import { at, egoOf, parseTrace, TraceError } from "./types.js";

const BUNDLED_TRACES = [
  "cut_in_moderate",
  "lead_vehicle_braking",
  "aggressive_cut_in",
  "stopped_obstacle_no_brake",
];

function element<T extends HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (found === null) {
    throw new Error(`missing element #${id}`);
  }
  return found as T;
}

function formatMetric(value: MaybeNumber, digits = 2, unit = ""): string {
  if (value === null || value === undefined) {
    return "n/a";
  }
  return `${value.toFixed(digits)}${unit}`;
}

class Viewer {
  private trace: Trace | null = null;
  private readonly canvas = element<HTMLCanvasElement>("stage");
  private readonly scrub = element<HTMLInputElement>("scrub");
  private readonly playButton = element<HTMLButtonElement>("play");
  private readonly rateSelect = element<HTMLSelectElement>("rate");
  private readonly scaleInput = element<HTMLInputElement>("scale");
  private readonly trailsInput = element<HTMLInputElement>("trails");
  private readonly sourceSelect = element<HTMLSelectElement>("source");
  private readonly fileInput = element<HTMLInputElement>("file");
  private readonly status = element<HTMLParagraphElement>("status");
  private readonly readout = element<HTMLDListElement>("readout");
  private readonly verdicts = element<HTMLUListElement>("verdicts");
  private readonly player: Player;

  constructor() {
    this.player = new Player(1, 0.05, () => this.draw());
    this.bind();
  }

  private bind(): void {
    for (const name of BUNDLED_TRACES) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      this.sourceSelect.append(option);
    }
    this.sourceSelect.addEventListener("change", () => {
      void this.loadBundled(this.sourceSelect.value);
    });
    this.fileInput.addEventListener("change", () => {
      const file = this.fileInput.files?.[0];
      if (file !== undefined) {
        void this.loadFile(file);
      }
    });
    this.playButton.addEventListener("click", () => {
      this.player.setPlaying(!this.player.state().playing);
    });
    this.scrub.addEventListener("input", () => {
      this.player.setPlaying(false);
      this.player.seek(Number(this.scrub.value));
    });
    this.rateSelect.addEventListener("change", () => {
      this.player.setRate(Number(this.rateSelect.value));
    });
    this.scaleInput.addEventListener("input", () => this.draw());
    this.trailsInput.addEventListener("change", () => this.draw());
    window.addEventListener("resize", () => this.draw());

    const frame = (timestamp: number): void => {
      this.player.tick(timestamp);
      window.requestAnimationFrame(frame);
    };
    window.requestAnimationFrame(frame);
  }

  /** Fetch one of the traces exported into `traces/`. Requires an HTTP origin. */
  async loadBundled(name: string): Promise<void> {
    try {
      const response = await fetch(`traces/${name}.json`);
      if (!response.ok) {
        throw new TraceError(`traces/${name}.json responded ${response.status}`);
      }
      this.accept(parseTrace(await response.json()));
    } catch (error) {
      this.fail(
        error instanceof Error ? error.message : String(error),
        "Serve this directory over HTTP, or use the file picker.",
      );
    }
  }

  /** Read a trace the user opened from disk. */
  async loadFile(file: File): Promise<void> {
    try {
      this.accept(parseTrace(JSON.parse(await file.text())));
    } catch (error) {
      this.fail(error instanceof Error ? error.message : String(error), "");
    }
  }

  private accept(trace: Trace): void {
    this.trace = trace;
    this.scrub.max = String(trace.time.length - 1);
    this.scrub.value = "0";
    this.player.reset(trace.time.length, trace.dt);
    this.status.textContent = `${trace.scenario}: ${trace.bodies.length} bodies, ${trace.time.length} steps, ended on ${trace.terminated}`;
    this.status.className = "status ok";
    this.renderVerdicts(trace);
    this.draw();
  }

  private fail(message: string, hint: string): void {
    this.trace = null;
    this.status.textContent = hint ? `${message}. ${hint}` : message;
    this.status.className = "status bad";
    this.verdicts.replaceChildren();
    this.readout.replaceChildren();
  }

  private renderVerdicts(trace: Trace): void {
    this.verdicts.replaceChildren();
    for (const verdict of trace.assertions) {
      const item = document.createElement("li");
      item.className = verdict.passed ? "verdict pass" : "verdict fail";
      const label = document.createElement("span");
      label.className = "verdict-name";
      label.textContent = verdict.name;
      const value = document.createElement("span");
      value.className = "verdict-value";
      value.textContent = `${verdict.passed ? "pass" : "fail"} worst ${formatMetric(
        verdict.worst_value,
        3,
      )} ${verdict.unit} at ${formatMetric(verdict.worst_time, 2, " s")} bound ${verdict.bound}`;
      item.append(label, value);
      this.verdicts.append(item);
    }
  }

  private draw(): void {
    const trace = this.trace;
    if (trace === null) {
      return;
    }
    const state = this.player.state();
    this.scrub.value = String(state.step);
    this.playButton.textContent = state.playing ? "Pause" : "Play";

    render(this.canvas, trace, state.step, {
      scale: Number(this.scaleInput.value),
      showTrails: this.trailsInput.checked,
    });

    const ego = egoOf(trace);
    const rows: [string, string][] = [
      ["time", `${at(trace.time, state.step).toFixed(2)} s`],
      ["ego speed", `${at(ego.speed, state.step).toFixed(2)} m/s`],
      ["longitudinal a", `${at(trace.metrics.ego_accel, state.step).toFixed(2)} m/s^2`],
      ["lateral a", `${at(trace.metrics.ego_lateral_accel, state.step).toFixed(2)} m/s^2`],
      ["clearance", formatMetric(trace.metrics.clearance[state.step] ?? null, 2, " m")],
      [
        "time to collision",
        formatMetric(trace.metrics.time_to_collision[state.step] ?? null, 2, " s"),
      ],
      ["arc length", `${at(trace.metrics.ego_s, state.step).toFixed(1)} m`],
    ];
    this.readout.replaceChildren();
    for (const [name, value] of rows) {
      const term = document.createElement("dt");
      term.textContent = name;
      const definition = document.createElement("dd");
      definition.textContent = value;
      this.readout.append(term, definition);
    }
  }
}

const viewer = new Viewer();
void viewer.loadBundled(BUNDLED_TRACES[0]!);
