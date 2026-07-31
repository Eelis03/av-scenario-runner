/**
 * Playback clock.
 *
 * Playback advances by wall clock time rather than one trace step per animation
 * frame, so the same recording plays at the same rate on a 60 Hz and a 120 Hz
 * display. The trace itself is never resampled; only the step index shown is
 * chosen from the elapsed time.
 */

export interface PlayerState {
  readonly step: number;
  readonly playing: boolean;
  readonly rate: number;
}

export class Player {
  private step = 0;
  private playing = false;
  private rate = 1;
  private lastFrame: number | null = null;
  private carry = 0;

  constructor(
    private steps: number,
    private dt: number,
    private readonly onChange: (state: PlayerState) => void,
  ) {}

  /** Point the player at a different recording, rewinding to its first step. */
  reset(steps: number, dt: number): void {
    this.steps = steps;
    this.dt = dt;
    this.step = 0;
    this.carry = 0;
    this.lastFrame = null;
    this.emit();
  }

  /** Jump to a step, clamped into range. Stops nothing. */
  seek(step: number): void {
    this.step = Math.min(Math.max(0, Math.round(step)), this.steps - 1);
    this.carry = 0;
    this.emit();
  }

  /** Start or stop playback. Starting from the last step rewinds first. */
  setPlaying(playing: boolean): void {
    if (playing && this.step >= this.steps - 1) {
      this.step = 0;
    }
    this.playing = playing;
    this.lastFrame = null;
    this.emit();
  }

  /** Playback rate as a multiple of real time. */
  setRate(rate: number): void {
    this.rate = rate;
    this.emit();
  }

  /** Advance the clock. Call once per animation frame with the frame timestamp. */
  tick(timestamp: number): void {
    if (!this.playing) {
      this.lastFrame = timestamp;
      return;
    }
    if (this.lastFrame === null) {
      this.lastFrame = timestamp;
      return;
    }
    const elapsed = (timestamp - this.lastFrame) / 1000;
    this.lastFrame = timestamp;
    this.carry += (elapsed * this.rate) / this.dt;
    const advance = Math.floor(this.carry);
    if (advance <= 0) {
      return;
    }
    this.carry -= advance;
    this.step += advance;
    if (this.step >= this.steps - 1) {
      this.step = this.steps - 1;
      this.playing = false;
    }
    this.emit();
  }

  /** The current state, for the caller to render. */
  state(): PlayerState {
    return { step: this.step, playing: this.playing, rate: this.rate };
  }

  private emit(): void {
    this.onChange(this.state());
  }
}
