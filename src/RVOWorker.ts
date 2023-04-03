import { Simulator } from "./Simulator";

export class RVOWorker {
  // private readonly doneEvent_: ManualResetEvent;
  private readonly end_: number;
  private readonly start_: number;

  /**
   * <summary>Constructs and initializes a worker.</summary>
   *
   * <param name="start">Start.</param>
   * <param name="end">End.</param>
   * <param name="doneEvent">Done event.</param>
   */
  constructor(public readonly simulator: Simulator, start: number, end: number) { // , doneEvent: ManualResetEvent) {
    this.start_ = start;
    this.end_ = end;
    // this.doneEvent_ = doneEvent;
  }

  /**
   * <summary>Performs a simulation step.</summary>
   *
   * <param name="obj">Unused.</param>
   */
  public step() {
    for (let agentNo = this.start_; agentNo < this.end_; ++agentNo) {
      this.simulator.agents_[agentNo].computeNeighbors();
      this.simulator.agents_[agentNo].computeNewVelocity();
    }

    // this.doneEvent_.Set();
  }

  /**
   * <summary>updates the two-dimensional position and
   * two-dimensional velocity of each agent.</summary>
   *
   * <param name="obj">Unused.</param>
   */
  public update() {
    for (let agentNo = this.start_; agentNo < this.end_; ++agentNo) {
      this.simulator.agents_[agentNo].update();
    }

    // this.doneEvent_.Set();
  }
}
