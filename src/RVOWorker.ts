import { Simulator } from "./Simulator";

export class RVOWorker {
  // private readonly doneEvent_: ManualResetEvent;
  private readonly end_: number;
  private readonly start_: number;

  /**
   * Constructs and initializes a worker.
   * @param simulator Simulator.
   * @param start Start.
   * @param end End.
   */
  constructor(public readonly simulator: Simulator, start: number, end: number) { // , doneEvent: ManualResetEvent) {
    this.start_ = start;
    this.end_ = end;
    // this.doneEvent_ = doneEvent;
  }

  /**
   * Performs a simulation step.
   */
  public step() {
    for (let agentNo = this.start_; agentNo < this.end_; ++agentNo) {
      if (this.simulator.agents_[agentNo].isFreeze) continue;
      this.simulator.agents_[agentNo].computeNeighbors();
      this.simulator.agents_[agentNo].computeNewVelocity();
    }

    // this.doneEvent_.Set();
  }

  /**
   * updates the two-dimensional position and two-dimensional velocity of each agent.
   */
  public update() {
    for (let agentNo = this.start_; agentNo < this.end_; ++agentNo) {
      this.simulator.agents_[agentNo].update();
    }

    // this.doneEvent_.Set();
  }
}
