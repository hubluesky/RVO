
import { RVOMath, Simulator, Vector2 } from "rvo";

const rvo = new Simulator();

function drawCicle(context: CanvasRenderingContext2D, position: Vector2, radius: number, color: string = "#ffff00"): void {
    context.beginPath();
    context.arc(position.x, position.y, radius, 0, Math.PI * 2, false);
    context.strokeStyle = color;
    context.stroke();
}

function circleExample(context: CanvasRenderingContext2D, center: Vector2) {
    const goals: Vector2[] = [];
    const radius = 6;
    const agentRender: { id: number, arrive: boolean, color: string }[] = [];
    const setupScenario = function () {
        /* Specify the global time step of the simulation. */
        rvo.setTimeStep(0.25);

        /*
         * Specify the default parameters for agents that are subsequently
         * added.
         */
        rvo.setAgentDefaults(radius * 4, 10, 10, 10, radius, 5.0, new Vector2);

        /*
         * Add agents, specifying their start position, and store their
         * goals on the opposite side of the environment.
         */
        const agentCount = 85;
        const distance = 280;
        for (let i = 0; i < agentCount; ++i) {
            const position = new Vector2(Math.cos(i * 2 * Math.PI / agentCount), Math.sin(i * 2 * Math.PI / agentCount));
            const render = { id: i, color: null, arrive: false };
            agentRender.push(render);
            if (i % 5 == 0) {
                goals.push(position.clone().multiply(+100).add(center));
                render.color = "#00ffff";
            } else {
                render.color = "#ffff00";
                goals.push(position.clone().multiply(-distance).add(center));
            }
            const offset = new Vector2(Math.random(), Math.random());
            position.multiply(distance).add(center).add(offset);
            rvo.addAgent(position);
        }
    }

    const setPreferredVelocities = function () {
        /*
         * Set the preferred velocity to be a vector of unit magnitude
         * (speed) in the direction of the goal.
         */
        for (let i = 0; i < rvo.getNumAgents(); ++i) {
            // if (agentRender[i].arrive) continue;

            const position = rvo.getAgentPosition(i);
            let goalVector = Vector2.subtract(goals[i], position);

            if (RVOMath.absSq(goalVector) > 1) {
                goalVector = RVOMath.normalize(goalVector);
            }

            rvo.setAgentPrefVelocity(i, goalVector);
        }
    }

    const renderAgents = function (): void {
        for (const render of agentRender) {
            const position = rvo.getAgentPosition(render.id);
            drawCicle(context, position, radius, render.color);
        }
    }

    const reachedGoal = function () {
        const vec2Temp = new Vector2();
        let result = true;
        /* Check if all agents have reached their goals. */
        for (let i = 0; i < rvo.getNumAgents(); ++i) {
            const position = rvo.getAgentPosition(i);
            const radius = rvo.getAgentRadius(i);
            const direction = Vector2.subtract(position, goals[i], vec2Temp);
            if (RVOMath.absSq(direction) > radius * radius) {
                result = false;
            } else {
                // rvo.setAgentPrefVelocity(i, new Vector2());
                // rvo.setAgentMaxNeighbors(i, 0);
                rvo.agents_[i].isFreeze = true;
                agentRender[i].color = "#ff0000";
                agentRender[i].arrive = true;
            }
        }

        return result;
    }

    /* Set up the scenario. */
    setupScenario();

    /* Perform (and manipulate) the simulation. */
    const step = function () {
        context.clearRect(0, 0, center.x * 2, center.y * 2);
        setPreferredVelocities();
        rvo.doStep();
        if (!reachedGoal())
            requestAnimationFrame(step);
        renderAgents();
    }
    step();
}

export function main() {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const context = canvas.getContext("2d");
    circleExample(context, new Vector2(canvas.width / 2, canvas.height / 2));
}

main();