
import { RVOMath, Simulator, Vector2 } from "rvo2";

const rvo = new Simulator();

function drawCicle(context: CanvasRenderingContext2D, position: Vector2, radius: number, color: string = "#ffff00"): void {
    context.beginPath();
    context.arc(position.x, position.y, radius, 0, Math.PI * 2, false);
    context.strokeStyle = color;
    context.stroke();
}

function circleExample(context: CanvasRenderingContext2D, center: Vector2) {
    const goals: Vector2[] = [];
    const radius = 3;

    const setupScenario = function () {
        /* Specify the global time step of the simulation. */
        rvo.setTimeStep(0.55);

        /*
         * Specify the default parameters for agents that are subsequently
         * added.
         */
        rvo.setAgentDefaults(15, 10, 10, 10, radius, 5.0, new Vector2);

        /*
         * Add agents, specifying their start position, and store their
         * goals on the opposite side of the environment.
         */
        const agentCount = 250;
        const distance = 280;
        for (let i = 0; i < agentCount; ++i) {
            const position = new Vector2(Math.cos(i * 2 * Math.PI / agentCount), Math.sin(i * 2 * Math.PI / agentCount));
            if(i % 30 == 0) {
                goals.push(position.clone().multiply(+100).add(center));
            } else {
                goals.push(position.clone().multiply(-distance).add(center));
            }
            position.multiply(distance).add(center);
            rvo.addAgent(position);
        }
    }

    const setPreferredVelocities = function () {
        /*
         * Set the preferred velocity to be a vector of unit magnitude
         * (speed) in the direction of the goal.
         */
        for (let i = 0; i < rvo.getNumAgents(); ++i) {
            const position = rvo.getAgentPosition(i);
            if(i % 30 ==0) {
                drawCicle(context, position, radius, "#ff0000");
            } else {
                drawCicle(context, position, radius);
            }
            let goalVector = Vector2.subtract(goals[i], position);

            if (RVOMath.absSq(goalVector) > 1) {
                goalVector = RVOMath.normalize(goalVector);
            }

            rvo.setAgentPrefVelocity(i, goalVector);
        }
    }

    const reachedGoal = function () {
        const vec2Temp = new Vector2();
        /* Check if all agents have reached their goals. */
        for (let i = 0; i < rvo.getNumAgents(); ++i) {
            const position = rvo.getAgentPosition(i);
            const radius = rvo.getAgentRadius(i);
            const direction = Vector2.subtract(position, goals[i], vec2Temp);
            if (RVOMath.absSq(direction) > radius * radius) {
                return false;
            }
        }

        return true;
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
    }
    step();
}

export function main() {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const context = canvas.getContext("2d");
    circleExample(context, new Vector2(canvas.width / 2, canvas.height / 2));
}

main();