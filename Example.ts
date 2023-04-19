
import { Simulator, Vector2 } from "rvo";

const rvo = new Simulator();

function drawCicle(context: CanvasRenderingContext2D, x: number, y: number, radius: number, color: string = "#ffff00"): void {
    context.beginPath();
    context.arc(x, y, radius, 0, Math.PI * 2, false);
    context.strokeStyle = color;
    context.stroke();
}

function drawPath(context: CanvasRenderingContext2D, path: readonly Vector2[], offset: Vector2, color: string = "#ffff00"): void {
    context.strokeStyle = color;
    context.beginPath();
    context.moveTo(path[0].x + offset.x, path[0].y + offset.y);
    for (let i = 0; i < path.length; i++) {
        context.lineTo(path[i].x + offset.x, path[i].y + offset.y);
    }
    context.closePath();
    context.stroke();
}

interface ExampleResult {
    goals: readonly Vector2[];
    agentRender: readonly AgentRender[];
    obstacleRender?: readonly Vector2[][];
}

interface AgentRender {
    id: number; color?: string;
}

function renderAgents(context: CanvasRenderingContext2D, center: Vector2, agentRender: readonly AgentRender[], obstacleRender?: readonly Vector2[][]): void {
    if (obstacleRender != null) {
        for (const obstacle of obstacleRender)
            drawPath(context, obstacle, center, "#883300");
    }
    for (const render of agentRender) {
        const position = rvo.getAgentPosition(render.id);
        const radius = rvo.getAgentRadius(render.id);
        drawCicle(context, position.x + center.x, position.y + center.y, radius, render.color);
    }
}

function setPreferredVelocities(goals: readonly Vector2[]): void {
    /*
     * Set the preferred velocity to be a vector of unit magnitude
     * (speed) in the direction of the goal.
     */
    rvo.forEachAgent((agentNo) => {
        const position = rvo.getAgentPosition(agentNo);
        let goalVector = Vector2.subtract(goals[agentNo], position);
        rvo.setAgentPrefVelocity(agentNo, goalVector);
    });
}

function reachedGoal(example: ExampleResult): boolean {
    const vec2Temp = new Vector2();
    let result = true;
    /* Check if all agents have reached their goals. */
    rvo.forEachAgent((agentNo) => {
        const position = rvo.getAgentPosition(agentNo);
        const radius = rvo.getAgentRadius(agentNo);
        const direction = Vector2.subtract(position, example.goals[agentNo], vec2Temp);
        if (direction.lengthSq() > radius * radius) {
            result = false;
        } else {
            // rvo.setAgentPrefVelocity(i, new Vector2());
            // rvo.setAgentVelocity(i, new Vector2());
            // rvo.setAgentMaxNeighbors(i, 0);
            rvo.freezeAgent(agentNo);
            example.agentRender[agentNo].color = "#ff0000";
            // rvo.delAgent(agentNo);
        }
    });
    return result;
}


function circleExample(context: CanvasRenderingContext2D, center: Vector2): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 5;
    const agentRender: AgentRender[] = [];

    const speed = 50.0;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const agentCount = 85;
    const distance = 280;
    for (let i = 0; i < agentCount; ++i) {
        const position = new Vector2(Math.cos(i * 2 * Math.PI / agentCount), Math.sin(i * 2 * Math.PI / agentCount));
        const render = { id: i, color: "#ffff00", arrive: false };
        agentRender.push(render);
        if (i % 5 == 0) {
            goals.push(position.clone().multiply(+100));
            render.color = "#00ffff";
        } else {
            render.color = "#ffff00";
            goals.push(position.clone().multiply(-distance));
        }
        const offset = new Vector2(Math.random(), Math.random());
        position.multiply(distance).add(offset);
        rvo.addAgent(position, radius, speed, 10, 10, 10, 10);
    }

    return { goals, agentRender };
}

function blockExample(context: CanvasRenderingContext2D, center: Vector2): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 5;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];

    const speed = 100;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const scale = 30;
    const distance = 150;

    const addAgent = function (position: Vector2) {
        return rvo.addAgent(position, radius, speed, 1, 0.1, 10, radius * 4);
    }

    for (let i = 0; i < 5; ++i) {
        for (let j = 0; j < 5; ++j) {
            const position1 = new Vector2(distance + i * scale, distance + j * scale);
            const id1 = addAgent(position1);

            const position2 = new Vector2(-distance - i * scale, distance + j * scale);
            const id2 = addAgent(position2);

            const position3 = new Vector2(distance + i * scale, -distance - j * scale);
            const id3 = addAgent(position3);

            const position4 = new Vector2(-distance - i * scale, -distance - j * scale);
            const id4 = addAgent(position4);

            goals[id1] = position4;
            goals[id2] = position3;
            goals[id3] = position2;
            goals[id4] = position1;

            agentRender[id1] = { id: id1, color: "#00ffff" };
            agentRender[id2] = { id: id2, color: "#00aaff" };
            agentRender[id3] = { id: id3, color: "#0044ff" };
            agentRender[id4] = { id: id4, color: "#0000ff" };
        }
    }

    const ow = 50, oh = 50, ox = 80, oy = 80;
    const obstacle1: Vector2[] = [];
    obstacle1.push(new Vector2(-ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, +oh + oy));
    obstacle1.push(new Vector2(-ow + ox, +oh + oy));
    const obstacleId1 = rvo.addObstacle(obstacle1);
    obstacleRender.push(obstacle1);

    const obstacle2: Vector2[] = [];
    obstacle2.push(new Vector2(-ow - ox, -oh + oy));
    obstacle2.push(new Vector2(+ow - ox, -oh + oy));
    obstacle2.push(new Vector2(+ow - ox, +oh + oy));
    obstacle2.push(new Vector2(-ow - ox, +oh + oy));
    const obstacleId2 = rvo.addObstacle(obstacle2);
    obstacleRender.push(obstacle2);

    const obstacle3: Vector2[] = [];
    obstacle3.push(new Vector2(-ow - ox, -oh - oy));
    obstacle3.push(new Vector2(+ow - ox, -oh - oy));
    obstacle3.push(new Vector2(+ow - ox, +oh - oy));
    obstacle3.push(new Vector2(-ow - ox, +oh - oy));
    const obstacleId3 = rvo.addObstacle(obstacle3);
    obstacleRender.push(obstacle3);

    const obstacle4: Vector2[] = [];
    obstacle4.push(new Vector2(-ow + ox, -oh - oy));
    obstacle4.push(new Vector2(+ow + ox, -oh - oy));
    obstacle4.push(new Vector2(+ow + ox, +oh - oy));
    obstacle4.push(new Vector2(-ow + ox, +oh - oy));
    const obstacleId4 = rvo.addObstacle(obstacle4);
    obstacleRender.push(obstacle4);

    rvo.processObstacles();

    // rvo.delObstacle(obstacleId3);
    // rvo.processObstacles();
    return { goals, agentRender, obstacleRender };
}

function circleObstacleExample(context: CanvasRenderingContext2D, center: Vector2): ExampleResult {
    return null;
}

export function main() {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const context = canvas.getContext("2d");
    const center = new Vector2(canvas.width / 2, canvas.height / 2);
    const result = blockExample(context!, center);
    // const result = circleExample(context, center);

    let lastTime = Date.now();
    /* Perform (and manipulate) the simulation. */
    const step = function () {
        context.clearRect(0, 0, center.x * 2, center.y * 2);
        const curTime = Date.now();
        const deltaTime = (curTime - lastTime) / 1000;

        lastTime = curTime;
        /* Specify the global time step of the simulation. */
        rvo.setTimeStep(deltaTime);
        setPreferredVelocities(result.goals);
        rvo.doStep();
        if (!reachedGoal(result))
            requestAnimationFrame(step);
        renderAgents(context, center, result.agentRender, result.obstacleRender);
    }
    step();
}

main();