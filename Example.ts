
import { Simulator, Vector2 } from "rvo";

enum Layer {
    Layer1,
    Layer2,
    Layer3,
    Layer4,
}

const rvo = new Simulator();
rvo.initLayerMatrix([
    [Layer.Layer1, Layer.Layer3],
    [Layer.Layer2, Layer.Layer3],
    [Layer.Layer3, Layer.Layer2, Layer.Layer1],
    [],
])

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


interface AgentRender {
    id: number;
    oldColor?: string;
    color?: string;
}

interface SightRender {
    position: Vector2;
    radius: number;
    color: string;
}

interface ExampleResult {
    goals: readonly Vector2[];
    agentRender: readonly AgentRender[];
    obstacleRender?: readonly Vector2[][];
    sightRender?: readonly SightRender[];
}


function renderAgents(context: CanvasRenderingContext2D, center: Vector2, agentRender: readonly AgentRender[], obstacleRender?: readonly Vector2[][], sightRender?: readonly SightRender[]): void {
    if (sightRender != null) {
        for (const render of sightRender)
            drawCicle(context, render.position.x + center.x, render.position.y + center.y, render.radius, render.color);
    }
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
        if (goals[agentNo] == null) return;
        const position = rvo.getAgentPosition(agentNo);
        let goalVector = Vector2.subtract(goals[agentNo], position);
        rvo.setAgentPrefVelocity(agentNo, goalVector);
    });
}

function reachedGoal(example: ExampleResult, deltaTime: number): boolean {
    const vec2Temp = new Vector2();
    let result = true;
    /* Check if all agents have reached their goals. */
    rvo.forEachAgent((agentNo) => {
        const goalPosition = example.goals[agentNo];
        if (goalPosition == null) return;
        const position = rvo.getAgentPosition(agentNo);
        const radius = rvo.getAgentRadius(agentNo);
        const direction = Vector2.subtract(position, goalPosition, vec2Temp);
        const moveOffset = deltaTime * rvo.getAgentMaxSpeed(agentNo);
        if (direction.lengthSq() > moveOffset * moveOffset) {
            result = false;
        } else {
            // rvo.setAgentPrefVelocity(i, new Vector2());
            // rvo.setAgentVelocity(i, new Vector2());
            // rvo.setAgentMaxNeighbors(i, 0);
            rvo.setAgentPosition(agentNo, goalPosition);
            rvo.freezeAgent(agentNo);
            example.agentRender[agentNo].color = "#ff0000";
            // rvo.delAgent(agentNo);
        }
    });
    return result;
}

const collisionAgentIds: number[] = [];
function renderSightAgent(example: ExampleResult): void {
    if (example.sightRender == null) return;

    for (const agentNo of collisionAgentIds) {
        example.agentRender[agentNo].color = "#000000";
    }

    collisionAgentIds.length = 0;
    for (const render of example.sightRender) {
        rvo.queryNearAgent(render.position, render.radius, (agentId) => collisionAgentIds.push(agentId));
    }

    for (const agentNo of collisionAgentIds) {
        example.agentRender[agentNo].color = "#00ff00";
    }
}


function circleExample(): ExampleResult {
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
        let layer: number;
        if (i % 5 == 0) {
            goals.push(position.clone().multiply(+100));
            render.color = "#00ffff";
            layer = Layer.Layer1;
        } else {
            render.color = "#ffff00";
            goals.push(position.clone().multiply(-distance));
            layer = Layer.Layer2;
        }
        const offset = new Vector2(Math.random(), Math.random());
        position.multiply(distance).add(offset);
        rvo.addAgent(layer, position, radius, speed, 1, 0, 10);
    }

    return { goals, agentRender };
}

function blockExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 5;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];

    const speed = 100;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const scale = 25;
    const distance = 170;

    const addAgent = function (layer: number, position: Vector2) {
        return rvo.addAgent(layer, position, radius, speed, 1, 0.1, 10);
    }

    const count = 5;
    for (let i = 0; i < count; ++i) {
        for (let j = 0; j < count; ++j) {
            const position1 = new Vector2(distance + i * scale, distance + j * scale);
            const id1 = addAgent(Layer.Layer1, position1);

            const position2 = new Vector2(-distance - i * scale, distance + j * scale);
            const id2 = addAgent(Layer.Layer1, position2);

            const position3 = new Vector2(distance + i * scale, -distance - j * scale);
            const id3 = addAgent(Layer.Layer1, position3);

            const position4 = new Vector2(-distance - i * scale, -distance - j * scale);
            const id4 = addAgent(Layer.Layer1, position4);

            goals[id1] = position4;
            goals[id2] = position3;
            goals[id3] = position2;
            goals[id4] = position1;

            agentRender[id1] = { id: id1, color: "#eeee00" };
            agentRender[id2] = { id: id2, color: "#00eeee" };
            agentRender[id3] = { id: id3, color: "#ffff00" };
            agentRender[id4] = { id: id4, color: "#00e00e" };
        }
    }

    const ow = 50, oh = 50, ox = 80, oy = 80;
    const obstacle1: Vector2[] = [];
    obstacle1.push(new Vector2(-ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, +oh + oy));
    obstacle1.push(new Vector2(-ow + ox, +oh + oy));
    const obstacleId1 = rvo.addObstacle(Layer.Layer3, obstacle1);
    obstacleRender.push(obstacle1);

    const obstacle2: Vector2[] = [];
    obstacle2.push(new Vector2(-ow - ox, -oh + oy));
    obstacle2.push(new Vector2(+ow - ox, -oh + oy));
    obstacle2.push(new Vector2(+ow - ox, +oh + oy));
    obstacle2.push(new Vector2(-ow - ox, +oh + oy));
    const obstacleId2 = rvo.addObstacle(Layer.Layer3, obstacle2);
    obstacleRender.push(obstacle2);

    const obstacle3: Vector2[] = [];
    obstacle3.push(new Vector2(-ow - ox, -oh - oy));
    obstacle3.push(new Vector2(+ow - ox, -oh - oy));
    obstacle3.push(new Vector2(+ow - ox, +oh - oy));
    obstacle3.push(new Vector2(-ow - ox, +oh - oy));
    const obstacleId3 = rvo.addObstacle(Layer.Layer3, obstacle3);
    obstacleRender.push(obstacle3);

    const obstacle4: Vector2[] = [];
    obstacle4.push(new Vector2(-ow + ox, -oh - oy));
    obstacle4.push(new Vector2(+ow + ox, -oh - oy));
    obstacle4.push(new Vector2(+ow + ox, +oh - oy));
    obstacle4.push(new Vector2(-ow + ox, +oh - oy));
    const obstacleId4 = rvo.addObstacle(Layer.Layer3, obstacle4);
    obstacleRender.push(obstacle4);

    rvo.processObstacles();

    // rvo.delObstacle(obstacleId3);
    return { goals, agentRender, obstacleRender };
}

function circleObstacleExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 10;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];
    const speed = 50.0;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const agentCount = 20;
    const distance = 280;
    for (let i = 0; i < agentCount; ++i) {
        const position = new Vector2(Math.cos(i * 2 * Math.PI / agentCount), Math.sin(i * 2 * Math.PI / agentCount));
        goals.push(position.clone().multiply(-distance));
        const offset = new Vector2(Math.random(), Math.random());
        position.multiply(distance).add(offset);
        const agentId = rvo.addAgent(Layer.Layer1, position, radius, speed + Math.random() * 8);
        agentRender.push({ id: agentId, color: "#ffff00" });
    }

    const vertices = Simulator.createCircleObstacleVertices(new Vector2(30, 0), 200, 100);
    const obstacleId = rvo.addObstacle(Layer.Layer1, vertices);
    obstacleRender.push(vertices);

    // const ow = 50, oh = 50, ox = 0, oy = 0;
    // const obstacle1: Vector2[] = [];
    // obstacle1.push(new Vector2(-ow + ox, -oh + oy));
    // obstacle1.push(new Vector2(+ow + ox, -oh + oy));
    // obstacle1.push(new Vector2(+ow + ox, +oh + oy));
    // obstacle1.push(new Vector2(-ow + ox, +oh + oy));
    // const obstacleId1 = rvo.addObstacle(obstacle1);
    // obstacleRender.push(obstacle1);

    return { goals, agentRender, obstacleRender };
}

function blockObstacleExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 10;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];
    const speed = 50.0;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const agentCount = 2;
    const distance = 280;
    for (let i = 0; i < agentCount; ++i) {
        const position = new Vector2(Math.cos(i * 2 * Math.PI / agentCount), Math.sin(i * 2 * Math.PI / agentCount));
        goals.push(position.clone().multiply(-distance));
        const offset = new Vector2(Math.random(), Math.random());
        position.multiply(distance).add(offset);
        const agentId = rvo.addAgent(Layer.Layer1, position, radius, speed + Math.random() * 8);
        agentRender.push({ id: agentId, color: "#ffff00" });
    }

    const ow = 50, oh = 50, ox = 0, oy = 0;
    const obstacle1: Vector2[] = [];
    obstacle1.push(new Vector2(-ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, +oh + oy));
    obstacle1.push(new Vector2(-ow + ox, +oh + oy));
    const obstacleId1 = rvo.addObstacle(Layer.Layer1, obstacle1);
    obstacleRender.push(obstacle1);

    return { goals, agentRender, obstacleRender };
}

function avoidenceWeightExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 20;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];
    const speed = 50.0;
    /*
     * Add agents, specifying their start position, and store their
     * goals on the opposite side of the environment.
     */
    const agentCount = 3;
    const distance = 280;
    const heightOffset = 100;

    for (let i = 0; i < agentCount; ++i) {
        const position1 = new Vector2(distance + i, heightOffset * i);
        const id1 = rvo.addAgent(Layer.Layer1, position1, radius, speed);
        rvo.setAgentAvoidenceWeight(id1, 1.0);
        agentRender.push({ id: id1, color: "#ffff00" });

        const position2 = new Vector2(-distance - i, heightOffset * i);
        const id2 = rvo.addAgent(Layer.Layer1, position2, radius, speed);
        rvo.setAgentAvoidenceWeight(id2, 0.0);
        agentRender.push({ id: id2, color: "#00ffff" });

        goals[id1] = position2;
        goals[id2] = position1;
        obstacleRender.push([position1, position2]);
    }

    return { goals, agentRender, obstacleRender };
}

function layerExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 20;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];
    const speed = 500.0;

    const distance = 280;

    const position1 = new Vector2(distance, 0);
    const id1 = rvo.addAgent(Layer.Layer1, position1, radius, speed);
    rvo.setAgentAvoidenceWeight(id1, 1.0);
    agentRender.push({ id: id1, color: "#ffff00" });

    goals[id1] = new Vector2(-distance, 0);

    const ow = 50, oh = 50, ox = 0, oy = 0;
    const obstacle1: Vector2[] = [];
    obstacle1.push(new Vector2(-ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, -oh + oy));
    obstacle1.push(new Vector2(+ow + ox, +oh + oy));
    obstacle1.push(new Vector2(-ow + ox, +oh + oy));
    const obstacleId1 = rvo.addObstacle(Layer.Layer4, obstacle1);
    obstacleRender.push(obstacle1);

    return { goals, agentRender, obstacleRender };
}

function queryNearAgentExample(): ExampleResult {
    const goals: Vector2[] = [];
    const radius = 8;
    const agentRender: AgentRender[] = [];
    const obstacleRender: Vector2[][] = [];
    const sightRender: SightRender[] = [];
    const speed = 20.0;

    const scale = 40;
    const distance = 170;

    const addAgent = function (layer: number, position: Vector2, r: number = radius) {
        return rvo.addAgent(layer, position, r, speed);
    }

    const count = 5;
    for (let i = 0; i < count; ++i) {
        for (let j = 0; j < count; ++j) {
            const position1 = new Vector2(distance + i * scale, j * scale);
            const id1 = addAgent(Layer.Layer1, position1);

            const position2 = new Vector2(-distance - i * scale, j * scale);
            const id2 = addAgent(Layer.Layer1, position2);

            goals[id1] = position2;
            goals[id2] = position1;

            agentRender[id1] = { id: id1, color: "#eeeeee" };
            agentRender[id2] = { id: id2, color: "#eeeeee" };
        }
    }

    sightRender.push({ position: new Vector2(0, 0), radius: 80, color: "#eeee00" });
    sightRender.push({ position: new Vector2(20, 130), radius: 50, color: "#eeee00" });

    return { goals, agentRender, obstacleRender, sightRender };
}

export function main() {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const context = canvas.getContext("2d");
    const center = new Vector2(canvas.width / 2, canvas.height / 2);
    // const result = queryNearAgentExample();
    // const result = blockExample();
    // const result = circleExample();
    // const result = circleObstacleExample();
    // const result = blockObstacleExample();
    // const result = avoidenceWeightExample();
    const result = layerExample();

    rvo.processObstacles();
    let lastTime = Date.now();
    /* Perform (and manipulate) the simulation. */
    const step = function () {
        context.clearRect(0, 0, center.x * 2, center.y * 2);
        const curTime = Date.now();
        const deltaTime = Math.min(0.1, (curTime - lastTime) / 1000);

        lastTime = curTime;
        setPreferredVelocities(result.goals);
        rvo.doStep(deltaTime);
        renderSightAgent(result);
        if (!reachedGoal(result, deltaTime)) {
            requestAnimationFrame(step);
        }
        renderAgents(context, center, result.agentRender, result.obstacleRender, result.sightRender);
    }
    step();
}

main();