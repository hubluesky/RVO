import typescript from "rollup-plugin-typescript2";
import dts from "rollup-plugin-dts";

const input = "src/Index.ts";
const outputName = "rvo";

export default [{
    input: input,
    output: {
        // sourcemap: "inline",
        file: `./dist/${outputName}.js`,
        format: "es",
        // name: 'rvo',
    },
    plugins: [
        typescript({
            tsconfig: "tsconfig.json",
            tsconfigOverride: { compilerOptions: { declaration: false, paths: {} }, include: ["./src"] }
        })
    ]
}, {
    input: input,
    output: {
        file: `./dist/${outputName}.d.ts`,
        format: "es",
    },
    plugins: [dts()]
}, {
    input: "./Example.ts",
    output: {
        file: `./dist/Example.js`,
        format: "es",
    },
    plugins: [typescript({
        tsconfig: "tsconfig.json",
        tsconfigOverride: { include: ["./Example.ts"] }
    })]
}
];