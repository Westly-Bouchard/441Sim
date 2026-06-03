import * as vscode from 'vscode';
import * as http from 'http';
import * as fs from 'fs';
import * as path from 'path';
import { spawn } from 'child_process';

import { ensureCompilerAvailable } from './compilerManager';

// Create a dedicated output tab for your extension
const buildChannel = vscode.window.createOutputChannel("Mosscap Build");

async function buildSimulator(workspaceRoot: string, context: vscode.ExtensionContext): Promise<void> {
    const emccPath = await ensureCompilerAvailable(context);

    const buildPath = path.join(workspaceRoot, 'build');
    if (!fs.existsSync(buildPath)) {
        fs.mkdirSync(buildPath, { recursive: true });
    }

    return new Promise((resolve, reject) => {
        vscode.window.withProgress({
            location: vscode.ProgressLocation.Notification,
            title: "Compiling Sketch...",
            cancellable: false
        }, async () => {
            try {
                // Find the main user file (assuming it's in a src folder or workspace root)
                const userSketch = path.join(workspaceRoot, 'ardMain.cpp');

                const simCoreDir = path.join(context.extensionPath, 'sim-core');
                const mainCpp = path.join(simCoreDir, 'main.cpp');
                const libPath = path.join(simCoreDir, 'libsimcore.a');
                const imguiPath = path.join(simCoreDir, 'libimgui.a');
                const includePath = path.join(simCoreDir, 'include');
                const arduinoIncludePath = path.join(simCoreDir, 'include', 'arduino');

                const ems = "-std=c++23 -pthread -sDISABLE_EXCEPTION_CATCHING=1 -sUSE_BOOST_HEADERS=1 --use-port=contrib.glfw3";
                const ldflags = "-s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s NO_EXIT_RUNTIME=0 -s ASSERTIONS=1 -s PTHREAD_POOL_SIZE=4 -s INITIAL_MEMORY=64MB";

                // Compile main.cpp and user sketch
                const compileCommand = `\
                    "${emccPath}" \
                    "${userSketch}" \
                    "${mainCpp}" \
                    "${libPath}" \
                    "${imguiPath}" \
                    -o build/sim.html \
                    -I"${arduinoIncludePath}" \
                    -I"${workspaceRoot}" \
                    -I"${includePath}" \
                    ${ems} \
                    ${ldflags} \
                    --shell-file ${path.join(simCoreDir, 'shell.html')}\
                `;

                await runCommand(compileCommand, workspaceRoot);
                resolve();
            } catch (error) {
                reject(error);
            }
        });
    });
}

let simulatorServer: http.Server | undefined;
let port = -1;

function startSimulatorServer(workspaceRoot: string): Promise<number> {
    return new Promise((resolve, reject) => {
        // If server is not already running
        if (!simulatorServer) {
            // Create it and set proper headers so that the extension can run with threads
            simulatorServer = http.createServer((req, res) => {
                // Inject the headers for WebAssembly Threads (SharedArrayBuffer)
                res.setHeader("Cross-Origin-Opener-Policy", "same-origin");
                res.setHeader("Cross-Origin-Embedder-Policy", "require-corp");

                // Look for the files in the 'build' folder of the open workspace
                const requestUrl = req.url === '/' ? '/sim.html' : req.url;
                const filePath = path.join(workspaceRoot, 'build', requestUrl!);

                fs.readFile(filePath, (err, data) => {
                    if (err) {
                        res.writeHead(404);
                        res.end("File not found");
                        return;
                    }

                    const ext = path.extname(filePath);
                    let contentType = 'text/html';
                    if (ext === '.js' || ext === '.worker.js') contentType = 'application/javascript';
                    else if (ext === '.wasm') contentType = 'application/wasm';

                    res.writeHead(200, { 'Content-Type': contentType });
                    res.end(data);
                });
            });

            // Start on a random port
            simulatorServer.listen(0, '127.0.0.1', () => {
                // const address = simulatorServer?.address();
                // if (address && typeof address === 'object') {
                //     port = address.port;
                // } 

                const address = simulatorServer?.address();
                if (address && typeof address === 'object') {
                    resolve(address.port);
                } else {
                    reject(new Error("Failed to get port"));
                }
            });
        } else {
            const address = simulatorServer?.address();
            if (address && typeof address === 'object') {
                resolve(address.port);
            } else {
                reject(new Error("Failed to get port"));
            }
        }
    });
}

async function startSimulator(rootPath: string, context: vscode.ExtensionContext) {
    try {
        await buildSimulator(rootPath, context);

        const port = await startSimulatorServer(rootPath);

        const localUrl = vscode.Uri.parse(`http://127.0.0.1:${port}/sim.html`);
        await vscode.env.openExternal(localUrl);
    } catch (error) {
        vscode.window.showErrorMessage(`Failed to start simulator: ${error}`);
    }
}

export function activate(context: vscode.ExtensionContext) {
    let init = vscode.commands.registerCommand('mosscap.init', async () => {
        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (!workspaceFolders) {
            vscode.window.showErrorMessage("Please open a folder.");
            return;
        }

        const emccPath = await ensureCompilerAvailable(context);

        const rootPath = workspaceFolders[0].uri.fsPath;

        // Create .vscode folder with proper include paths for simulator and emscripten
        const vsCodePath = path.join(rootPath, '.vscode');
        if (!fs.existsSync(vsCodePath)) {
            fs.mkdirSync(vsCodePath);
        }

        const cppPropertiesPath = path.join(vsCodePath, 'c_cpp_properties.json');
        try {
            let content = {
                "configurations": [
                    {
                        "compilerPath": emccPath,
                        "includePath": [
                            path.join(context.extensionPath, 'sim-core', 'include'),
                            path.join(context.extensionPath, 'sim-core', 'include', 'arduino')
                        ],
                        "cppStandard": "c++23"
                    }
                ]
            };
            fs.writeFileSync(cppPropertiesPath, JSON.stringify(content));
        } catch (err) {
            vscode.window.showErrorMessage(`Failed to create C++ configuration file: ${err}`);
            return;
        }
    });

    let simStart = vscode.commands.registerCommand('mosscap.start', async () => {
        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (!workspaceFolders) {
            vscode.window.showErrorMessage("Please open a folder containing your simulator build.");
            return;
        }


        const rootPath = workspaceFolders[0].uri.fsPath;

        startSimulator(rootPath, context);
    });

    let openDemo = vscode.commands.registerCommand('mosscap.demo', async () => {
        const p = path.join(context.extensionPath, 'demos', 'intro');

        startSimulator(p, context);
    });

    let openDemoSPA = vscode.commands.registerCommand('mosscap.demoSPA', async () => {
        const p = path.join(context.extensionPath, 'demos', 'sense-plan-act');

        startSimulator(p, context);
    });

    context.subscriptions.push(init);
    context.subscriptions.push(simStart);
    context.subscriptions.push(openDemo);
    context.subscriptions.push(openDemoSPA);
}

export function deactivate() {
    if (simulatorServer) {
        simulatorServer.close();
    }
}

function runCommand(command: string, cwd: string): Promise<void> {
    return new Promise((resolve, reject) => {
        buildChannel.clear();
        buildChannel.show(true);

        // Print the command we are about to run
        buildChannel.appendLine(`> Executing: ${command}\n`);
        
        const child = spawn(command, { cwd, shell: true });
        
        let output = "";

        // Stream standard output
        child.stdout.on('data', (data) => { 
            const text = data.toString();
            output += text; 
            buildChannel.append(text); // Print to the Output tab live
        });

        // Stream error output (Emscripten uses this for almost everything)
        child.stderr.on('data', (data) => { 
            const text = data.toString();
            output += text; 
            buildChannel.append(text); // Print to the Output tab live
        });

        child.on('close', (code) => {
            buildChannel.appendLine(`\n> Process exited with code ${code}\n`);
            if (code === 0) {
                resolve(); 
            } else {
                reject(output || `Process exited with code ${code}`);
            }
        });
    });
}