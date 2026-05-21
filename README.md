# Mosscap

Mosscap is a simulation platform for Arduino based robots like [these](https://osoyoo.com/2022/07/05/v2-metal-chassis-mecanum-wheel-robotic-for-arduino-mega2560-introduction-model-2021006600/). The goal of this project is to provide
a tool that can be used for robotics education from high school through college. Essentially,
Mosscap addresses the following problem statement:

> Robotics hardware is often prohibitively expensive. Even simple wheeled mobile robot kits that
> can be purchased on Amazon can exceed $100 in cost, which puts them out of reach for many students
> or hobbyists.
> 
> Additionally, many software-based educational tools and simulators provide their own barriers to
> entry in the form of complicated installation and usage patterns, along with large amounts of
> assumed prerequisite knowledge for their use.
> 
> Naturally, the question arises: How might one create an educational robotics platform that is
> simple to install and use, and that can leverage users' existing knowledge to provide educational
> opportunities without such barriers to entry?

Mosscap addresses this problem by packaging a lightweight physics simulator and Arduino interface
into a Visual Studio Code extension to allow for simple installation. It leverages WebAssembly through
emscripten to compile a user's Arduino sketch and open the simulation interface in browser.

This circumvents the hassle of c++ build systems and dependency management and simplifies the usage
requirements to two simple items:
1. Do you have a computer?
2. Can that computer run VS Code?

If the answer to both of these is yes, then you can use Mosscap to start learning robotics.

# The Basics
## Installation
First, you have to install VS Code if you haven't already. Instructions for this can be
found through the [VS Code Documentation](https://code.visualstudio.com/docs/setup/setup-overview).

Next, you can install Mosscap from the [VS Code Extensions Marketplace](https://marketplace.visualstudio.com/items?itemName=westly-bouchard.mosscap). You should also install the [C/C++
Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack) (or the regular [C/C++ Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) either one should work just fine).

> Note that you can install extensions from within VS Code from the extensions tab (on the left), or
> by pressing `Ctrl/Cmd+Shift+X`.
## Opening the Demo
Mosscap provides a quick demo that you can run right out of the box, without writing any code.

Open the command palette (hit `Ctrl/Cmd+Shift+P`, or click the search bar at the top middle of the window), type `Mosscap: Open Demo`, and hit enter. Here's what will happen next:

1. Mosscap will download and install the necessary tools to compile and run the simulator.
This may take a couple of minutes, depending on the speed of your internet connection and
your machine.

2. Mosscap will compile the [demo sketch](https://github.com/Westly-Bouchard/Mosscap/blob/main/demos/intro/ardMain.cpp). The extension should pop open an output terminal where it reports the commands it's
running.

3. Mosscap will open the simulator in your web browser. Now, it's time to have some fun!

Take a few minutes to play around with the demo. This demo features a simple tank (differential)
drive robot that is wired up to some buttons. Notice that pressing the button labeled `Forward`
will cause the robot to drive forward.

Here's the cool part, this is all running from a simple Arduino sketch!

## Extension Interface
Besides opening the demo, the Mosscap extension provides two commands for you to use:

1. `Mosscap: Initialize Environment` -> This is used when you first create a sketch, it tells VS Code
where to look for libraries and tools to provide IntelliSense while you're writing code.
In other words, this makes those annoying red squiggles go away.

2. `Mosscap: Start Simulator` -> This command compiles the sketch you currently have open,
and opens the simulator in a web browser (like it did with the demo), this is the command
you will use most.

# Writing Your First Mosscap Sketch
See the [Getting Started](https://github.com/Westly-Bouchard/Mosscap/wiki/Getting-Started) section on the GitHub wiki!