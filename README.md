<div align="center">
    <img src='media/Mosscap-SPA-Demo.gif' alt='Sense plan act demo' style="width: 75%;"/>
    <p style="width: 70%;">A small demo that showcases a simulated robot mapping its environment, path planning with A*, and executing the first few moves in the path.</p>
</div>


# Mosscap

Put simply: Mosscap is a simulator for Arduino based robots. But, more broadly, Mosscap aims to be a friction-less, accessible tool for robotics education.

Learning robotics can be incredibly difficult because it is so interdisciplinary. At the same time, hardware (even simple starter kits on Amazon) can be prohibitively expensive, and most robotics simulators are not designed with beginners in mind. These existing tools are incredibly powerful, but they assume that the user possesses a great amount of prerequisite knowledge. In practice, this means that robotics learners must learn foundational robotics theory in parallel with the tools to apply it. While this can have some advantages, it can also be incredibly overwhelming. 

Broadly, these factors combine to create a formidable set of barriers to entry into robotics as a field.

Mosscap aims to remove some of these barriers by significantly reducing the complexity of tooling and eliminating the cost of hardware. It provides a VS Code extension that packages a browser-based simulator with an Arduino interface and a set of semi-guided programming challenges to introduce fundamental robotics concepts.

By leveraging learners' previous Arduino experience (still a barrier, but a significantly smaller one) and providing a simplified interface which abstracts away the complexity of C++ tool-chains and build systems, Mosscap serves as an ideal entry point for learners; allowing them to explore and build confidence without feeling overwhelmed.

# Installation
Mosscap runs inside of VS Code, the industry standard text / code editor. Instructions for its installation can be found through the [VS Code Documentation](https://code.visualstudio.com/docs/setup/setup-overview).

Additionally, Mosscap relies on the [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) extension. See the [Documentation](https://code.visualstudio.com/docs/configure/extensions/extensions) for more information about installing extensions.

Finally, the Mosscap extension can be installed from the extensions tab. That's it!
# Opening the Demo
To ensure that everything is working properly Mosscap provides a demo that you can run right out of the box. Hit `Ctrl/Cmd+Shift+P` to open the command palette, search "Mosscap: Open Demo", and hit enter.

The first time you do this, the extension will download and install the necessary tools to compile and run the simulator. This may take a couple of minutes, depending on the speed of your internet connection and computer.

After this, the demo sketch should open in your system's default internet browser. Take a minute or two to play around with it. The different buttons in the telemetry window will cause the robot to move in different ways in the simulated environment.

Here's the cool part, this behavior is all coming from an Arduino sketch! The sketch can tell if you're pressing a button by calling `digitalRead()`, and it tells the motors on the robot to run with `motor.run()`, just like you would do with an actual Arduino.
# Start Learning
Now that you're all set up, it's time to start coding! Head to the [Getting Started](https://github.com/Westly-Bouchard/Mosscap/wiki/Getting-Started) page on the GitHub wiki to learn how to set up a new sketch.

After that, check out the series of provided coding challenges designed to teach you the basics of robot programming.