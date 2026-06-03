//
// Created by west on 5/6/26.
//

#ifndef INC_441SIM_RENDERER_H
#define INC_441SIM_RENDERER_H


#include <functional>
#include <thread>
#include <utility>
#include <vector>

#include <imgui.h>

// Forward declare
class Drawable;

// Useful constants to have while rendering
inline double pxPerMeter = 0.0;
inline double scale = 0;

/**
 * Singleton that manages all drawing to the ImGui
 * visualization window.
 */
class Renderer {
public:
    /**
     * Singleton
     * @return Renderer instance
     */
    static Renderer& getInstance();

    /**
     * Deleted because singleton
     */
    Renderer(const Renderer&) = delete;
    void operator=(const Renderer&) = delete;

    /**
     * Set the rendering scale.
     * This is intended to be called by the user at some point in Sim.h
     * @param meters Side length of visualization square in meters
     */
    static void setScale(double meters);

    /**
     * Draw all objects
     */
    void draw() const;

    /**
     * Register a drawable object with the Renderer.
     * The robot renders to layer zero, so anything below the robot should use
     * a negative layer and anything to be superimposed should use a positive
     * layer.
     * @param layer Layer to draw on
     * @param drawable Pointer to obj that implements Drawable
     */
    void registerDrawable(int layer, Drawable* drawable);

    /**
     * Register a function to be called at render time.
     * This is useful for drawing things that don't directly relate to the robot
     * or hardware.
     * The robot renders to layer zero, so anything below the robot should use
     * a negative layer and anything to be superimposed should use a positive
     * layer.
     * @param layer Layer to draw on
     * @param func Callable block of code
     */
    void registerDrawFunction(int layer, const std::function<void()>& func);

    /**
     * This provides a method for a drawable object that goes out of scope
     * to remove itself from the draw list.
     * @param drawable Drawable object to remove
     */
    void unregisterDrawable(Drawable* drawable);

    /**
     * Helper function to draw a rectangle to draw a rectangle to the
     * visualization window.
     * @param x X position of rect center
     * @param y Y position of rect center
     * @param theta Angle of rect from +x-axis
     * @param w Width of rect
     * @param h Height of rect
     * @param color Color
     */
    static void drawRect(double x, double y, double theta, double w, double h, ImColor color);

private:
    /**
     * Private because singleton
     */
    Renderer() = default;
    ~Renderer() = default;

    /**
     * Used to synchronize threads when drawable objects exist within user code.
     * This is primarily an issue if a `Drawable` object goes out of scope in the Arduino
     * thread and the Renderer attempts to `draw` it before it has finished de-registering
     * itself from the Renderer.
     *
     * Note: Use of mutable here to allow locking in `const` member functions
     */
    mutable std::mutex m;

    /**
     * Vectors of things to draw, be they objects or functions to call
     */
    std::vector<std::pair<int, Drawable*>> drawables;
    std::vector<std::pair<int, std::function<void()>>> drawFunctions;

    /**
     * Guards against erroneous ImGui calls when not drawing a frame
     */
    inline static bool drawing;
};

/**
 * Interface for objects to implement that allows them to draw something
 * to the ImGui visualization window.
 */
class Drawable {
public:
    /**
     * Implementer must specify drawing layer
     * @param layer Layer to draw to
     */
    explicit Drawable(const int layer) : l(layer) {
        Renderer::getInstance().registerDrawable(l, this);
    }

    /**
     * Copy constructor and copy assignment to ensure that copied objects
     * will still be drawn, even if the original drawable object goes out
     * of scope.
     */
    Drawable(const Drawable& other) : l(other.l) {
        Renderer::getInstance().registerDrawable(other.l, this);
    }

    Drawable& operator=(const Drawable& other) {
        if (this != &other) {
            l = other.l;
            Renderer::getInstance().registerDrawable(other.l, this);
        }

        return *this;
    }

    /**
     * Dtor unregisters self from Renderer
     */
    virtual ~Drawable() {
        Renderer::getInstance().unregisterDrawable(this);
    }

    /**
     * Draw function to be implemented by derived classes
     */
    virtual void draw() const {};

protected:
    /**
     * Included so that objects that run in the Arduino thread can be drawable
     * Right now it's the responsibility of the implementer to properly
     * synchronize threads when drawing, even though that's not ideal.
     */
    mutable std::mutex drawMtx;

private:
    int l;
};

#endif //INC_441SIM_RENDERER_H
