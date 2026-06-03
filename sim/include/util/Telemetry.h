//
// Created by west on 5/7/26.
//

#ifndef INC_441SIM_TELEMETRY_H
#define INC_441SIM_TELEMETRY_H

#include <thread>
#include <utility>
#include <vector>

// Forward declare
class DataLink;

/**
 * Singleton that manages all telemetry writing
 * to the ImGui telemetry window.
 */
class Telemetry {
public:
    /**
     * Singleton
     * @return Telemetry instance
     */
    static Telemetry& getInstance();

    /**
     * Deleted because singleton
     */
    Telemetry(const Telemetry&) = delete;
    void operator=(const Telemetry&) = delete;

    /**
     * Write from all providers
     */
    void write() const;

    /**
     * Register a provider with the Telemetry system.
     * The robot / simulator writes with priority zero. So values above
     * any robot telemetry should write with a negative priority and
     * values that should appear below the robot telemetry should use a
     * positive priority.
     * @param priority Priority of provider
     * @param provider Pointer to obj that implements TelemetryProvider
     */
    void registerDataLink(int priority, DataLink* provider);

    /**
     * This provides a method for a provider that goes out of scope to remove
     * itself from the list of providers.
     * @param provider Provider to remove
     */
    void unregisterDataLink(DataLink* provider);

    /**
     * This just wraps the ImgGui::CollapsingHeader function
     * The goal is to insulate implementers from direct ImGui calls
     * in the common case.
     *
     * @param name Name of header at the top of the section
     * @return Whether the header is collapsed
     */
    static bool section(const std::string& name);

    /**
     * Similarly to Telemetry::section, this just wraps the ImGui::Text
     * function to insulate implementers from direct ImGui calls in the
     * common case.
     * @param text Text to display
     */
    static void text(const std::string& text);

private:
    /**
     * Private because singleton
     */
    Telemetry() = default;
    ~Telemetry() = default;

    /**
     * Used to synchronize threads when DataLinks exist within user code.
     * Similar to the mutex in the Renderer.
     */
    mutable std::mutex m;

    /**
     * The vector of providers
     */
    std::vector<std::pair<int, DataLink*>> providers;

    /**
     * Guards against erroneous ImGui calls when not drawing a frame
     */
    inline static bool writing = false;
};

/**
 * Interface for objects to implement that allows them to write telemetry
 * data to the ImGui telemetry window.
 */
class DataLink {
public:
    /**
     * Implementer must specify the priority
     * @param priority Priority of data
     */
    explicit DataLink(const int priority) : p(priority) {
        Telemetry::getInstance().registerDataLink(priority, this);
    };

    /**
     * Copy constructor and copy assignment to ensure that copied Providers will
     * still be written even if the original provider goes out of scope.
     */
    DataLink(const DataLink& other) : p(other.p){
        Telemetry::getInstance().registerDataLink(other.p, this);
    }

    DataLink &operator=(const DataLink& other) {
        if (this != &other) {
            p = other.p;
            Telemetry::getInstance().registerDataLink(other.p, this);
        }

        return *this;
    }

    /**
     * Dtor unregisters the object from the telemetry system
     */
    virtual ~DataLink() {
        Telemetry::getInstance().unregisterDataLink(this);
    }

    /**
     * Write function to be implemented by derived classes
     */
    virtual void update() = 0;

private:
    int p;
};

class TelemetryProvider : public DataLink {
public:
    explicit TelemetryProvider(const int priority) : DataLink(priority) {}

    virtual void write() const = 0;

    void update() override {
        /*
         * In the case that the TelemetryProvider is running in the Arduino thread,
         * we must lock the object when writing to synchronize the threads before
         * writing.
         */
        linkMtx.lock();
        write();
        linkMtx.unlock();
    }

protected:
    /**
     * Included so that objects that run on the Arduino thread can provide
     * telemetry if desired. At the moment, properly synchronizing threads
     * is the responsibility of the implementer, this is not ideal.
     */
    std::mutex linkMtx;
};

// Type alias for DataLink to signal that something accepts input semantically
using InputAcceptor = DataLink;

#endif //INC_441SIM_TELEMETRY_H
