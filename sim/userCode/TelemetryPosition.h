#ifndef TELEMETRY_POSITION
#define TELEMETRY_POSITION

#include <util/Telemetry.h>
#include <imgui.h>
#include <sstream>

class TelemetryPose : public TelemetryProvider {
public:
    TelemetryPose() : TelemetryProvider(1) {}

    double getX() { return x; }

    double getY() { return y; }

    double getTheta() { return theta; }

    void setX(const double newX) {
        linkMtx.lock();
        x = newX;
        linkMtx.unlock();
    }

    void setY(const double newY) {
        linkMtx.lock();
        y = newY;
        linkMtx.unlock();
    }

    void setTheta(const double newTheta) {
        linkMtx.lock();
        theta = newTheta;
        linkMtx.unlock();
    }

    void write() const override {
        std::stringstream ss;
        
        if (Telemetry::section("Odometry")) {
            ss << "X\t: " << x;
            Telemetry::text(ss.str());

            ss.str("");
            ss << "Y\t: " << y;
            Telemetry::text(ss.str());

            ss.str("");
            ss << "Theta\t: " << theta;
            Telemetry::text(ss.str());
        }
    }

private:
    double x{0.0}, y{0.0}, theta{0.0};
};

#endif