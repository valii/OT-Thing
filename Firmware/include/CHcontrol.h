#pragma once

#include "ArduinoJson.h"
#include "heatingcurve.h"
#include "sensors.h"
#include "HADiscLocal.h"
#include "scheduler.h"
#include "sensors.h"
#include <vector>

template <typename T1>
class ChannelOverride {
public:
    bool active; // overriding activated
    T1 value; // overriden value (set from roomunit)
    bool isMasterOverride() const;
};

class CHcontrol {
private:
    bool roomCompEnabled() const;
    bool getChActive() const;
    bool getCurrentFlow(double &d) const;
    const uint8_t channel;
    struct Config {
        double roomSet; // default room set point
        double flow; // default flow temperature 
        struct {
            bool enabled; // suspend on roomsetpoint
            double hysteresis;
            double offset;
        } roomSuspend;
        struct {
            bool enabled;
            double p; // Kp K/K
            double i; // Ki 1/h
            double boost; // Kb K/K
        } roomComp;
        bool minSuspend;
        double minSuspendHyst;
        struct AdaptMin {
            bool enabled;       // adaptive lower flow limit
            double step;        // K per step
            double waitMin;     // minutes to wait for the modulation to settle after a step
            double modTol;      // % tolerance: modulation "stays at the previous level"
            double maxRise;     // K, max. rise above flowMin
            double reprobeMin;  // minutes in HOLD before probing upwards again (0 = never)
        } adaptMin;
        struct OutsideSuspend {
            enum {
                OUTSIDE_SUSPEND_DISABLED,
                OUTSIDE_SUSPEND_CURRENT,
                OUTSIDE_SUSPEND_AVERAGE
            } type;
            double hysteresis;
            double offset;
        } outsideSuspend;
    } config;
    struct PiCtrl {
        HADiscovery::ClimateMode mode {HADiscovery::MODE_AUTO};
        bool init { false };
        double rspPrev; // previous room setpoint
        double integState {0}; // state of integrator / K
        double deltaT {0}; // result of PI controller
    } roomComp;
    bool roomSuspended {false};
    struct {
        double integState {0}; // state of integrator / K
        double reduction {0}; // result of PI controller
    } retLimit;
    bool minSuspended {false};
    // adaptive lower flow limit: searches the highest flow setpoint the boiler can hold
    // at its previous (minimum) modulation, to avoid on/off cycling at low load
    struct {
        enum State { IDLE, PROBING, HOLD } state {IDLE};
        bool init {false};
        double flow {0};        // current adaptive lower limit
        double calcFlow {0};    // last calculated flow before clipping
        bool prevValid {false};
        double prevAvg {0};     // average modulation of the previous window
        double modSum {0};
        uint16_t modCnt {0};
        unsigned long windowStart {0};
        unsigned long lastSample {0};
        unsigned long holdSince {0};
    } adapt;
    void loopAdaptMin();
    void adaptStartWindow(const unsigned long now);
    double getFlowFloor() const;
    bool outSuspended {false};
    HeatingCurve curve;
    Scheduler schedule;
    double flowTemp;
    Sensor::Source lastFlowTempSrc;
    struct {
        bool init {false};
        double min;
        double max;
    } flowStats;
public:
    CHcontrol(const uint8_t channel);
    void setConfig(JsonObject &obj, const bool init);
    void getJson(JsonObject &obj);
    double getFlow();
    bool getChOn();
    double getFlowMax() const;
    bool suspendEnabled() const;
    bool loop();
    void loopRoomComp();
    void loopReturnLimit();
    void setMode(const HADiscovery::ClimateMode mode);
    void setRoomComp(const HADiscovery::ClimateMode mode);
    bool sendDiscoveries(const bool en);
    void setFlowTemp(const double temp, const Sensor::Source src);
    void flameChange(const bool newFlame);
    double flowMin;
    ChannelOverride<bool> ovrdOn;
    ChannelOverride<double> ovrdTemp;
    HADiscovery::ClimateMode mode {HADiscovery::MODE_AUTO};
    struct Turbo {
        double shift {10.0};
        time_t endTime {0};
    } turbo;
};