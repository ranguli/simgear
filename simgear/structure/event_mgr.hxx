#ifndef _SG_EVENT_MGR_HXX
#define _SG_EVENT_MGR_HXX

#include <simgear/props/props.hxx>
#include <simgear/structure/subsystem_mgr.hxx>

#include "callback.hxx"

class SGEventMgr;

class SGTimer
{
public:
    SGTimer() = default;
    ~SGTimer();
    void run();

    std::string name;
    double interval = 0.0;
    SGCallback* callback = nullptr;
    bool repeat = false;
    bool running = false;

    // Allow move only
    SGTimer& operator=(const SGTimer &) = delete;
    SGTimer(const SGTimer &other) = delete;
    SGTimer& operator=(SGTimer &&) = default;
    SGTimer(SGTimer &&other) = default;
};

/*! Queue to execute SGTimers after given delays */
class SGTimerQueue
{
public:
    SGTimerQueue() = default;
    ~SGTimerQueue() = default;

    void clear();
    void update(double deltaSecs, std::map<std::string, double> &timingStats);
    void insert(std::unique_ptr<SGTimer> timer, double time);
    bool removeByName(const std::string& name);

    void dump();

private:
    std::unique_ptr<SGTimer> remove();
    double nextTime() const { return _table[0].pri; }

    struct Entry {
        double pri;
        std::unique_ptr<SGTimer> timer;

        bool operator>(const Entry& other) const { return pri > other.pri; }
    };

    std::unique_ptr<SGTimer> _current_timer;
    double _now = 0.0;
    std::vector<Entry> _table;
};

class SGEventMgr : public SGSubsystem
{
public:
    SGEventMgr();
    ~SGEventMgr();

    // Subsystem API.
    void init() override;
    void shutdown() override;
    void unbind() override;
    void update(double delta_time_sec) override;

    // Subsystem identification.
    static const char* staticSubsystemClassId() { return "events"; }

    void setRealtimeProperty(SGPropertyNode* node) { _rtProp = node; }

    /**
     * Add a single function callback event as a repeating task.
     * ex: addTask("foo", &Function ... )
     */
    template<typename FUNC>
    inline void addTask(const std::string& name, const FUNC& f,
                        double interval, double delay=0, bool sim=false)
    { add(name, make_callback(f), interval, delay, true, sim); }

    /**
     * Add a single function callback event as a one-shot event.
     * ex: addEvent("foo", &Function ... )
     */
    template<typename FUNC>
    inline void addEvent(const std::string& name, const FUNC& f,
                         double delay, bool sim=false)
    { add(name, make_callback(f), 0, delay, false, sim); }

    /**
     * Add a object/method pair as a repeating task.
     * ex: addTask("foo", &object, &ClassName::Method, ...)
     */
    template<class OBJ, typename METHOD>
    inline void addTask(const std::string& name,
                        const OBJ& o, METHOD m,
                        double interval, double delay=0, bool sim=false)
    { add(name, make_callback(o,m), interval, delay, true, sim); }

    /**
     * Add a object/method pair as a repeating task.
     * ex: addEvent("foo", &object, &ClassName::Method, ...)
     */
    template<class OBJ, typename METHOD>
    inline void addEvent(const std::string& name,
                         const OBJ& o, METHOD m,
                         double delay, bool sim=false)
    { add(name, make_callback(o,m), 0, delay, false, sim); }


    void removeTask(const std::string& name);

    void dump();

private:
    friend class SGTimer;

    void add(const std::string& name, SGCallback* cb,
             double interval, double delay,
             bool repeat, bool simtime);

    SGPropertyNode_ptr _freezeProp;
    SGPropertyNode_ptr _rtProp;
    SGTimerQueue _rtQueue;
    SGTimerQueue _simQueue;
    bool _inited, _shutdown;
};

#endif // _SG_EVENT_MGR_HXX
