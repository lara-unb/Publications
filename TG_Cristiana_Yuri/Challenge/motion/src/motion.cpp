#include <motion.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include <../runswift/motion/MotionAdapter.hpp>
#include <../runswift/utils/options.hpp>

#include <cmath>

#define FREQUENCY 50

namespace po = boost::program_options;

struct Motion::Impl
{
    Motion &module;

    //boost::shared_ptr<AL::ALMotionProxy> motionProxy;

    boost::thread *ptr_t;                           //Thread
    boost::posix_time::time_duration samplingPeriod;
    bool stopThread;                                //flag to stop the thread, and predicate against spurious unblocks
    boost::mutex stopThreadLock;                    //lock
    boost::condition_variable condVar;              //mechanism to control the thread's execution
    boost::mutex motion_lock;                       //Lock da locomoção

    po::variables_map vm;

    boost::shared_ptr<MotionAdapter> runswiftMotion;



    Impl(Motion &mod) :  module(mod), samplingPeriod(boost::posix_time::milliseconds(1000/FREQUENCY)), stopThread(false), ptr_t(NULL)
    {
        //motionProxy = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(mod.getParentBroker()));
        //motionProxy->setStiffnesses("HeadYaw", 1.0f);

        po::options_description cmdline_options = store_and_notify(std::vector<string>(0), vm, NULL);

        runswiftMotion = boost::shared_ptr<MotionAdapter>(new MotionAdapter(vm));
    }

    ~Impl()
    {
    
    }

    void operator()()
    {
        bool stopThreadCopy;
        boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
        stopThreadLock.lock();
        stopThreadCopy = stopThread;
        stopThreadLock.unlock();

        boost::unique_lock<boost::mutex> motion_unique_lock(motion_lock); // starts locked

        //float i = 0;

        //thread loop
        while (!stopThreadCopy)
        {
            boost::system_time tickTime = boost::get_system_time();
            //motionProxy->setAngles("HeadYaw", sin(i), 0.5f);
            //qiLogVerbose("Motion") << "Teste de thread" << std::endl;
            //i += 0.01f;

            runswiftMotion->tick();

            // note the time in nanoseconds since Jan 1 1970
            boost::posix_time::time_duration duration = boost::get_system_time() - tickTime;
            long long timestampNanos = duration.total_nanoseconds();

            qiLogInfo("Motion") << "Time: " << timestampNanos << std::endl;

            // calculate next tick time
            tickTime += samplingPeriod;
            // wait for timeout, unlock sockLock while waiting, and control for spurious wakes
            // by checking stopThread.
            condVar.timed_wait(motion_unique_lock, tickTime, boost::lambda::var(stopThread));
            stopThreadLock.lock();
            stopThreadCopy = stopThread;
            stopThreadLock.unlock();
        }
    }
};


Motion::Motion(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName) : AL::ALModule(pBroker, pName)
{
    //Bind the methods so they can be called by other modules
    setModuleDescription("Control Goalkeeper motions and motors.");

    /** How to bind a method:
      * functionName("NAME_OF_THE_METHOD" , getName(), "DESCRIPTION_OF_THE_METHOD");
      * addParam("NAME_OF_THE_PARAMETER_N", "DESCRIPTION_OF_THE_PARAMETER_N");
      * setReturn("TYPE_OF_RETURN", "DESCRIPTION_OF_THE_RETURN");
      * BIND_METHOD(CLASS::METHOD);
      * Exemple:
      * functionName("LeftHandInverseKinematics" , getName(), "Move the left hand to a desired position and orientation using Inverse Kinematics.");
      * addParam("frame", "Vector that contains 12 values: 9 to represent the rotation and 3 to represent the position.");
      * setReturn("boolean", "return true if there is a result to the inverse kinematics calculation.");
      * BIND_METHOD(KinematicsModule::LeftHandInverseKinematics);
    **/

    functionName("StartLoop", getName(), "Start the Motion Thread.");
    BIND_METHOD(Motion::StartLoop);

    functionName("StopLoop", getName(), "Stop the Motion Thread.");
    BIND_METHOD(Motion::StopLoop);
}

Motion::~Motion()
{
    //Continue
}

/**
 * Overrides ALModule::init(). This is called right after the module has been constructed.
 */
void Motion::init()
{
    try
    {
        isActive = false;
        AL::ALModule::init();
    }
    catch (std::exception& e)
    {
        qiLogError("Motion") << "Failed to initialize Motion Module" << e.what() << std::endl;
        exit();
    }
}

/**
 * Overides ALModule::exit(), called before unloading
 */
void Motion::exit()
{
    //Always call ALModule::exit() or the module won't be destroyed properly
    isActive = false;
    AL::ALModule::exit();
}

void Motion::StartLoop()
{
    if(!isActive)
    {
        try
        {
            impl = boost::shared_ptr<Impl>(new Impl(*this));
            boost::thread::attributes attr;
            impl->ptr_t = new boost::thread(boost::ref(*impl));

            pthread_t threadID = (pthread_t) impl->ptr_t->native_handle();

            struct sched_param param;

            int retcode, policy;

            if((retcode = pthread_getschedparam(threadID, &policy, &param)) != 0)
            {
                throw std::runtime_error(std::string("Failed to get thread param"));
            }
            qiLogVerbose("Motion") << "policy: " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" :
                                                  (policy == SCHED_RR) ? "SCHED_RR" :
                                                  (policy == SCHED_OTHER) ? "SCHED_OTHER" :
                                                                            "???")
                                << std::endl << "priority: " << param.sched_priority << std::endl;
            /*policy = SCHED_RR;
            param.sched_priority = 2;

            if((retcode = pthread_setschedparam(threadID, policy, &param)) != 0)
            {
                throw std::runtime_error(std::string("Failed to change priority"));
            }*/

        }
        catch (std::exception& e)
        {
            qiLogError("Motion") << "Failed to start the thread." << e.what() << std::endl;
            throw MotionError("Failed to start the thread.");
            exit();
        }
        isActive = true;
    }
    else
    {
        throw MotionError("Module is already active.");
    }
}

void Motion::StopLoop()
{
    if(isActive)
    {
        try
        {
            impl->stopThreadLock.lock();
            impl->stopThread = true;
            impl->stopThreadLock.unlock();
            impl->condVar.notify_one();
            if (impl->ptr_t)
            {
                impl->ptr_t->join();
            }
            impl.reset();
        }
        catch (std::exception& e)
        {
            qiLogError("Motion") << "Failed to stop the thread." << e.what() << std::endl;
            throw MotionError("Failed to stop the thread.");
            exit();
        }
        isActive = false;
    }
    else
    {
        throw MotionError("Module is already inactive.");
    }
}

//Exception Class

MotionError::MotionError(const char* message)
  : msg(message) {}

MotionError::MotionError(const std::string message)
  : msg(message.c_str()) {}

const char* MotionError::what() { return msg; }
