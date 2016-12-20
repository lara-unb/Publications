// Local module library initialization code

#ifndef _WIN32
# include <signal.h>
#endif

#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include "motion.hpp"

#ifdef _WIN32
  // export the dll entry point
  #define ALCALL __declspec(dllexport)
#else
  #define ALCALL
#endif

extern "C"
{
  ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {
    // init broker with the main broker instance from the parent executable
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
    // create module instance
    AL::ALModule::createModule<Motion>(pBroker, "Motion");
    return 0;
  }

  ALCALL int _closeModule(  )
  {
    return 0;
  }
}
