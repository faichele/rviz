#include "qsingleton.h"

namespace rviz
{
  template<class SingletonType>
  std::unique_ptr<SingletonType> QSingleton<SingletonType>::getInstance()
  {
    QMutex _mutex;

    if (m_instance == nullptr)
    {
      QMutexLocker locker(&_mutex);
      {
        m_instance = new SingletonType();
      }
    }

    return m_instance;
  }

  template<typename SingletonType> std::unique_ptr<SingletonType> QSingleton<SingletonType>::m_instance = nullptr;
}
