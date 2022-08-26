#ifndef QSINGLETON_H
#define QSINGLETON_H

#include <QMutex>
#include <memory>
#ifndef Q_MOC_RUN
#include <rviz/rviz_export.h>
#endif

namespace rviz
{
    template <class T>
    class QSingleton
    {
      /// \brief Get an instance of the singleton
      public: static T *Instance()
              {
                return &GetInstance();
              }

      /// \brief Constructor
      protected: QSingleton() {}

      /// \brief Destructor
      protected: virtual ~QSingleton() {}

      /// \brief Creates and returns a reference to the unique (static) instance
      private: static T &GetInstance()
               {
                 static T t;
                 return static_cast<T &>(t);
               }
    };
}

#endif // QSINGLETON_H
