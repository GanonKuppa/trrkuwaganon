#pragma once

namespace module {
    template <class T>
    class BaseModule {
      public:
        virtual void update() = 0;
        virtual void setDeltaT(float delta_t) {_delta_t = delta_t;}

        static T& getInstance() {
            static T _instance;
            return _instance;
        }

      protected:
        float _delta_t;
        BaseModule() {}

      private:
        BaseModule(const BaseModule&) = delete;
        BaseModule& operator=(const BaseModule&) = delete;
        BaseModule(BaseModule&&) = delete;
        BaseModule& operator=(BaseModule&&) = delete;

    };
}
