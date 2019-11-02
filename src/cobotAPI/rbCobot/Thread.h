#ifndef THREAD_H
#define THREAD_H

#include <functional>
#include <thread>
class Thread
{

    std::thread mThread;
protected:
    bool mflagRequestRelease;
    double mWaitTime;
    std::string name;

    std::function <void()> mFuncUpdate;

public:
    Thread()
    {

    }
    virtual ~Thread()
    {

    }
    void setName(const char *name)
    {
        this->name = name;
    }
    void Join()
    {
        if(mThread.joinable())
        {
            mThread.join();
        }
    }

    void start(double waitTime = 0.01)
    {
        mWaitTime = waitTime;
        createThread();
    }
    void resume();
    virtual void release()
    {
        if(mflagRequestRelease == false)
        {
            mflagRequestRelease = true;
        }
    }
    void SetUpdateFn(std::function <void()> fn)
    {
        mFuncUpdate = fn;
    }

public:
    static double HzToSec(int hertz)
    {
        return 1.0 / hertz;
    }

protected:
    virtual void onInit() = 0;
    virtual void onRelease() = 0;

    virtual void onUpdate(double deltaTime)
    {
        if(mFuncUpdate != nullptr)
        {
            mFuncUpdate();
        }
    }

    virtual void printUpdate()
    {
        printf("This thread is running\n");
    }
    void createThread()
    {
        std::thread t = std::thread([&]() {
            onInit();
            mflagRequestRelease = false;
            //long nano = (long)(mWaitTime * 100000000000);
            //wait 10 ms
            long nano = (long)(mWaitTime * 1000000000);
            std::chrono::nanoseconds waitTime(nano);
            std::chrono::nanoseconds JitterTime(0);

            while (!mflagRequestRelease)
            {
                if(mWaitTime > 0.0)
                {
                    std::chrono::high_resolution_clock::time_point prevClock = std::chrono::high_resolution_clock::now();

                    std::this_thread::sleep_until(prevClock + waitTime);

                    std::chrono::high_resolution_clock::time_point curClock = std::chrono::high_resolution_clock::now();
                    std::chrono::nanoseconds duration = (curClock - prevClock);
                    JitterTime = waitTime - duration;
                    onUpdate(std::chrono::duration<double>(duration).count());
                }
                else
                {
                    onUpdate(0.0);
                }
                //printf("!! %d\n", (int)mflagRequestRelease);

            }

            onRelease();
        });

        mThread.swap(t);

    }
};

#endif
