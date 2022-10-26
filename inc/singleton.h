#ifndef SINGLETON_H
#define SINGLETON_H

template<typename T>
class Singleton
{
public:
    static T& instance() {
        static T t;
        return t;
    }
    Singleton(T&&) = delete;
    Singleton(const T&) = delete;
    void operator=(const T&) = delete;
private:
    Singleton() = default;
    ~Singleton() = default;
};

#endif // SINGLETON_H
