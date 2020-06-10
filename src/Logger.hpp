#ifndef LOG_PLAN_H
#define LOG_PLAN_H
#define LOG_PLAN(...) PlanTrace(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#include<iostream>



void privTrace() {
    std::cout << std::endl;
}
template <typename T, typename... Args>
void privTrace(T t, Args... args)
{
    std::cout << t << " ";
    privTrace(args...);
}
template <typename ...Args>
void PlanTrace(const char* file, const char* func, const int line, Args&& ...args)
{
    std::cout << '[' << file << '|' << func << '@' << line << "]:\t";
    privTrace(args...);
}
#endif
