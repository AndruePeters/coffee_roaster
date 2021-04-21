#ifndef ALGORITHMS_H
#define ALGORITHMS_H

namespace algorithms {

template<class T> 
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

template<class T> 
const T& min(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

template<class T>
const T& clamp(const T& val, const T& low, const T& high)
{
  return max(low, min(val, high));
}

} /// namespace algorithms
#endif
