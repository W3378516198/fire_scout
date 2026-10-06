#pragma once
#include <string>
#include <type_traits>
namespace fire_scout {
template<class T,class=void>struct MessageVersion {static constexpr unsigned value=0;};
template<class T>struct MessageVersion<T,std::void_t<decltype(T::MESSAGE_VERSION)>>{
  static constexpr unsigned value=T::MESSAGE_VERSION;
};
template<class T>std::string px4Topic(const std::string&base){
  constexpr auto version=MessageVersion<T>::value;
  return version?base+"_v"+std::to_string(version):base;
}
}
