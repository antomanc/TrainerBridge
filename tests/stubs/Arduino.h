#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <cstdlib>
#include <cmath>
inline unsigned long testNow = 0;
inline unsigned long millis() { return testNow; }
inline void delay(unsigned long ms) { testNow += ms; }
constexpr int OUTPUT=1, HIGH=1, LOW=0, HEX=16;
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
class String {
  std::string s;
public:
  String(const char* v=""): s(v) {}
  size_t length() const { return s.size(); }
  void toLowerCase() { std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return std::tolower(c);}); }
  int indexOf(const String& v) const { auto p=s.find(v.s); return p==std::string::npos?-1:static_cast<int>(p); }
  bool operator==(const String& v) const { return s==v.s; }
  bool startsWith(const char* v) const { return s.rfind(v,0)==0; }
  String substring(size_t i) const { return String(s.substr(i).c_str()); }
  void trim() { auto a=s.find_first_not_of(" \t\r\n"); if(a==std::string::npos) {s.clear(); return;} s=s.substr(a,s.find_last_not_of(" \t\r\n")-a+1); }
  float toFloat() const { return std::strtof(s.c_str(),nullptr); }
  int toInt() const { return std::atoi(s.c_str()); }
};
struct TestSerial {
  void begin(int) {}
  template<class... T> void print(T...) {}
  template<class... T> void println(T...) {}
  template<class... T> void printf(T...) {}
  int available() { return 0; }
  int read() { return 0; }
};
inline TestSerial Serial;
