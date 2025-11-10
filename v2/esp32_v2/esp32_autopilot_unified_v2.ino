/*
  esp32_autopilot_unified_v2.ino
  - Same protocol as v1 with optional checksum: CMD:...*XX
  - Use your existing pins & PWM from v1. This file shows the checksum idea.
*/
#include <Arduino.h>

int checksum(const String& s){
  int sum = 0;
  for (size_t i=0;i<s.length();++i) sum = (sum + (unsigned char)s[i]) & 0xFF;
  return sum;
}

bool parseWithChecksum(String raw, String& out){
  int star = raw.lastIndexOf('*');
  if (star < 0) { out = raw; return true; }
  String data = raw.substring(0, star);
  String tag  = raw.substring(star+1);
  if (tag.length() < 2) return false;
  int want = (int) strtol(tag.substring(0,2).c_str(), NULL, 16);
  if (((int)checksum(data)) == want) { out = data; return true; }
  return false;
}
