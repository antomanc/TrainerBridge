#include "TrainerMatcher.h"

bool TrainerMatcher::stringContainsIgnoreCase(String source, String needle)
{
  if (needle.length() == 0) return false;

  source.toLowerCase();
  needle.toLowerCase();

  return source.indexOf(needle) >= 0;
}

bool TrainerMatcher::addressesMatch(const String &a, const String &b)
{
  String lowerA = a;
  String lowerB = b;
  lowerA.toLowerCase();
  lowerB.toLowerCase();
  return lowerA == lowerB;
}

bool TrainerMatcher::matches(
  const String &name,
  const String &address,
  bool advertisesFtms,
  const char *targetNameContains,
  const char *targetMac
)
{
  if (targetMac != nullptr && strlen(targetMac) > 0)
  {
    return addressesMatch(address, String(targetMac));
  }

  if (targetNameContains != nullptr && strlen(targetNameContains) > 0)
  {
    return stringContainsIgnoreCase(name, String(targetNameContains));
  }

  return advertisesFtms;
}

bool TrainerMatcher::selfCheck()
{
  return matches("VanRysel D500", "AA:BB", false, "RYSEL", "") &&
         matches("Van Rysel D500", "AA:BB", false, "RYSEL", "") &&
         matches("Varysel D500", "AA:BB", false, "RYSEL", "") &&
         !matches("Other trainer", "AA:BB", true, "RYSEL", "") &&
         matches("Other", "AA:BB", false, "VANRYSEL", "aa:bb") &&
         !matches("VanRysel D500", "CC:DD", true, "VANRYSEL", "aa:bb") &&
         matches("Other", "AA:BB", true, "", "");
}
