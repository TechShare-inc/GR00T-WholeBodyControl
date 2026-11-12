#pragma once

#include <charconv>
#include <cstring>

// Parses a --dds-domain value: a plain decimal integer in [0, 232] (the DDS domain
// range). Anything else, such as "1abc", returns false and leaves `domain` unchanged,
// so a typo can never land the deploy on the wrong domain.
inline bool parse_dds_domain(const char* text, int& domain) {
  int value = 0;
  const char* end = text + std::strlen(text);
  const auto [stop, error] = std::from_chars(text, end, value);
  if (error != std::errc() || stop != end || value < 0 || value > 232) {
    return false;
  }
  domain = value;
  return true;
}
