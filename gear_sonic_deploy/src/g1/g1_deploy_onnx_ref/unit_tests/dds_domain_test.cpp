#include <gtest/gtest.h>

#include "dds_domain.hpp"

// A typo'd domain must be refused, never run on some other domain: that is the
// cross-talk --dds-domain exists to prevent.
TEST(DdsDomain, AcceptsTheWholeRange) {
  int domain = -1;
  EXPECT_TRUE(parse_dds_domain("0", domain));
  EXPECT_EQ(domain, 0);
  EXPECT_TRUE(parse_dds_domain("1", domain));
  EXPECT_EQ(domain, 1);
  EXPECT_TRUE(parse_dds_domain("232", domain));
  EXPECT_EQ(domain, 232);
}

TEST(DdsDomain, RejectsNonIntegers) {
  int domain = 7;
  for (const char* text : {"1abc", "abc", "", " 1", "1 ", "1.5", "+1", "0x1", "1e1"}) {
    EXPECT_FALSE(parse_dds_domain(text, domain)) << text;
  }
  EXPECT_EQ(domain, 7);  // a refused value leaves the caller's domain alone
}

TEST(DdsDomain, RejectsOutOfRange) {
  int domain = 7;
  for (const char* text : {"-1", "233", "999", "99999999999999999999"}) {
    EXPECT_FALSE(parse_dds_domain(text, domain)) << text;
  }
  EXPECT_EQ(domain, 7);
}
