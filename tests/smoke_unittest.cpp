#include <gtest/gtest.h>
#include <openbeam/openbeam.h>

TEST(Smoke, VersionString) { EXPECT_STRNE(OPENBEAM_VERSION, ""); }
