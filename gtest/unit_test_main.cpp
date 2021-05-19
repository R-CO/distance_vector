/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
#include <gtest/gtest.h>

#include <cstdlib>

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  if (RUN_ALL_TESTS() != 0) {
    // some test-cases failed
  }

  return EXIT_SUCCESS;
}