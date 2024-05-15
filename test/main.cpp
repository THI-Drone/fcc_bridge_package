//
// Created by Johan <job8197@thi.de> on 10.05.2024.
//

#include "setup.hpp"
#include "test_fixtures.hpp"

int main(int argc, char **argv) {
    fcc_bridge::test::setup::setup();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
