#include <catch2/catch_test_macros.hpp>
#include <ecs/SparseSet.hpp>
#include <iostream>

TEST_CASE("Add sanity") {
  SparseSet testSet;
  Entity testEntity = 4;
  testSet.add(testEntity);

  REQUIRE(testSet.sparseArray[testEntity] == 0);
  REQUIRE(testSet.denseArray[0] == testEntity);

  Entity testEntity2 = 2;
  testSet.add(testEntity2);

  REQUIRE(testSet.sparseArray[testEntity2] == 1);
  REQUIRE(testSet.denseArray[1] == testEntity2);
}

TEST_CASE("Contains sanity") {
  SparseSet testSet;
  Entity testEntity = 4;

  REQUIRE(testSet.contains(testEntity) == false);
  testSet.add(testEntity);
  REQUIRE(testSet.contains(testEntity) == true);
}

TEST_CASE("Remove sanity") {
  SparseSet testSet;
  Entity testEntity = 7;
  REQUIRE_NOTHROW(testSet.remove(testEntity));
  testSet.add(testEntity);
  REQUIRE(testSet.contains(testEntity) == true);
  testSet.remove(testEntity);
  REQUIRE(testSet.contains(testEntity) == false);
}

TEST_CASE("Simulated run") {
  SparseSet testSet;
  std::vector<Entity> riders = {0, 2, 3, 5};

  for (size_t i = 0; i < riders.size(); i++) {
    testSet.add(riders[i]);

    REQUIRE(testSet.sparseArray[riders[i]] == i);
    REQUIRE(testSet.denseArray[i] == riders[i]);
  }

  // sparse [0,-,1,2,-,3]
  // dense [0,2,3,5]
  // 3rd rider finished, sparse array swapped index with 4th rider, dense array swapped with last rider
  testSet.remove(riders[2]);
  REQUIRE(testSet.contains(riders[2]) == false);
  REQUIRE(testSet.sparseArray[riders[3]] == 2);
  REQUIRE(testSet.denseArray[2] == riders[3]);

  // sparse [0,-,1,3,-,2]
  // dense [0,2,5]
  // 1st rider finished, sparse array swapped index with 4th rider, dense array swapped with last rider
  testSet.remove(riders[0]);
  REQUIRE(testSet.contains(riders[0]) == false);
  REQUIRE(testSet.sparseArray[riders[3]] == 0);
  REQUIRE(testSet.denseArray[0] == riders[3]);

  // sparse [2,-,1,3,-,0]
  // dense [5,2]
  // 2nd rider finished, sparse array swapped index with 2nd rider, dense array swapped with last rider
  testSet.remove(riders[1]);
  REQUIRE(testSet.contains(riders[1]) == false);
  REQUIRE(testSet.sparseArray[riders[3]] == 0);
  REQUIRE(testSet.denseArray[0] == riders[3]);

  // sparse [2,-,1,3,-,0]
  // dense [5]
  // 4th rider finished, dense array empty
  testSet.remove(riders[3]);
  REQUIRE(testSet.contains(riders[3]) == false);
  REQUIRE(testSet.denseArray.size() == 0);
}