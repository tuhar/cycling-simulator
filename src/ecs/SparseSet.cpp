#include <ecs/SparseSet.hpp>

bool SparseSet::contains(Entity e) {
  if (e >= sparseArray.size() || sparseArray[e] >= denseArray.size()) {
    return false;
  }
  return denseArray[sparseArray[e]] == e;
}

void SparseSet::add(Entity e) {
  size_t size = denseArray.size();
  denseArray.push_back(e);
  if (e >= sparseArray.size()) {
    sparseArray.resize(e + 1);  // use sentinel value
  }
  sparseArray[e] = size;
}

void SparseSet::remove(Entity e) {
  if (e >= sparseArray.size() || !contains(e)) {
    return;
  }
  size_t lastIndex = denseArray.size() - 1;
  Entity lastEntity = denseArray[lastIndex];
  std::swap(denseArray[sparseArray[e]], denseArray[lastIndex]);
  std::swap(sparseArray[e], sparseArray[lastEntity]);
  denseArray.pop_back();
}