#include "bodies_manager.hpp"
#include <stdexcept>

size_t BodiesManager::AddSoftBody(const SoftBody& body) {
    mSoftBodies.push_back(body);
    return mSoftBodies.size() - 1;
}

void BodiesManager::RemoveSoftBody(size_t index) {
    if (index >= mSoftBodies.size())
        throw std::out_of_range("Invalid body index");

    mSoftBodies.erase(mSoftBodies.begin() + index);
}

void BodiesManager::Clear() {
    mSoftBodies.clear();
}

std::vector<SoftBody>& BodiesManager::GetSoftBodies() {
    return mSoftBodies;
}

const std::vector<SoftBody>& BodiesManager::GetSoftBodies() const {
    return mSoftBodies;
}

SoftBody& BodiesManager::GetSoftBody(size_t index) {
    if (index >= mSoftBodies.size())
        throw std::out_of_range("Invalid body index");

    return mSoftBodies[index];
}

const SoftBody& BodiesManager::GetSoftBody(size_t index) const {
    if (index >= mSoftBodies.size())
        throw std::out_of_range("Invalid body index");

    return mSoftBodies[index];
}

size_t BodiesManager::GetCount() const {
    return mSoftBodies.size();
}
