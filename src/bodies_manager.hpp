#pragma once
#include <vector>
#include "soft_body.hpp"

class BodiesManager {
public:
    size_t AddSoftBody(const SoftBody& body);

    void RemoveSoftBody(size_t index);

    void Clear();

    std::vector<SoftBody>& GetSoftBodies();
    const std::vector<SoftBody>& GetSoftBodies() const;

    SoftBody& GetSoftBody(size_t index);
    const SoftBody& GetSoftBody(size_t index) const;

    size_t GetCount() const;

private:
    std::vector<SoftBody> mSoftBodies;
};
