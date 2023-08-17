// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIALS_H
#define MATERIALS_H

#include <limits>
#include <unordered_map>

namespace ow_materials
{

using MaterialID = std::uint8_t;

struct Color
{
  double r, g, b;
};

struct Material
{
  constexpr static MaterialID id_min = std::numeric_limits<MaterialID>::min();
  constexpr static MaterialID id_max = std::numeric_limits<MaterialID>::max();

  std::string name;
  float occurrence;
  float science_value;

  double density;

  Color color;

  // TODO:
  // double cohesion;
  // double friction_angle;

};

// Possible improvements to MaterialBlend:
//   1. Shared contiguous memory for all instances (locality of reference)
//   2. Cache commonly reference values, like blend hardness (computation)

class MaterialBlend {
public:
  MaterialBlend() = default;
  ~MaterialBlend() = default;
  MaterialBlend(const MaterialBlend&) = default;
  MaterialBlend& operator=(const MaterialBlend&) = default;

  using BlendType = std::unordered_map<MaterialID, float>;

  inline BlendType const &getBlendMap() const {
    return m_blend;
  }

  inline BlendType &getBlendMap() {
    return m_blend;
  }

  bool isNormalized() const;

  void normalize();

  void merge(MaterialBlend const &other);

private:
  void divideElementWise(float denominator);

  float sumConcentrations() const;

  BlendType m_blend;
};

}

#endif // MATERIALS_H
