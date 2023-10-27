// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MATERIALS_H
#define MATERIALS_H

#include <limits>
#include <unordered_map>

// pragmas suppress an "ISO C++17 does not allow 'register' storage class
// specifier" warning that occurs multiple times when this header is included
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <OgreColourValue.h>
#pragma GCC diagnostic pop

namespace ow_materials
{

struct Color
{
  Color(float red, float green, float blue) : r(red), g(green), b(blue) { };
  Color(Ogre::ColourValue c) {
    r = c.r * 255.0f;
    g = c.g * 255.0f;
    b = c.b * 255.0f;
  };

  float r, g, b;

  inline bool operator==(const Color &other) {
    return (r == other.r && g == other.g && b == other.b);
  };
  inline bool operator!=(const Color &other) {
    return !(*this == other);
  };
};

using MaterialID = std::uint8_t;

struct Material
{
  constexpr static MaterialID id_min = std::numeric_limits<MaterialID>::min();
  constexpr static MaterialID id_max = std::numeric_limits<MaterialID>::max();

  std::string name;
  float occurrence;
  float science_value;

  double density;

  Color reference_color;
  Color visualize_color;

  // TODO:
  // double cohesion;
  // double friction_angle;

};

// Possible improvements to MaterialBlend:
//   1. Shared contiguous memory for all instances (locality of reference)
//   2. Cache commonly reference values, like blend hardness (computation)

class MaterialBlend {

  // A blend of different materials at concentrations that add up to unity.

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

  void add(MaterialID id, float concentration);

private:
  void divideElementWise(float denominator);

  float sumConcentrations() const;

  BlendType m_blend;
};

}

#endif // MATERIALS_H
