//Include this header to include Handy and ImGui.
//This header allows for implicit casting between ImVec2/4 and Eigen::Vector2f/2d/4f/4d
//Things need to be included and defined/redefined in the right order for the magic to happen, so include this *instead* of
//directly including handy or ImGui elsewhere to ensure that things happen in the right order.

#include "../../eigen/Eigen/Core"
#include "../../handycpp/Handy.hpp"

#undef IM_VEC2_CLASS_EXTRA
#undef IM_VEC4_CLASS_EXTRA

#define IM_VEC2_CLASS_EXTRA \
FORCEINLINE   ImVec2(HANDYMATH_NS::Vector2   const & v2) : x(v2.X), y(v2.Y) { }              \
FORCEINLINE operator HANDYMATH_NS::Vector2() const { return HANDYMATH_NS::Vector2(x, y); }   \
FORCEINLINE   ImVec2(Eigen::Vector2f         const & v2) : x(v2(0)), y(v2(1)) { }            \
FORCEINLINE operator Eigen::Vector2f()       const { return Eigen::Vector2f(x, y); }         \
FORCEINLINE   ImVec2(Eigen::Vector2d         const & v2) : x(v2(0)), y(v2(1)) { }            \
FORCEINLINE operator Eigen::Vector2d()       const { return Eigen::Vector2d(x, y); }


#define IM_VEC4_CLASS_EXTRA \
FORCEINLINE   ImVec4(HANDYMATH_NS::Vector4   const & v4) : x(v4.X), y(v4.Y), z(v4.Z), w(v4.W) { }      \
FORCEINLINE operator HANDYMATH_NS::Vector4() const { return HANDYMATH_NS::Vector4(x, y, z, w); }       \
FORCEINLINE   ImVec4(Eigen::Vector4f         const & v4) : x(v4(0)), y(v4(1)), z(v4(2)), w(v4(3)) { }  \
FORCEINLINE operator Eigen::Vector4f()       const { return Eigen::Vector4f(x, y, z, w); }             \
FORCEINLINE   ImVec4(Eigen::Vector4d         const & v4) : x(v4(0)), y(v4(1)), z(v4(2)), w(v4(3)) { }  \
FORCEINLINE operator Eigen::Vector4d()       const { return Eigen::Vector4d(x, y, z, w); }

#include "../../imgui/app/ImGuiApp.hpp"
