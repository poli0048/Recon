#pragma once

//External Includes
#include "HandyImGuiInclude.hpp"

//This header is obsolete now

inline Eigen::Vector2d ImVecToEigen(ImVec2 const & a) { return Eigen::Vector2d(a.x, a.y); }
inline Eigen::Vector4d ImVecToEigen(ImVec4 const & a) { return Eigen::Vector4d(a.x, a.y, a.z, a.w); }

inline ImVec2 EigenToImVec(Eigen::Vector2d const & a) { return ImVec2(float(a(0)), float(a(1))); }
inline ImVec4 EigenToImVec(Eigen::Vector4d const & a) { return ImVec4(float(a(0)), float(a(1)), float(a(2)), float(a(3))); }
