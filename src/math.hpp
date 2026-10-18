#pragma once

#include <cmath>

namespace math {

constexpr float PI = 3.14159265358979323846f;
constexpr float EPSILON = 1e-6f;

inline float radians(float degrees) { return degrees * PI / 180.0f; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }

    Vec3 normalized() const {
        float len = length();
        if (len < EPSILON) return {0.0f, 0.0f, 0.0f};
        return *this / len;
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

// Rotation matrix stored as three column vectors (column-major, OpenGL friendly).
struct Mat3 {
    Vec3 c0{1.0f, 0.0f, 0.0f};
    Vec3 c1{0.0f, 1.0f, 0.0f};
    Vec3 c2{0.0f, 0.0f, 1.0f};

    Mat3() = default;
    Mat3(const Vec3& c0_, const Vec3& c1_, const Vec3& c2_) : c0(c0_), c1(c1_), c2(c2_) {}

    static Mat3 diagonal(const Vec3& d) {
        return Mat3({d.x, 0.0f, 0.0f}, {0.0f, d.y, 0.0f}, {0.0f, 0.0f, d.z});
    }

    // M = a * b^T (outer product), used to build the world inertia tensor.
    static Mat3 outerProduct(const Vec3& a, const Vec3& b) {
        return Mat3({a.x * b.x, a.y * b.x, a.z * b.x},
                    {a.x * b.y, a.y * b.y, a.z * b.y},
                    {a.x * b.z, a.y * b.z, a.z * b.z});
    }

    Mat3 transposed() const {
        return Mat3({c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z});
    }

    Mat3 operator*(float s) const { return Mat3(c0 * s, c1 * s, c2 * s); }
    Mat3 operator+(const Mat3& o) const { return Mat3(c0 + o.c0, c1 + o.c1, c2 + o.c2); }

    Vec3 operator*(const Vec3& v) const { return c0 * v.x + c1 * v.y + c2 * v.z; }
};

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        Vec3 a = axis.normalized();
        float half = angle * 0.5f;
        float s = std::sin(half);
        return {std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    Quat normalized() const {
        float len = std::sqrt(w * w + x * x + y * y + z * z);
        if (len < EPSILON) return {};
        return {w / len, x / len, y / len, z / len};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat operator+(const Quat& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }
    Quat operator*(float s) const { return {w * s, x * s, y * s, z * s}; }

    // Hamilton product: result applies "o" first, then "this".
    Quat operator*(const Quat& o) const {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    Vec3 rotate(const Vec3& v) const {
        Vec3 qv{x, y, z};
        Vec3 t = cross(qv, v) * 2.0f;
        return v + t * w + cross(qv, t);
    }

    Mat3 toMat3() const {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        return Mat3(
            {1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
            {2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
            {2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}
        );
    }
};

// 4x4 matrix, column-major layout (same convention as OpenGL).
struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static Mat4 identity() { return Mat4{}; }

    static Mat4 translation(const Vec3& t) {
        Mat4 r;
        r.m[12] = t.x;
        r.m[13] = t.y;
        r.m[14] = t.z;
        return r;
    }

    static Mat4 scale(const Vec3& s) {
        Mat4 r;
        r.m[0] = s.x;
        r.m[5] = s.y;
        r.m[10] = s.z;
        return r;
    }

    static Mat4 rotation(const Quat& q) {
        Mat3 rot = q.toMat3();
        Mat4 r;
        r.m[0] = rot.c0.x; r.m[1] = rot.c0.y; r.m[2] = rot.c0.z; r.m[3] = 0.0f;
        r.m[4] = rot.c1.x; r.m[5] = rot.c1.y; r.m[6] = rot.c1.z; r.m[7] = 0.0f;
        r.m[8] = rot.c2.x; r.m[9] = rot.c2.y; r.m[10] = rot.c2.z; r.m[11] = 0.0f;
        r.m[12] = 0.0f; r.m[13] = 0.0f; r.m[14] = 0.0f; r.m[15] = 1.0f;
        return r;
    }

    static Mat4 perspective(float fovyRadians, float aspect, float nearZ, float farZ) {
        float f = 1.0f / std::tan(fovyRadians * 0.5f);
        Mat4 r;
        for (float& v : r.m) v = 0.0f;
        r.m[0] = f / aspect;
        r.m[5] = f;
        r.m[10] = (farZ + nearZ) / (nearZ - farZ);
        r.m[11] = -1.0f;
        r.m[14] = (2.0f * farZ * nearZ) / (nearZ - farZ);
        return r;
    }

    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = (center - eye).normalized();
        Vec3 s = cross(f, up).normalized();
        Vec3 u = cross(s, f);
        Mat4 r;
        r.m[0] = s.x; r.m[4] = s.y; r.m[8]  = s.z; r.m[12] = -dot(s, eye);
        r.m[1] = u.x; r.m[5] = u.y; r.m[9]  = u.z; r.m[13] = -dot(u, eye);
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z; r.m[14] = dot(f, eye);
        r.m[3] = 0.0f; r.m[7] = 0.0f; r.m[11] = 0.0f; r.m[15] = 1.0f;
        return r;
    }

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                    sum += m[k * 4 + row] * o.m[col * 4 + k];
                r.m[col * 4 + row] = sum;
            }
        }
        return r;
    }
};

} // namespace math
