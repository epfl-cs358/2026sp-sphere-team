#pragma once

namespace imu {

template <int N>
class Vector {
public:
    Vector() { for (int i = 0; i < N; i++) _data[i] = 0.0; }
    Vector(double a, double b, double c) { _data[0] = a; _data[1] = b; _data[2] = c; }
    double x() const { return _data[0]; }
    double y() const { return _data[1]; }
    double z() const { return _data[2]; }
private:
    double _data[N];
};

class Quaternion {
public:
    Quaternion() : _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}
    Quaternion(double w, double x, double y, double z) : _w(w), _x(x), _y(y), _z(z) {}
    double w() const { return _w; }
    double x() const { return _x; }
    double y() const { return _y; }
    double z() const { return _z; }
private:
    double _w, _x, _y, _z;
};

} // namespace imu
