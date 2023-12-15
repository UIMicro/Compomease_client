#include "Vector2.h"
#include <cmath>

float Vector2::Dot(Vector2 object) {
    return x1 * object.x1 + x2 * object.x2;
}

float Vector2::magnitude() {
    return sqrt(x1 * x1 + x2 * x2);
}

Vector2::Vector2(float x1, float x2) : x1(x1), x2(x2) {

}

Vector2::Vector2() : x1(0), x2(0){
    
}

Vector2 Vector2::normalized() {
    float mag = magnitude();
    return Vector2(x1 / mag, x2 / mag);
}

Vector2 Vector2::operator+(const Vector2& op) {
    return Vector2(x1 + op.x1, x2 + op.x2);
}
Vector2& Vector2::operator+=(const Vector2& op) {
    x1 = x1 + op.x1;
    x2 = x2 + op.x2;
    return *this;
}
Vector2& Vector2::operator-=(const Vector2& op) {
    x1 = x1 - op.x1;
    x2 = x2 - op.x2;
    return *this;
}
Vector2 Vector2::operator-(const Vector2& op) {
    return Vector2(x1 - op.x1, x2 - op.x2);
}

Vector2 Vector2::operator*(const float op) {
    return Vector2(x1 * op, x2 * op);
}
Vector2 Vector2::operator/(const float op) {
    return Vector2(x1 / op, x2 / op);
}

float& Vector2::operator[](const int op) {
    return (op == 0) ? x1 : x2;
}

Vector2& Vector2::operator=(const Vector2& op) {
  x1 = op.x1;
  x2 = op.x2;
  return *this;
}
Vector2 operator*(const float op1, const Vector2& op2) {
  return Vector2(op1 * op2.x1, op1 * op2.x2);
}