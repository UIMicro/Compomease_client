#ifndef _VECTOR2_H
#define _VECTOR2_H
class Vector2 {
private:
    float x1, x2;
public:
    Vector2(float x1, float x2);
    Vector2();
    float Dot(Vector2 object);
    float magnitude();
    Vector2 normalized();
    Vector2 operator+(const Vector2& op);
    Vector2 operator-(const Vector2& op);
    Vector2 operator*(const float op);
    Vector2 operator/(const float op);
    Vector2& operator=(const Vector2& op);
    Vector2& operator+=(const Vector2& op);
    Vector2& operator-=(const Vector2& op);
    float& operator[](const int op);
    friend Vector2 operator*(const float op1, const Vector2& op2);
};
#endif