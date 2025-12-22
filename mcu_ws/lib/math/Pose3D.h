#ifndef POSE3D_H
#define POSE3D_H

class Pose3D {
 public:
  float x;
  float y;
  float z;

  // quaternion orientation
  float qx;
  float qy;
  float qz;
  float qw;

  ~Pose3D() = default;
  Pose3D(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f, float _qx = 0.0f,
         float _qy = 0.0f, float _qz = 0.0f, float _qw = 0.0f)
      : x(_x), y(_y), z(_z), qx(_qx), qy(_qy), qz(_qz), qw(_qw) {};

  float getX() { return x; }
  float getY() { return y; }
  float getZ() { return z; }

  float getQx() { return qx; }
  float getQy() { return qy; }
  float getQz() { return qz; }
  float getQw() { return qw; }

 private:
};

#endif